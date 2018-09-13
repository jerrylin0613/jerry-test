//--------------------------------------------------------------------------
// Refer Specification:
//
//   TypeC1.1    : USB Type-C Specification Release 1.1.pdf
//   TypeC1.2    : USB Type-C Specification Release 1.2.pdf
//   TypeC_FT0.73: USB_Type_C_Functional_Test_Specification_2016_06_24.pdf
//
//   TCPC1.0     : USB-Port Controller Specification R1.0 [21051020].pdf
//
//   PD2 V1.1    : USB_PD_R2_0 V1.1 - 20150507.pdf
//   PD2 V1.2    : USB_PD_R2_0 V1.2 -20160325.pdf
//   PD2 V1.2 ECN: USB_PD_R2_0 V1.2 -20160325 - ECN clean markup 20160802.pdf
// 
//--------------------------------------------------------------------------

#include "utility.h"
#include "exrom_jm.h"

//--------------------------------------------------------------------------

#if (JM_USB_PD_ENABLE)

//--------------------------------------------------------------------------

#pragma SAVE        // save current optimization level
#pragma OPTIMIZE(9) // prevent common subroutine optimization

//--------------------------------------------------------------------------

#include "dp80390.h"
#include "global.h"
#include "constant.h"

#include "rs232.h"
#include "exrom_main.h"
#include "exrom_i2c.h"
#include "exrom_tcpm_comm.h"
#include "exrom_tcpm_bank.h"
#include "main.h"
#include "utility.h"
#include "register.h"

void usb_enter_lowest_power(void);
#define aUSB3_LTSSM_CTRL1           (*(UBYTE volatile xdata*)0x5082)    // Link FSM Control1

// Sink Only ---------------------------------------------------------------
//TASKP          xdata _PE_task;
TASKP          xdata _PE_state;
TASKP          xdata _msg_state;
unsigned char ErrorRecovery_Timer_Flag;

unsigned char  xdata U8_temp;
unsigned short xdata U16_temp;
unsigned char  xdata msg_Received;
unsigned char  xdata msg_Transmit;
unsigned char  xdata msg_trans_NG;
unsigned char  xdata msg_Disc;
unsigned char  xdata msg_trans_OK;
unsigned char  xdata VBUS_Alarm_Hi;
unsigned char  xdata VBUS_Alarm_Lo;

// Sink Only ---------------------------------------------------------------

// Protocol Layer TX
void tx_wait_for_message_request(void);
void tx_construct_vdm_message(void);
void tx_construct_message(void);
void tx_vdm_check_alert_or_receiving(void);
void tx_check_alert_or_receiving(void);
void tx_wait_for_phy_response(void);
void tx_hr_request_hard_reset(void);
void tx_hr_disable_tcpc_receiver(void);
void tx_hr_wait_for_phy_hard_reset_complete(void);
void tx_hr_wait_for_pe_hard_reset_complete(void);

// AC Adapter
void ac_unattached(void);
void ac_wait_adapter_coming(void);
void ac_wait_debounce_timeout(void);
void ac_ready(void);
void ac_wait_detached(void);
void ac_detached(void);

// Hard Disk Drive
void hd_clr_busy_wait_turn_on(void);
void hd_wait_turn_on(void);
void hd_wait_timeout(void);
void hd_turn_on_disk(void);
void hd_wait_usb_turn_on_hdd(void);
void hd_wait_hdd_start_finished(void);
void hd_wait_partner_accepted_hdd_starting_pdo(void);
void hd_wait_hdd_start_finished(void);
void hd_wait_partner_accepted_hdd_idle_pdo(void);
void hd_wait_partner_accepted_hdd_stop_pdo(void);
void hd_ready(void);
void hd_req_usb_turn_off_hdd(void);
void hd_wait_usb_turn_off_hdd(void);
void hd_usb_turn_off_hdd(void);

//
void init_tcpc(void);

void request_turn_on_disk(unsigned char dwcnt_100ms);
void request_turn_off_disk(void);

// AC adapter detached timer
#define read_ac_detached_timer_100ms()           read_mcu_dwcnt_100ms_ac()
#define write_ac_detached_timer_100ms(value)     write_mcu_dwcnt_100ms_ac(value)

// AC adapter debounce timer
#define read_ac_debounce_timer_100ms()           read_mcu_dwcnt_100ms_ac()
#define write_ac_debounce_timer_100ms(value)     write_mcu_dwcnt_100ms_ac(value)

// turn on/off hdd timer
#define read_turn_on_disk_timer_100ms()          read_mcu_dwcnt_100ms_hd()
#define write_turn_on_disk_timer_100ms(value)    write_mcu_dwcnt_100ms_hd(value)

//--------------------------------------------------------------------------

// table for initial tcpc
unsigned char xdata reg_tab_size;
const unsigned char code init_reg_tab[][2] = 
    { 
      {0x9B,                                    0X20},  // sleep mode to active mode
      {RT1711P_REG_UNLOCK_PW1,                  0X62}, 
      {RT1711P_REG_UNLOCK_PW2,                  0X86},
      {RT1711P_REG_BMCIO_RXDZSEL0,              0X80},  //
      {0X87,                                    0XDE},
      {0X88,                                    0X5C},
      {TCPC_REG_ROLE_CONTROL,                   0X0A},
      {TCPC_REG_FAULT_CONTROL,                  0X84},  // disable OCP, enable OVP
      {RT1711P_REG_TTCPC_FILTER,                0X05},
      {RT1711P_REG_DRP_TOGGLE_CYCLE,            0X04},
      {RT1711P_REG_DRP_DUTY_CTRL,               0X34},  // dcSRC.DRP[9:0]/1024, 30%=0x134, 70%=0x2CC
      {RT1711P_REG_DRP_DUTY_CTRL+1,             0X01},
      {RT1711P_REG_PHY_CTRL1,                   0X31},  // bit7(ENRETRY_DISCARD)=0, assert discard during reply goodcrc
      {RT1711P_REG_PHY_CTRL3,                   0X70},
      {TCPC_REG_RECEIVE_BYTE_COUNT,             0X00},
      {TCPC_REG_ALERT_MASK,                     0XFF},
      {TCPC_REG_ALERT_MASK+1,                   0X07}, // disable sink disconnect detect
      {TCPC_REG_POWER_STATUS_MASK,              0X06},
      {TCPC_REG_FAULT_STATUS_MASK,              0XFF},
      {TCPC_REG_COMMAND,                        0x33}, // enable vbus detect
      {TCPC_REG_RECEIVE_DETECT,                 0x00}, // disable receive detect
      {TCPC_REG_POWER_CONTROL,                  0x00}, // RT1711P Ver.D default is 0x18(Enable bleed discharge), disable AutoDischargeDisconnect 
      {TCPC_REG_VBUS_VOLTAGE_ALARM_HI_CFG,      0XFF},
      {TCPC_REG_VBUS_VOLTAGE_ALARM_HI_CFG+1,    0X03},
      {RT1711P_REG_PHY_CTRL4,                   0XC0},  // tReceive_L, 0x0A50=2640, 2640/2.4MHz=1.1ms; 0x0960=2400, 2400/2.4MHz=1.0ms 
      {RT1711P_REG_PHY_CTRL4+1,                 0X08},  // tReceive_H, 0x08C0 for Ver.E, 0x08C0=2240, 2240/2400=0.933ms
      {0x9C,                                    0X80},  // enable chip reset if Alert pin 200ms timeout
      {RT1711P_REG_DIS_SNK_VBUS_GP_EN,          0xAA},  // sink   role gate control for AutoDischargeDisconnect 
      {RT1711P_REG_DIS_SRC_VBUS_GP_EN,          0xAA},  // source role gate control for AutoDischargeDisconnect   
      {RT1711P_REG_OCP_OVP_VBUS_GP_EN,          0x0A},  // gate control for OVP, OCP     
      {0XA5,                                    0X00},  // Set GPx as control NMOS
      {0XAA,                                    0XC0},  // Enable charge pump for GPx       
      {RT1711P_REG_VBUS_OVPR_DISCHG,            0XAF},  // OVP = 20V (Max)
      {RT1711P_REG_VBUS_OCPR,                   0X29},  // 0CP = 5A  (Max)
      {0XAE,                                    0X06},
      {RT1711P_REG_VDCLEVEL,                    0XC5},  // VDC threshlod = 15v
      }; 

 
// source fixed pdo, PD2 V1.1 p155
const unsigned char  code src_fix_pdo_shift[]={    0,   10,  20,  22,  25,  26,  27,  28,  29,  30};
const unsigned short code src_fix_pdo_mask[] ={0x3FF,0x3FF,0x03,0x07,0x01,0x01,0x01,0x01,0x01,0x03};

// request pdo, PD2 V1.1 p160
const unsigned char  code req_pdo_shift[]={     0,    10,  20,  24,  25,  26,  27,  28,  31};
const unsigned short code req_pdo_mask[] ={0x03FF,0x03FF,0x0F,0x01,0x01,0x01,0x01,0x07,0x01};

// sink fixed pdo, PD2 V1.1 p158
const unsigned char  code snk_fix_pdo_shift[]={    0,     10,  20,  25,  26,  27,  28,  29,  30};
const unsigned short code snk_fix_pdo_mask[] ={0x03FF,0x03FF,0x1F,0x01,0x01,0x01,0x01,0x01,0x03};

// source/sink variable or battery pdo, PD2 V1.1 p157
const unsigned char  code var_bat_pdo_shift[]={    0,   10,   20,  30};
const unsigned short code var_bat_pdo_mask[] ={0x3FF,0x3FF,0x3FF,0x03};

#if BIST_SUPPORT
// BIST header, PD2 V1.1 p165
const unsigned char  code bist_header_shift[]={     0,    16,  28};
const unsigned short code bist_header_mask[] ={0xFFFF,0x0FFF,0x0F};
#endif

const char* code msg_id2string[32] = {
    // 0~15 is Control Message
    "0000"     , // 0
    "Good"     , // 1
    "Goto"     , // 2
    "Acc"      , // 3
    "Rej"      , // 4
    "Pi"       , // 5
    "PS"       , // 6
    "G_Src_C"  , // 7
    "G_Snk_C"  , // 8
    "DR"       , // 9
    "PR"       , // 10
    "VCON"     , // 11
    "Wait"     , // 12
    "SRST"     , // 13
    "1110"     , // 14
    "1111"     , // 15
    // 16~31 stores Data Message
    "0000"     , // 16
    "Src_C"    , // 17
    "Req"      , // 18
    "BIST"     , // 19
    "Snk_C"    , // 20
    "0101"     , // 21       
    "0110"     , // 22
    "0111"     , // 23
    "1000"     , // 24
    "1001"     , // 25
    "1010"     , // 26
    "1011"     , // 27
    "1100"     , // 28
    "1101"     , // 29
    "1110"     , // 30
    "VDM"      , // 31
    };

//--------------------------------------------------------------------------

// local use
bit timeout;
bit GiveBackFlag;

// RT1711P Device ID
#define RT1711P_DEV_ID_B 0x2091
#define RT1711P_DEV_ID_D 0x2093
#define RT1711P_DEV_ID_E 0x2094
// XDATA Port Memory Map
#define MEM_ADDR_SIZE   0x0120
#define MEM_ADDR_PORTX  0x4500
#define MEM_ADDR_PORT0  (MEM_ADDR_PORTX + MEM_ADDR_SIZE)
#define MEM_ADDR_PORT1  (MEM_ADDR_PORT0 + MEM_ADDR_SIZE)

// port memory
unsigned char xdata port0_mem[MEM_ADDR_SIZE] _at_ MEM_ADDR_PORT0;
unsigned char xdata port1_mem[MEM_ADDR_SIZE] _at_ MEM_ADDR_PORT1;

// flag, allocate in 0~287
// Caution: every bytes shall be allocated by using _at_ 
unsigned char  xdata msg_sent                 _at_ MEM_ADDR_PORTX + 0;
unsigned char  xdata msg_failed               _at_ MEM_ADDR_PORTX + 1;
unsigned char  xdata msg_discarded            _at_ MEM_ADDR_PORTX + 2;
unsigned char  xdata reserved003              _at_ MEM_ADDR_PORTX + 3;  // reserved 
unsigned char  xdata reserved004              _at_ MEM_ADDR_PORTX + 4;  // reserved 
unsigned char  xdata hrst_sent                _at_ MEM_ADDR_PORTX + 5;
unsigned char  xdata vdm_pending              _at_ MEM_ADDR_PORTX + 6;
unsigned char  xdata pd_connected             _at_ MEM_ADDR_PORTX + 7;
unsigned char  xdata vbus_present             _at_ MEM_ADDR_PORTX + 8;
unsigned char  xdata vconn_present            _at_ MEM_ADDR_PORTX + 9;
unsigned char  xdata vsafe_5v                 _at_ MEM_ADDR_PORTX + 10;
unsigned char  xdata vsafe_0v                 _at_ MEM_ADDR_PORTX + 11;
unsigned short xdata vbus_target_25mv         _at_ MEM_ADDR_PORTX + 12;  // 2 bytes
unsigned short xdata vbus_present_25mv        _at_ MEM_ADDR_PORTX + 14;  // 2 bytes
unsigned char  xdata pe_hrst_completed        _at_ MEM_ADDR_PORTX + 16;
unsigned char  xdata explicit_contract        _at_ MEM_ADDR_PORTX + 17;
unsigned char  xdata PortPowerRole            _at_ MEM_ADDR_PORTX + 18;
unsigned char  xdata InitPowerRole            _at_ MEM_ADDR_PORTX + 19;
unsigned char  xdata PortDataRole             _at_ MEM_ADDR_PORTX + 20;
unsigned char  xdata InitDataRole             _at_ MEM_ADDR_PORTX + 21;
unsigned char  xdata cc1_state                _at_ MEM_ADDR_PORTX + 22;
unsigned char  xdata cc2_state                _at_ MEM_ADDR_PORTX + 23;
unsigned char  xdata cc_attached              _at_ MEM_ADDR_PORTX + 24;
unsigned char  xdata monitored_rd_cc          _at_ MEM_ADDR_PORTX + 25;
unsigned char  xdata monitored_ra_cc          _at_ MEM_ADDR_PORTX + 26;
unsigned char  xdata cable_connected          _at_ MEM_ADDR_PORTX + 27;
unsigned char  xdata cable_speed              _at_ MEM_ADDR_PORTX + 28;
unsigned char  xdata cable_current            _at_ MEM_ADDR_PORTX + 29;
unsigned char  xdata cable_ack                _at_ MEM_ADDR_PORTX + 30;
unsigned char  xdata supplied_vconn           _at_ MEM_ADDR_PORTX + 31;
unsigned char  xdata requested_obj_pos        _at_ MEM_ADDR_PORTX + 32;
unsigned char  xdata pwr_role_swapped         _at_ MEM_ADDR_PORTX + 33;
unsigned char  xdata valid_request            _at_ MEM_ADDR_PORTX + 34;   
unsigned char  xdata src_pdo_count            _at_ MEM_ADDR_PORTX + 35;
unsigned char  xdata usb_comm_capable         _at_ MEM_ADDR_PORTX + 36;
unsigned char  xdata external_power           _at_ MEM_ADDR_PORTX + 37;
unsigned char  xdata enable_port              _at_ MEM_ADDR_PORTX + 38;
unsigned char  xdata vbus_tolerance_25mv      _at_ MEM_ADDR_PORTX + 39;

// dpm flag
unsigned char  xdata dpm_hard_reset_request   _at_ MEM_ADDR_PORTX + 40;
unsigned char  xdata dpm_new_power_request     _at_ MEM_ADDR_PORTX + 41;
unsigned char  xdata dpm_get_src_cap_request   _at_ MEM_ADDR_PORTX + 42;
unsigned char  xdata dpm_get_snk_cap_request   _at_ MEM_ADDR_PORTX + 43;
unsigned char  xdata msg_abort                 _at_ MEM_ADDR_PORTX + 44;  
unsigned char  xdata vbus_to_20v               _at_ MEM_ADDR_PORTX + 45;
unsigned char  xdata vbus_from_20v             _at_ MEM_ADDR_PORTX + 46;

//
unsigned short xdata req_max_opr_cur_10ma      _at_ MEM_ADDR_PORTX + 47;  // 2 bytes
unsigned short xdata req_opr_cur_10ma          _at_ MEM_ADDR_PORTX + 49;  // 2 bytes
unsigned short xdata req_opr_vol_50mv          _at_ MEM_ADDR_PORTX + 51;  // 2 bytes

// bist
unsigned char  xdata bist_mode                 _at_ MEM_ADDR_PORTX + 53;

// vdm
unsigned short xdata vdm_svid                  _at_ MEM_ADDR_PORTX + 54;  // 2 bytes
unsigned char  xdata vdm_type                  _at_ MEM_ADDR_PORTX + 56;
unsigned char  xdata vdm_cmd_type              _at_ MEM_ADDR_PORTX + 57;
unsigned char  xdata vdm_command               _at_ MEM_ADDR_PORTX + 58;
unsigned char  xdata vdm_obj_pos               _at_ MEM_ADDR_PORTX + 59;

// timer enable flag
unsigned char xdata EnableSinkWaitCapTimer    _at_ MEM_ADDR_PORTX + 60;
unsigned char xdata EnablePSTransitionTimer   _at_ MEM_ADDR_PORTX + 61;
unsigned char xdata EnableWaitSendTimer       _at_ MEM_ADDR_PORTX + 62;
unsigned char xdata EnableNoResponseTimer     _at_ MEM_ADDR_PORTX + 63;
unsigned char xdata EnableSenderResponseTimer _at_ MEM_ADDR_PORTX + 64;

// counter
unsigned char xdata HardResetCounter          _at_ MEM_ADDR_PORTX + 65;
unsigned char xdata modify_sink_wait_cap      _at_ MEM_ADDR_PORTX + 66;
unsigned char xdata CapsCounter               _at_ MEM_ADDR_PORTX + 67;
unsigned char xdata DiscoverIdentityCounter   _at_ MEM_ADDR_PORTX + 68;
unsigned char xdata reserved069               _at_ MEM_ADDR_PORTX + 69; // reserved 

// Message ID variable
unsigned char xdata rx_stored_flag            _at_ MEM_ADDR_PORTX + 70;
unsigned char xdata MessageIDCounter          _at_ MEM_ADDR_PORTX + 71; // same name with spec
unsigned char xdata StoredMessageID           _at_ MEM_ADDR_PORTX + 72; // same name with spec

unsigned char  xdata cc_result                _at_ MEM_ADDR_PORTX + 73;
unsigned char  xdata cc_looking               _at_ MEM_ADDR_PORTX + 74;
unsigned char  xdata cc_state                 _at_ MEM_ADDR_PORTX + 75;  // bit0~4 of CC_STATUS Register
unsigned char  xdata dual_role_port           _at_ MEM_ADDR_PORTX + 76;
unsigned char  xdata wait_src_recover         _at_ MEM_ADDR_PORTX + 77;

// Message Type, for quickly identify and reduce code size
unsigned char xdata tx_msg_type               _at_ MEM_ADDR_PORTX + 78;
unsigned char xdata rx_msg_type               _at_ MEM_ADDR_PORTX + 79;  

//
unsigned char  xdata partner_external_power   _at_ MEM_ADDR_PORTX + 80;
unsigned char  xdata partner_supp_pr_swap     _at_ MEM_ADDR_PORTX + 81;
unsigned char  xdata partner_supp_dr_swap     _at_ MEM_ADDR_PORTX + 82;
unsigned char  xdata partner_usb_commun       _at_ MEM_ADDR_PORTX + 83;
unsigned char  xdata partner_reject_pr_swap   _at_ MEM_ADDR_PORTX + 84;
unsigned char  xdata partner_reject_dr_swap   _at_ MEM_ADDR_PORTX + 85;
unsigned char  xdata partner_reject_vc_swap   _at_ MEM_ADDR_PORTX + 86;
unsigned char  xdata partner_type             _at_ MEM_ADDR_PORTX + 87;
unsigned char  xdata support_power_swap       _at_ MEM_ADDR_PORTX + 88;
unsigned char  xdata support_data_swap        _at_ MEM_ADDR_PORTX + 89;
unsigned char  xdata reserved90               _at_ MEM_ADDR_PORTX + 90;  
unsigned char  xdata snk_hardresting          _at_ MEM_ADDR_PORTX + 91;
TASKP          xdata ms_state                 _at_ MEM_ADDR_PORTX + 92;  // 2 bytes, ms: message sent
TASKP          xdata md_state                 _at_ MEM_ADDR_PORTX + 94;  // 2 bytes, md: message dropped
TASKP          xdata bk_state                 _at_ MEM_ADDR_PORTX + 96;  // 2 bytes, bk: backup
unsigned short xdata device_id                _at_ MEM_ADDR_PORTX + 98;  // 2 bytes

// Tx Message Payload
struct tx_buf_frame_type xdata tx_frame_type  _at_ MEM_ADDR_PORTX + 100;  // 2  bytes
struct message_header    xdata tx_header      _at_ MEM_ADDR_PORTX + 102;  // 6  bytes
unsigned long            xdata tx_object[7]   _at_ MEM_ADDR_PORTX + 108;  // 28 bytes

// Rx Message Payload
struct rx_buf_frame_type xdata rx_frame_type  _at_ MEM_ADDR_PORTX + 136;  // 1  bytes
struct message_header    xdata rx_header      _at_ MEM_ADDR_PORTX + 137;  // 6  bytes
unsigned long            xdata rx_object[7]   _at_ MEM_ADDR_PORTX + 143;  // 28 bytes

// Request Message, re-send if port partner send Wait
struct tx_buf_frame_type xdata req_frame_type _at_ MEM_ADDR_PORTX + 171;  // 2  bytes
struct message_header    xdata req_header     _at_ MEM_ADDR_PORTX + 173;  // 6  bytes
unsigned long            xdata req_object     _at_ MEM_ADDR_PORTX + 179;  // 4  bytes

// VDM message, backup if interrupted by pd msg, PD2 V1.1 p183
struct tx_buf_frame_type xdata vdm_frame_type _at_ MEM_ADDR_PORTX + 183;  // 2  bytes
struct message_header    xdata vdm_header     _at_ MEM_ADDR_PORTX + 185;  // 6  bytes
unsigned long            xdata vdm_object[7]  _at_ MEM_ADDR_PORTX + 191;  // 28  bytes

// 
unsigned short xdata src_cap_vol_25mv[7]      _at_ MEM_ADDR_PORTX + 219;  // 14 bytes
unsigned short xdata src_cap_cur_10ma[7]      _at_ MEM_ADDR_PORTX + 233;  // 14 bytes

// 
unsigned char  xdata partner_non_responsive   _at_ MEM_ADDR_PORTX + 247;  
unsigned char  xdata reserved248              _at_ MEM_ADDR_PORTX + 248;  
unsigned char  xdata alarm_lo_occurred_0v8    _at_ MEM_ADDR_PORTX + 249;  // 
unsigned char  xdata alarm_lo_occurred_0v1    _at_ MEM_ADDR_PORTX + 250;  // 
unsigned short xdata alarm_lo_threshold_25mv  _at_ MEM_ADDR_PORTX + 251;  // 2 bytes
unsigned short xdata backup_vol_25mv          _at_ MEM_ADDR_PORTX + 253;  // 2 bytes
unsigned short xdata vbus_target_10ma         _at_ MEM_ADDR_PORTX + 255;  // 2 bytes
unsigned char  xdata snk_pdo_count            _at_ MEM_ADDR_PORTX + 257;  // 
unsigned char  xdata reserved258              _at_ MEM_ADDR_PORTX + 258;  // 
unsigned char  xdata CapabilityMismatch       _at_ MEM_ADDR_PORTX + 259;  // 
unsigned char  xdata enable_cc_detach_timer   _at_ MEM_ADDR_PORTX + 260;  // 
unsigned short xdata alarm_hi_threshold_25mv  _at_ MEM_ADDR_PORTX + 261;  // 2 bytes
unsigned short xdata req_opr_vol_25mv         _at_ MEM_ADDR_PORTX + 263;  // 2 bytes
unsigned char  xdata pe_one_busy              _at_ MEM_ADDR_PORTX + 265;  // one port
unsigned char  xdata tx_one_busy              _at_ MEM_ADDR_PORTX + 266;  // one port
unsigned char  xdata enable_discharge_timer   _at_ MEM_ADDR_PORTX + 267;  // 
unsigned short xdata stop_discharge_th_25mv   _at_ MEM_ADDR_PORTX + 268;  // 2 bytes
unsigned char  xdata wait_send_request        _at_ MEM_ADDR_PORTX + 270;  // 
unsigned char  xdata enable_cc_handle         _at_ MEM_ADDR_PORTX + 271;  // 
unsigned char  xdata hard_reset_received      _at_ MEM_ADDR_PORTX + 272;  // 
unsigned char  xdata soft_reset_received      _at_ MEM_ADDR_PORTX + 273;  // 
unsigned char  xdata vol_alarm_hi_occurred    _at_ MEM_ADDR_PORTX + 274;  // 
unsigned char  xdata vol_alarm_lo_occurred    _at_ MEM_ADDR_PORTX + 275;  // 
unsigned char  xdata ov_oc_fault_latched      _at_ MEM_ADDR_PORTX + 276;  // 
unsigned char  xdata src_detached             _at_ MEM_ADDR_PORTX + 277;  // 
unsigned char  xdata reserved278              _at_ MEM_ADDR_PORTX + 278;  // reserved
unsigned char  xdata reserved279              _at_ MEM_ADDR_PORTX + 279;  // reserved
unsigned char  xdata reserved280              _at_ MEM_ADDR_PORTX + 280;  // reserved
unsigned char  xdata reserved281              _at_ MEM_ADDR_PORTX + 281;  // reserved
unsigned char  xdata reserved282              _at_ MEM_ADDR_PORTX + 282;  // reserved
unsigned char  xdata reserved283              _at_ MEM_ADDR_PORTX + 283;  // reserved
unsigned char  xdata reserved284              _at_ MEM_ADDR_PORTX + 284;  // reserved
unsigned char  xdata reserved285              _at_ MEM_ADDR_PORTX + 285;  // reserved
unsigned char  xdata reserved286              _at_ MEM_ADDR_PORTX + 286;  // reserved
unsigned char  xdata reserved287              _at_ MEM_ADDR_PORTX + 287;  // reserved

// common variable
unsigned short xdata vbus_vol;
unsigned short data  supply_pwr_100mw;   // the power suppied to computer port
unsigned short data  adapter_pwr_100mw;  // the power suppied from adapter port
unsigned short xdata adapter_cur_10ma;
unsigned short xdata adapter_vol_25mv;
unsigned short xdata barrel_adapter_25mv;
unsigned short xdata type_c_adapter_25mv;
#if SEND_GET_SNK_CAP_REQUEST_ONCE_OPT
unsigned char xdata dpm_get_snk_cap_request_sent;
#endif

// local use
static unsigned char  xdata local8;
static unsigned short xdata local16;

// alert status
unsigned char   data alert0;      // DATA memory for code size
unsigned char   data alert1;      // DATA memory for code size	
unsigned char  xdata cc_status;   // bit0~7 of CC_STATUS Register
unsigned char  xdata pwr_status;
unsigned char  xdata fault_status;
unsigned char  xdata pwr_ctrl;
unsigned char  xdata fault_ctrl;

// task
unsigned char data port; // must in DATA memory
TASKP  data pe_state;    // must in DATA memory
TASKP  data tx_state;    // must in DATA memory
TASKP  data ac_state;    // must in DATA memory, ac: analog current
TASKP  data hd_state;    // must in DATA memory, hd: hard disk drive
// backup
TASKP xdata bak_pe_state[PORT_COUNT];    
TASKP xdata bak_tx_state[PORT_COUNT];    


// for sata power @PD or non-PD issue
unsigned char  xdata i2c_nak;
unsigned char  xdata i2c_timeout;

// info from DC Jack
unsigned char  xdata ac_inserted;
unsigned char  xdata ac_attached;

// info from adapter partner side
unsigned char  xdata adapter_contract;

// info from computer partner side
unsigned char  xdata computer_contract;
unsigned short xdata sink_from_computer_25mv;
unsigned char  xdata is_data_role_ufp;
unsigned char  xdata vbus_detected;

// variables for hard disk drive state
unsigned char  xdata tcpm_reject_turn_on_disk;
unsigned char  xdata usb_req_turn_on_disk;
        
// (local)                 
unsigned short xdata max_cur_10ma;
unsigned short xdata     cur_10ma;
unsigned short xdata max_vol_50mv;
unsigned short xdata     vol_50mv;
unsigned short xdata max_pwr_100mw;
unsigned short xdata pdo0_10ma;

// (local)    
unsigned char  xdata src_supply_type[7];

// busy flag
unsigned char data combined_busy; // combine all busy flag
unsigned char data pe_all_busy;   // all port flag combines one port flag
unsigned char data tx_all_busy;   // all port flag combines one port flag
unsigned char data ac_busy;
unsigned char data hd_busy;

#ifdef UART
#define DBG_MAX_BUF_INDEX  895
unsigned short xdata dbg_buf_index;
unsigned short xdata dbg_print_index;
unsigned char  xdata dbg_buf[DBG_MAX_BUF_INDEX+1];
unsigned char  xdata dbg_busy;
#endif

//--------------------------------------------------------------------------

void write_mcu_dwcnt_1ms(unsigned char value)
{
    if(port)
    {
        aMCU_DWCNT14_1MS = value;
    }
    else
    {
        aMCU_DWCNT2_1MS = value;
    }
}

unsigned char read_mcu_dwcnt_1ms(void)
{
    if(port)
    {
        return aMCU_DWCNT14_1MS;
    }
    else
    {
        return aMCU_DWCNT2_1MS;
    }
}

void write_mcu_dwcnt_1ms_detach(unsigned char value)
{
    if(port)
    {
        aMCU_DWCNT12_1MS = value;
    }
    else
    {
        aMCU_DWCNT11_1MS = value;
    }
}

unsigned char read_mcu_dwcnt_1ms_detach(void)
{
    if(port)
    {
        return aMCU_DWCNT12_1MS;
    }
    else
    {
        return aMCU_DWCNT11_1MS;
    }
}


void write_mcu_dwcnt_10ms(unsigned char value)
{
    if(port)
    {
        aMCU_DWCNT18_10MS = value;
    }
    else
    {
        aMCU_DWCNT17_10MS = value;
    }
}

unsigned char read_mcu_dwcnt_10ms(void)
{
    if(port)
    {
        return aMCU_DWCNT18_10MS;
    }
    else
    {
        return aMCU_DWCNT17_10MS;
    }
}


void write_mcu_dwcnt_100ms(unsigned char value)
{
    if(port)
    {
        aMCU_DWCNT21_100MS = value;
    }
    else
    {
        aMCU_DWCNT20_100MS = value;
    }
}

unsigned char read_mcu_dwcnt_100ms(void)
{
    if(port)
    {
        return aMCU_DWCNT21_100MS;
    }
    else
    {
        return aMCU_DWCNT20_100MS;
    }
}

void write_mcu_dwcnt_100ms_ac(unsigned char value)
{
    aMCU_DWCNT22_100MS = value;
}

unsigned char read_mcu_dwcnt_100ms_ac(void)
{
    return aMCU_DWCNT22_100MS;
}

void write_mcu_dwcnt_100ms_hd(unsigned char value)
{
    aMCU_DWCNT23_100MS = value;
}

unsigned char read_mcu_dwcnt_100ms_hd(void)
{
    return aMCU_DWCNT23_100MS;
}

//--------------------------------------------------------------------------

void clear_xdata16(void xdata *ptr, unsigned short size)
{
    if(size <= 256)
    {    
        xdata_clear(ptr, size);
    }
    else
    {
        xdata_clear(ptr,     256);
        xdata_clear(ptr+256, size-256);
    }
}

void copy_xdata16(const void xdata *src, void xdata *dest, unsigned short size)
{
    if(size <= 256)
    {    
        xdata_copy(src, dest, size);
    }
    else
    {
        xdata_copy(src,     dest,     256);
        xdata_copy(src+256, dest+256, size-256);
    }
}
        
void restore_port_variables(void)
{
	unsigned char iii=3;
    if(port)
    {
        copy_xdata16(MEM_ADDR_PORT1, MEM_ADDR_PORTX, MEM_ADDR_SIZE);
    }
    else
    {
        copy_xdata16(MEM_ADDR_PORT0, MEM_ADDR_PORTX, MEM_ADDR_SIZE);
    }           
    
    // task, pe_state and tx_state must in DATA memory
    pe_state = bak_pe_state[port];
    tx_state = bak_tx_state[port];        
}

void backup_port_variables(void)
{
    if(port)
    {
        copy_xdata16(MEM_ADDR_PORTX, MEM_ADDR_PORT1, MEM_ADDR_SIZE);
    }
    else
    {
        copy_xdata16(MEM_ADDR_PORTX, MEM_ADDR_PORT0, MEM_ADDR_SIZE);
    }
        
    // task, pe_state and tx_state must in DATA memory
    bak_pe_state[port] = pe_state;
    bak_tx_state[port] = tx_state;         
}


//--------------------------------------------------------------------------

void dpm_build_sop_ctrl_msg(unsigned char msg)
{
    tx_frame_type.SOP = TX_SOP_0P;
    set_tx_ctrl_header(msg);
}

void dpm_build_req_msg(void)
{
    // decide Request Message's SOP
    tx_frame_type.SOP = TX_SOP_0P;
    
    // decide Request Message's Header
    set_tx_data_header(MSG_REQUEST, 1);
    
    //
    shift_tab = req_pdo_shift;
    list_count = sizeof(req_pdo_shift);
    dpm_combine_cmm_list_to_tx_obj(0);
                
    // backup, re-send if port partner send Wait
    xdata_copy(&tx_frame_type, &req_frame_type, sizeof(req_frame_type));
    xdata_copy(&tx_header,     &req_header,     sizeof(req_header));
    xdata_copy(&tx_object[0],  &req_object,     sizeof(req_object));  // Request Message only have 1 data object
}

void dpm_build_snk_cap_msg(void)
{
    // decide SOP
    tx_frame_type.SOP = TX_SOP_0P;

#if 0 // code size = 40 bytes
    // decide Power Data Object
    cmm_pdo_list[0] = req_opr_cur_10ma;  // OperationalCurrent10mA
    cmm_pdo_list[1] = req_opr_vol_50mv;  // Voltage50mV
    cmm_pdo_list[2] = 0;            // Reserved – shall be set to zero.
    cmm_pdo_list[3] = 0;            // DataRoleSwap
    cmm_pdo_list[4] = usb_comm_capable;  // USBCommunicationsCapable
    cmm_pdo_list[5] = external_power;    // ExternallyPowered
    cmm_pdo_list[6] = 0;            // HigherCapability
    cmm_pdo_list[7] = 0;            // DualRolePower
    cmm_pdo_list[8] = FIXED_SUPPLY; // SupplyType
        
    // build 32-bit data object, for reduce code size    
    shift_tab = snk_fix_pdo_shift;
    list_count = sizeof(snk_fix_pdo_shift);
    dpm_combine_cmm_list_to_tx_obj(0);

#else
    
    // code size = 11 bytes
    // 1.5A for 2.5" HDD
    if(PARTNER_COMPUTER == partner_type)
    {
        if(dual_role_port)
        {
            // requiring no power from the Source, the Voltage (B19..10) shall be set to 5V 
            //   and the Operational Current shall be set to 0mA. PD2 V1.2 ECN p161
            //   Actually our PCBA still draw small current, so we choise 0.5ma
            snk_pdo_count = 1;
            tx_object[0] = SNK_CAP_PDO_5V0_0A5_COMPUTER_DRP;
        }
        else
        {
            #if (HDD_5V_ONLY == DRIVE_HDD_PWR_TYPE)
            {
                snk_pdo_count = 1;
                tx_object[0] = SNK_CAP_PDO_5V0_0A9_COMPUTER_SNK;                  
            }
            #elif (HDD_5V_12V == DRIVE_HDD_PWR_TYPE)
            {
                snk_pdo_count = 3;
                tx_object[0] = SNK_CAP_PDO_5V0_3A0_COMPUTER_SNK;
                tx_object[1] = SNK_CAP_PDO_12V_3A0_COMPUTER_SNK;
                tx_object[2] = SNK_CAP_VAR_PDO_15V_20V_3A0_COMPUTER_SNK;                   
            }
            #endif 
        }
    }
    else if(PARTNER_ADAPTER == partner_type)
    {
        snk_pdo_count = 2;
        tx_object[0] = SNK_CAP_PDO_5V0_3A0_ADAPTER_SNK; 
        tx_object[1] = SNK_CAP_PDO_14V8_20V0_3A0_ADAPTER_SNK; 
    }

    // decide Header
    set_tx_data_header(MSG_SINK_CAPABILITIES, snk_pdo_count);    
#endif
    
    
    
//ds(""); dw(tx_object[0] >> 16);  dw(tx_object[0]);                    
}

void dpm_parser_snk_pdo(void)
{
    // 
    // Since all USB Consumers support vSafe5V, the required vSafe5V Fixed Supply Power Data Object is also used 
    // to convey additional information that is returned in bits 29 through 20. PD2 V1.1 p158
    //
    
    // shift table
    shift_tab   = snk_fix_pdo_shift;
    list_count  = sizeof(snk_fix_pdo_shift);
    mask_tab    = snk_fix_pdo_mask;
        
    // extract common list from data object
    dpm_extract_cmm_list_from_rx_obj(0);  
    
    // 
    partner_supp_dr_swap   = cmm_pdo_list[3];
    partner_usb_commun     = cmm_pdo_list[4];
    partner_external_power = cmm_pdo_list[5];
    partner_supp_pr_swap   = cmm_pdo_list[7];
    
//U ds("dr"); db(partner_supp_dr_swap); 
//U ds("pr"); db(partner_supp_pr_swap); 
//U ds("ex"); db(partner_external_power);
//U ds("cm"); db(partner_usb_commun);      
}

void dpm_parser_src_pdo(unsigned char index)
{
    if(index >= 7)
    {
U ds("!e");        
        return;
    }
        
    // extract bit30~31: SupplyType
    src_supply_type[index] = (rx_object[index] >> 30) & 0x03;
    
    
    // decide shift table
    if(FIXED_SUPPLY == src_supply_type[index])  // 
    {
        shift_tab   = src_fix_pdo_shift;
        list_count  = sizeof(src_fix_pdo_shift);
        mask_tab    = src_fix_pdo_mask;
    }    
    else // variable or battery
    {
        shift_tab   = var_bat_pdo_shift;
        list_count  = sizeof(var_bat_pdo_shift);
        mask_tab    = var_bat_pdo_mask;
    }
   
    // extract common list from data object
    dpm_extract_cmm_list_from_rx_obj(index);

//ds(""); dw(rx_object[0] >> 16); dw(rx_object[0]); 
   
    // code size: 318 bytes
    // assign each item
//    if(FIXED_SUPPLY == src_supply_type[index])  
//    {
//        src_fix_pdo[index].MaximumCurrent10mA       = cmm_pdo_list[0];
//        src_fix_pdo[index].Voltage50mV              = cmm_pdo_list[1];
//        src_fix_pdo[index].PeakCurrent              = cmm_pdo_list[2];
//        src_fix_pdo[index].DataRoleSwap             = cmm_pdo_list[3];
//        src_fix_pdo[index].USBCommunicationsCapable = cmm_pdo_list[4];
//        src_fix_pdo[index].ExternallyPowered        = cmm_pdo_list[5];
//        src_fix_pdo[index].USBSuspendSupported      = cmm_pdo_list[6];
//        src_fix_pdo[index].DualRolePower            = cmm_pdo_list[7];
//    } 
//    else if(BATTERY_SUPPLY == src_supply_type[index])  
//    {
//        src_bat_pdo[index].MaximumAllowablePower250mW = cmm_pdo_list[0];
//        src_bat_pdo[index].MinimumVoltage50mV         = cmm_pdo_list[1];
//        src_bat_pdo[index].MaximumVoltage50mV         = cmm_pdo_list[2];
//    }     
//    else if(VARIABLE_SUPPLY == src_supply_type[index])  
//    {
//        src_var_pdo[index].MaximumCurrent10mA = cmm_pdo_list[0];
//        src_var_pdo[index].MinimumVoltage50mV = cmm_pdo_list[1];
//        src_var_pdo[index].MaximumVoltage50mV = cmm_pdo_list[2];
//    }    
}

#if BIST_SUPPORT
void dpm_paser_bist_header(void)
{
    shift_tab   = bist_header_shift;
    list_count  = sizeof(bist_header_shift);
    mask_tab    = bist_header_mask;
   
    // extract common list from data object
    dpm_extract_cmm_list_from_rx_obj(0);

    bist_mode = cmm_pdo_list[2];
//ds(""); dw(rx_object[0] >> 16); dw(rx_object[0]); 
   
}   

void dpm_build_transmit_bist_carrier_mode2(void)
{
    clr_tx_payload();
    
    // decide SOP
    tx_frame_type.SOP = TX_BIST_MODE_2;
}
#endif
    
void dpm_evaluate_src_cap(void)
{
    unsigned char i;
                
    // parser 5V Fixed Power Data Object
    // The vSafe5V Fixed Supply Object shall always be the first object, PD2 V1.1 p152
    dpm_parser_src_pdo(0);
    
    // vSafe5V Fixed Supply Power Data Object is also used to convey additional information that is returned in bits 29 through 25. 
    //   other Fixed Supply Power Data Objects shall set bits 29...22 to zero, PD2 V1.1 p154
    partner_supp_dr_swap   = cmm_pdo_list[4];
    partner_usb_commun     = cmm_pdo_list[5];
    partner_external_power = cmm_pdo_list[6];
    partner_supp_pr_swap   = cmm_pdo_list[8];
    
//U ds("dr"); db(partner_supp_dr_swap); 
//U ds("pr"); db(partner_supp_pr_swap); 
//U ds("ext"); db(partner_external_power);
//U ds("com"); db(partner_usb_commun);
    
    // default request 5v 
    max_cur_10ma = cmm_pdo_list[0];
    max_vol_50mv = cmm_pdo_list[1]; 
    pdo0_10ma    = max_cur_10ma;   
    requested_obj_pos = 0; 

               
    // request max power if adapter port
    if(PARTNER_ADAPTER == partner_type)
    {
        // parser others power data object
        for(i=1; i<rx_header.NumOfDataObj; i++)
        {
            // parser Power Data Object, from low to high
            dpm_parser_src_pdo(i);
    
            // decide current of request pdo 
            if(FIXED_SUPPLY == src_supply_type[i])
            {
                cur_10ma = cmm_pdo_list[0];  // Maximum Current in 10mA units
                vol_50mv = cmm_pdo_list[1];  // Voltage in 50mV units    
            }
            else if(VARIABLE_SUPPLY == src_supply_type[i])
            {
                cur_10ma = cmm_pdo_list[0];  // Maximum Current in 10mA units
                vol_50mv = cmm_pdo_list[2];  // Maximum Voltage in 50mV units
            }
            else  // battery
            {
                vol_50mv = cmm_pdo_list[2];  // Maximum Voltage in 50mV units
                
                // calculate max current, I = P / V, 500 = 100mA * 250mW / 50mV
                cur_10ma = 0;
                if(vol_50mv)
                {
                    cur_10ma = 500 * cmm_pdo_list[0] / vol_50mv;  // [0]: Maximum Allowable Power in 250mW units
                }
            }       
            
            // select max voltage
            if(vol_50mv > max_vol_50mv)
            {
                max_vol_50mv = vol_50mv;
                max_cur_10ma = cur_10ma;
                requested_obj_pos = i;
            }
        }//for
        
        CapabilityMismatch   = 0;
        // revise max_vol_50mv
        if(0 == max_cur_10ma)
        {
            // if partner supplied 0mA, just request 5V PDO
            max_vol_50mv = VOL05V_50MV;
            requested_obj_pos = 0;
        }
        else if(TYPEC_ACCEPTABLE_V_25MV > max_vol_50mv || TYPEC_ACCEPTABLE_PWR_100MW > calc_power_100mw(max_vol_50mv*2, max_cur_10ma) )
        {
            // only select over 14.8V/29W or 5V PDO 
            max_vol_50mv = VOL05V_50MV;
            requested_obj_pos = 0;
            CapabilityMismatch   = 1;          
        }
        
        // request maximum voltage
        req_opr_vol_50mv = max_vol_50mv;  
        req_max_opr_cur_10ma = max_cur_10ma;
        req_opr_cur_10ma     = max_cur_10ma;
    }
    else if(PARTNER_COMPUTER == partner_type)
    {
        // decide:
        //   1. req_max_opr_cur_10ma
        //   2. req_opr_cur_10ma
        //   3. CapabilityMismatch
        //   4. req_opr_vol_50mv
        //   5. requested_obj_pos
        if(external_power) 
        {
            // request 5v and don't care current
            requested_obj_pos    = 0;   
            req_opr_vol_50mv     = VOL05V_50MV;    
            req_max_opr_cur_10ma = pdo0_10ma;
            req_opr_cur_10ma     = pdo0_10ma;
            CapabilityMismatch   = 0;            
        }
        else
        {   
            #if (HDD_5V_ONLY == DRIVE_HDD_PWR_TYPE)
            {                
                // only request 5v
                req_opr_vol_50mv  = VOL05V_50MV;    
                requested_obj_pos = 0;
                                
                if(pdo0_10ma < CUR0A9_10MA)  // 1.5A for 2.5" HDD
                {
                    // Maximum Operating Current/Power field may contain a value "larger than" the maximum current/power offered in the Source_Capabilities
                    req_max_opr_cur_10ma = CUR0A9_10MA;
                    req_opr_cur_10ma     = pdo0_10ma;
                    CapabilityMismatch   = 1;

                    // we have to draw 900ms at least
                    // tcpm_reject_turn_on_disk = 1; 
                }
                else
                {
                    // Maximum Operating Current/Power field shall contain a value "less than or equal to" the maximum current/power offered in the Source_Capabilities
                    req_max_opr_cur_10ma = CUR0A9_10MA;
                    req_opr_cur_10ma     = CUR0A9_10MA;
                    CapabilityMismatch   = 0;
                }
            }
            #elif (HDD_5V_12V == DRIVE_HDD_PWR_TYPE)
            {
                // find max voltage
                for(i=1; i<rx_header.NumOfDataObj; i++)
                {
                    // parser Power Data Object, from low to high
                    dpm_parser_src_pdo(i);
            
                    // decide current of request pdo
                    if(FIXED_SUPPLY == src_supply_type[i])
                    {
                        //cur_10ma = cmm_pdo_list[0];  // Maximum Current in 10mA units
                        //vol_50mv = cmm_pdo_list[1];  // Voltage in 50mV units
                        // select max voltage
                        if(cmm_pdo_list[1] > max_vol_50mv)
                        {
                            max_cur_10ma  = cmm_pdo_list[0];
                            max_vol_50mv  = cmm_pdo_list[1];
                            max_pwr_100mw = calc_power_100mw(max_vol_50mv*2, max_cur_10ma);
                            requested_obj_pos = i;
                        } 
                    }
                }//for
                
                // if supplied voltage < 12v, just request 5v because 3.5" HDD cannot be drived
                if(max_vol_50mv < VOL12V_50MV)
                {
                    requested_obj_pos    = 0;
                    req_opr_vol_50mv     = VOL05V_50MV;
                    req_max_opr_cur_10ma = pdo0_10ma;
                    req_opr_cur_10ma     = pdo0_10ma;
                    CapabilityMismatch   = 0;
                }
                else if(max_pwr_100mw < BUS_PWR_5V_12V_OPR_100MW)
                {
                    req_opr_vol_50mv     = max_vol_50mv;
                    req_max_opr_cur_10ma = calc_current_10ma(BUS_PWR_5V_12V_OPR_100MW, max_vol_50mv*2);
                    req_opr_cur_10ma     = max_cur_10ma;
                    CapabilityMismatch   = 1;
                }
                else    
                {
                    req_opr_vol_50mv     = max_vol_50mv;
                    req_max_opr_cur_10ma = calc_current_10ma(max_pwr_100mw, max_vol_50mv*2);
                    req_opr_cur_10ma     = req_max_opr_cur_10ma;
                    CapabilityMismatch   = 0;  
                }
            }//(DRIVE_HDD_PWR_TYPE)
            #endif   
        }//(external_power)
    }//(partner_type)

    // one base
    requested_obj_pos++;
    
    // update voltage with unit 25mv
    req_opr_vol_25mv = req_opr_vol_50mv * 2;
    
//U ds("v"); dw(max_vol_50mv*2);  // *2: to 25mv
//U ds("c"); dw(max_cur_10ma); 
     
    

//U ds("p"); db(requested_obj_pos); 
//U ds("c"); dw(req_opr_cur_10ma);
U ds("v"); dw(req_opr_vol_25mv);  


    // decide Request's Power Data Object 
    cmm_pdo_list[0] = req_max_opr_cur_10ma;      // MaximumOperatingCurrent_10mA
    cmm_pdo_list[1] = req_opr_cur_10ma;          // OperatingCurrent_10mA
    cmm_pdo_list[2] = 0;                    // Reserved
    cmm_pdo_list[3] = 1;                    // NoUSBSuspend
    cmm_pdo_list[4] = usb_comm_capable;     // USBCommunicationsCapable
    cmm_pdo_list[5] = CapabilityMismatch;   // CapabilityMismatch
    cmm_pdo_list[6] = 0;                    // GiveBackFlag
    cmm_pdo_list[7] = requested_obj_pos;  // ObjectPosition
    cmm_pdo_list[8] = 0;                    // Reserved
}

void dpm_build_src_cap_obj_index(unsigned char index)
{
    // calculate current
    local16 = calc_current_10ma(supply_pwr_100mw, src_cap_vol_25mv[index]);
    
    // resolution is 500ma(50*10ma) because OC resolution is 0.5A
    // Ex : 9V , 2.78A set supply pwr = 2.5A
    // Ex : 15V，1.67A set supply pwr = 1.8A
//    local16 /= 50;
//    local16 *= 50;
    
    // limit current by cable capability 
    if ((local16 > 300) && (CABLE_CUR_3A == cable_current))
    {
        local16 = 300;
    }

    // protect
    if (index >= 7)
    {
U       ds("!e");
        return;        
    }
        
    //
    src_cap_cur_10ma[index] = local16;
    
    // decide Power Data Object
    cmm_pdo_list[0] = src_cap_cur_10ma[index];   // Maximum Current in 10mA units
    cmm_pdo_list[1] = src_cap_vol_25mv[index] >> 1;    // Voltage in 50mV units
    cmm_pdo_list[2] = 0;                // Peak Current
    cmm_pdo_list[3] = 0;                // Reserved – shall be set to zero.
    cmm_pdo_list[4] = 1;                // Data Role Swap
    cmm_pdo_list[5] = usb_comm_capable; // USBCommunicationsCapable
    cmm_pdo_list[6] = external_power;   // ExternallyPowered
    cmm_pdo_list[7] = 0;                // USB Suspend Supported
    cmm_pdo_list[8] = 1;                // Dual-Role Power
    cmm_pdo_list[9] = FIXED_SUPPLY;     // SupplyType    
    
    // except vSafe5V PDO, other Fixed Supply Power Data Objects shall set bits 29...22 to zero. PD2 V1.1 p154
    if (0 != index)
    {
        cmm_pdo_list[4] = 0;                // Data Role Swap
        cmm_pdo_list[5] = 0;                // USBCommunicationsCapable
        cmm_pdo_list[6] = 0;                // ExternallyPowered
        cmm_pdo_list[7] = 0;                // USB Suspend Supported
        cmm_pdo_list[8] = 0;                // Dual-Role Power        
    }
    
        
    // build 32-bit data object, for reduce code size    
    shift_tab = src_fix_pdo_shift;
    list_count = sizeof(src_fix_pdo_shift);
    dpm_combine_cmm_list_to_tx_obj(index); 
    
//ds(""); dw(tx_object[0] >> 16);  dw(tx_object[0]);               
}

void dpm_build_all_vol_pdo(void)
{
    unsigned char i;

    // 
    // source must follow power rule, PD2 V1.2 p490
    //

    //
    calc_supply_pwr_100mw();
    
U   ds("pw"); dw(supply_pwr_100mw);
    
   
    // voltage, unit: 25mv
    src_cap_vol_25mv[0] = SAFE5V_25MV;
    src_cap_vol_25mv[1] = VOL9V0_25MV;
    src_cap_vol_25mv[2] = VOL12V_25MV;
    src_cap_vol_25mv[3] = VOL15V_25MV;
    src_cap_vol_25mv[4] = VOL20V_25MV;
    
    // count of PDO
    src_pdo_count = 5;
    
    // calculate src_cap_cur_10ma[x] and tx_object[x]
    for(i=0; i<src_pdo_count; i++) 
    {
        dpm_build_src_cap_obj_index(i); 
    }
}

void dpm_build_src_cap_msg(void)
{
    //
    // source must follow power rule, PD2 V1.2 p490
    // 
   
    // decide SOP
    tx_frame_type.SOP = TX_SOP_0P;
        
    // Type-C standard cable's Current Rating is 3A if without Electronically Marked, TypeC1.1 p24
    if (!cable_ack)
    {
        cable_current = CABLE_CUR_3A;
    }
    
    dpm_build_all_vol_pdo();
    
    // decide Header
    set_tx_data_header(MSG_SOURCE_CAPABILITIES, src_pdo_count);      
}

bit dpm_parser_req_pdo(void)
{
//    ds(""); dw(rx_object[0] >> 16); dw(rx_object[0]);    

    //
    shift_tab   = req_pdo_shift;
    list_count  = sizeof(req_pdo_shift);
    mask_tab    = req_pdo_mask;

    // extract common list from data object
    dpm_extract_cmm_list_from_rx_obj(0);
    
    // check ObjectPosition
    if ((!cmm_pdo_list[7]) || (cmm_pdo_list[7] > src_pdo_count))  
    {
        return 0;
    }
    else
    {
        requested_obj_pos = cmm_pdo_list[7];

U       ds("p"); db(requested_obj_pos);    
    }

#if 0       
    // over max current loading
    if(cmm_pdo_list[0] > src_cap_cur_10ma[requested_obj_pos-1])  // MaximumOperatingCurrent_10mA
    {
        return 0;
    }
#endif
    
    // over current loading
    if(cmm_pdo_list[1] > src_cap_cur_10ma[requested_obj_pos-1])  // OperatingCurrent_10mA
    {
        return 0;
    }

    // match
    return 1;
}

#if IDENTIFY_CABLE_SUPPORT
void dpm_build_cable_identity_msg(void)
{
    // decide SOP' for cable
    tx_frame_type.SOP = TX_SOP_1P;
    
    // decide Header, VDM Header
    set_tx_cable_data_header(MSG_VENDOR_DEFINED, 1);
    
    // VDM Header
    tx_object[0] = DISCOVER_IDENTITY_INITIATOR_HEADER;
    
//ds("obj="); dw(tx_object[0] >> 16);  dw(tx_object[0]);    
}

void dpm_parser_cable_vdm(void)
{
    // setup
    shift_tab   = vdm_header_shift;
    list_count  = sizeof(vdm_header_shift);
    mask_tab    = vdm_header_mask;    
    
    // extract common list from data object
    dpm_extract_cmm_list_from_rx_obj(0);   
    
//U ds("p:cmd"); db(cmm_pdo_list[0]);
//U ds("p:type"); db(cmm_pdo_list[2]);

    if((VDM_DISCOVER_IDENTITY == cmm_pdo_list[0]) &&
       (VDM_ACK               == cmm_pdo_list[2]))  
    {    
        
        // rising flag
        cable_ack = 1;
        
        // PD2 V1.1 p174
        cable_speed   =  rx_object[4] & 0x07;
        cable_current = (rx_object[4] >> 5) & 0x03;
       

U ds("p:s"); db(cable_speed);  
U ds("p:c"); db(cable_current);     
                
#if 0  // 80 bytes
            // extract VDM Header
            shift_tab   = id_header_shift;
            list_count  = sizeof(id_header_shift); 
            mask_tab    = id_header_mask;
            dpm_extract_cmm_list_from_rx_obj(1); 
            
//U ds("p: VID="); dw(cmm_pdo_list[0]);     
//U ds("p: product="); db(cmm_pdo_list[3]);  

            // extract cable VDO
            shift_tab   = vdm_cable_shift;
            list_count  = sizeof(vdm_cable_shift); 
            mask_tab    = vdm_cable_mask;
            dpm_extract_cmm_list_from_rx_obj(4);

//U ds("p: speed="); db(cmm_pdo_list[0]);     
//U ds("p: current="); db(cmm_pdo_list[3]);     
//U ds("p: termin="); db(cmm_pdo_list[8]);       
#endif                                     
    }      
}
#endif

void tx_wait_for_message_request(void)
{
    // do nothing
    
    // task idle, one port
    tx_one_busy = 0;
}

#if VDM_SUPPORT
void tx_construct_vdm_message(void)
{   
    // task busy, one port
    tx_one_busy = 1;
    
    // clear 
	msg_trans_OK = 0;
	msg_trans_NG = 0;
	msg_Disc = 0;
	msg_abort = 0;
        
    if(pd_msg_received() || check_msg_receiving())
    {
//U ds("t:v_r-ed_r-ing,bak");

        // pending if interrupted by pd msg, PD2 V1.1 p183
        vdm_pending = 1;
        
        backup_vdm_msg();
        
        tx_state = tx_wait_for_message_request;        
    }
    else
    {
U ds("t:VDM");

        // send message
        vdm_pending = 0;
        
        prl_send_msg();
        
        tx_state = tx_wait_for_phy_response;
    }
}
#endif

void tx_construct_message(void)
{   
    // task busy, one port
    tx_one_busy = 1;
    
    // clear 
	msg_trans_OK = 0;
	msg_trans_NG = 0;
	msg_Disc = 0;
	msg_abort = 0;
    
    if(sop_msg_received() || check_msg_receiving())
    {
//U ds("t:r-ed_r-ing");
        
        // send message is ignored
        msg_abort = 1;
        _msg_state = _Protocol_Lay_Standby;
    }
    else
    {
U ds("t:"); 
U ds(msg_id2string[tx_msg_type]);
        
        prl_send_msg();
        
        _msg_state = _Protocol_Lay_Standby;     
    }
}

void tx_wait_for_phy_response(void)
{
    if(msg_trans_OK)
    {                       
//U ds("t:sent");
        
#if VDM_SUPPORT
        if(vdm_pending)
        {
U ds("t:v_pend");

            // re-send, PD2 V1.1 p183
            restore_vdm_msg();
            
            tx_state = tx_construct_vdm_message;
        }
        else
#endif        
        {
            tx_state = tx_wait_for_message_request;
        }
    }
    else if(msg_trans_NG)  // wait GoodCRC timeout
    {
//U ds("tx_fail"); 
                   
        if(pd_connected)
        {
            if(MSG_SOFT_RESET == tx_msg_type)  // soft_reset message timeout
            {                
//U ds("t:hrst");
    
                if(PWR_ROLE_SNK == PortPowerRole)
                {
//                    pe_state = pe_snk_hard_reset;
                }
                else
                {
//                    pe_state = pe_src_hard_reset;
                }   
            }
            else  // non-soft_reset message timeout
            {
//U ds("t:srst"); db(tx_msg_type);
    
//                pe_state = pe_xxx_send_soft_reset;            
            }
        }
        else
        {
            // task idle, port partner is not responsive to PD msg
            tx_one_busy = 0;            
        }    
    }
    else if(msg_Disc)
    {
//U ds("t:discard");
        
#if VDM_SUPPORT
        if(MSG_VENDOR_DEFINED == tx_msg_type)
        {
            if(pd_msg_received())
            {
                // pending if interrupted by pd msg, PD2 V1.1 p183            
                // rising flag
                vdm_pending = 1;
                
                backup_vdm_msg();                                           
            }            
        }
#endif

        //should not happen, need to retransmit or ignore? 
        tx_state = tx_wait_for_message_request;          
    }    
}

void tx_hr_request_hard_reset(void)
{
//U ds("t:->HR");
   	tcpm_soft_reset();
    // clear
	msg_trans_OK = 0;
	msg_trans_NG = 0;
	msg_Disc = 0;
	msg_abort = 0;
	pd_connected = 0;
    
    // send hard reset, TCPC1.0 p82
    prl_send_hard_reset();
    
    _msg_state = tx_hr_wait_for_phy_hard_reset_complete;
}

void tx_hr_wait_for_phy_hard_reset_complete(void)
{
    if(msg_trans_OK)
    {
//U ds("t:hr_sent");

        // Inform Policy Engine Hard Reset has been sent 
        hrst_sent = 1;
        
        _msg_state = tx_hr_disable_tcpc_receiver;
    }
    else if(msg_Disc)
    {
        // send again
        _msg_state = tx_hr_request_hard_reset;
    }      
}

void tx_hr_disable_tcpc_receiver(void)
{
//U ds("t:disable_det");

    // clear
    pe_hrst_completed = 0;
        
    // RECEIVE_DETECT=0x00 (disable TCPC receiver)
    disable_tcpc_receiver();
        
    tx_state = tx_hr_wait_for_pe_hard_reset_complete;    
}

void tx_hr_wait_for_pe_hard_reset_complete(void)
{
//U ds("t:wait_completed");

    if(pe_hrst_completed)
    {
//U ds("t:hr_comp");
        
        // setting RECEIVE_DETECT to non-zero value (enable TCPC receiver)
        enable_receive_detect_message_header();
        
        tx_state = tx_wait_for_message_request;        
    }
}

//--------------------------------------------------------------------------

void ac_task(void)
{
    ac_state();
}

void ac_unattached(void)
{
U ds("a:Un");
    
    ac_state = ac_wait_adapter_coming;
}

void ac_wait_adapter_coming(void)
{
    if(ac_inserted)
    {
//U ds("a:Wa");         
            
        //
        write_ac_debounce_timer_100ms(1);
        
        ac_state = ac_wait_debounce_timeout;
    }
    else
    {    
        // task idle, no ac barrel adapter 
        ac_busy = 0;
    }                
}

void ac_wait_debounce_timeout(void)
{
    if( ! ac_inserted)
    {
//U ds("a:bk"); 

        ac_state = ac_wait_adapter_coming;
    }
    else if( ! read_ac_debounce_timer_100ms())
    {
U ds("a:ed");

        //
        if(usb_req_turn_on_disk)
        {
            #if (HDD_5V_ONLY == DRIVE_HDD_PWR_TYPE)
            {
                // partner supply 5V to us, so we have turned on hdd 5v bypass nmos.
                // even if externel power is attached, the setting cannot be changed.
                // if swap to source, must call rt1711_switch_to_hdd_buck_path()(cannot keep bypass nmos)
            }
            #elif (HDD_5V_12V == DRIVE_HDD_PWR_TYPE)
            {
                // partner supply 12V to us, so we have turned on hdd 12v bypass nmos.
                // even if externel power is attached, the setting cannot be changed.
                // if swap to source, must call rt1711_switch_to_hdd_buck_path()(cannot keep bypass nmos) 
            }                
            #endif
        }
        
        ac_attached = 1;
        
        // limit 20v
        barrel_adapter_25mv = VOL20V_25MV;
        
        
        ac_state = ac_ready;
    }
}

void ac_ready(void)
{
    if( ! ac_inserted)
    {
        // detecting detached timer
        write_ac_detached_timer_100ms(1);
        
        // check bounce or truly detached
        ac_state = ac_wait_detached;
    }
    else
    {
        // task idle, ac barrel adapter is attached 
        ac_busy = 0;
    }
}

void ac_wait_detached(void)
{
    if( ! read_ac_detached_timer_100ms())
    {
        // power adapter has been removed, i.e. detached
        ac_state = ac_detached;           
    }
    else if(ac_inserted)
    {
        // just a bounce, so back to ready state
        ac_state = ac_ready;
    }
}

void ac_detached(void)
{
    // clear, encount detach 
    ac_attached = 0;
    barrel_adapter_25mv = 0;
            
    // turn off hdd power directly if no external power(barrel & type-c) 
    // due to external power supplied 5v/12v to hdd via buck/nmos, the buck/nmos must be closed if no exteral power
    // do NOT inform usb to turn off disk because abnormal 5v/12v causes hdd cannot be communicated  
    if( ! adapter_contract)
    {
//        rt1711_turn_off_hdd_power();
    }            
    
    ac_state = ac_unattached;        
}
//--------------------------------------------------------------------------

void hd_task(void)
{
    hd_state();
}

void hd_clr_busy_wait_turn_on(void)
{
    // task idle, hdd is stop
    hd_busy = 0;
    
    hd_state = hd_wait_turn_on;
}

void hd_wait_turn_on(void)
{
    if (usb_req_turn_on_disk)
    {
    #if (HDD_5V_ONLY == DRIVE_HDD_PWR_TYPE)
        if (!tcpm_reject_turn_on_disk && (adapter_contract || ac_attached)) )
    #else
        if (!tcpm_reject_turn_on_disk && (adapter_contract || ac_attached || (SAFE5V_25MV < sink_from_computer_25mv)) ) 
    #endif
        {
U           ds("h:star");
    
            // turn on directly
            rt1711_turn_on_hdd_power();
            hd_state = hd_ready;
        }
    }
}

void hd_ready(void)
{
    if (tcpm_reject_turn_on_disk)  // TCPM request to turn off hdd
    {
U       ds("h:offT");
        hd_state = hd_req_usb_turn_off_hdd;
        hd_busy = 1;
    }
    else if (!usb_req_turn_on_disk)  // USB request to turn off hdd
    {
//U       ds("h:offU");
        hd_state = hd_usb_turn_off_hdd;
        hd_busy = 1;
    }
    else
    {
        // task idle, hdd is idle
        hd_busy = 0;
    }
}

void hd_req_usb_turn_off_hdd(void)
{
    // request usb to turn off hdd
    force_spin_down_hdd();
    
    hd_state = hd_wait_usb_turn_off_hdd;
}

void hd_wait_usb_turn_off_hdd(void)
{
    // wait usb to turn off hdd
    if( ! usb_req_turn_on_disk)
    {
        hd_state = hd_usb_turn_off_hdd;
    }
}

void hd_usb_turn_off_hdd(void)
{
U ds("h:stop");
  
    // cut power directly
    rt1711_turn_off_hdd_power();

    //
    hd_state = hd_clr_busy_wait_turn_on;    
}

//--------------------------------------------------------------------------

void init_tcpc(void)
{
    unsigned char i;
    
    enable_cc_handle = 1;
	//rt1711_i2c_write8(index, RT1711P_REG_SOFTRESET, 1);
    write8_tcpc_reg(0, RT1711P_REG_SOFTRESET, 1);


	// DelayMs(2);
	write_mcu_dwcnt_1ms(2);
	while(read_mcu_dwcnt_1ms());	  
    
    // read Device ID
    read_tcpc_reg(0, TCPC_REG_DEVICE_ID, 2);
    device_id = i2c_data[0] + (i2c_data[1] << 8);
    
    // calculat size
    reg_tab_size = sizeof(init_reg_tab) / 2;

	// setting for computer/adapter port
	for (i = 0; i < reg_tab_size; i++)
	{ 
        write8_tcpc_reg(0, init_reg_tab[i][0], init_reg_tab[i][1]);
	}

    // setting only for computer port
    //if (PARTNER_COMPUTER == partner_type)
    {      
        // Enable VBus DAC, enable VDC detection
        write8_tcpc_reg(0, RT1711P_REG_VBUS_ADDA_CTRL, 0xD4);
        
        // enable VDC detection INT
        write8_tcpc_reg(0, RT1711P_REG_RT_MASK, 0x80);
        // enable OVP and OCP
        write8_tcpc_reg(0, TCPC_REG_FAULT_CONTROL, 0x84);      

    	// set fastest ramping up/down rate
    	tcpc_reg_set_bits(0, RT1711P_REG_VBUS_ADDA_CTRL, 0x30);
          	
    	rt1711_disable_vbus();
    	
    	rt1711_turn_off_all_nmos();
    	
    	rt1711_turn_on_port0_pmos();
    	
    	// always turn on hdd 5v/12v NMOS for hardware control automatically
    	//rt1711_turn_on_hdd_5v_nmos();
    	//rt1711_turn_on_hdd_12v_nmos();
    }
	/*
    // setting only for adapter port
    if (PARTNER_ADAPTER == partner_type)
    {      
        // Enable VBus DAC, disable ADC detection
        write8_tcpc_reg(port, RT1711P_REG_VBUS_ADDA_CTRL, 0xD5);  
        
        // enable ADC detection INT
        // write8_tcpc_reg(port, RT1711P_REG_RT_MASK, 0X80);
        
        rt1711_turn_on_port1_pmos();
    }
    */
    #if RT1711P_VER_D_ALLOW_ONLY
    {
        if (RT1711P_DEV_ID_D != device_id)
        {
            uart_ch(port); uart_str("NOT VER D_0x"); uart_ch(device_id >> 8); uart_ch(device_id);
        
            enable_port = 0;       
        }
    }
    #elif RT1711P_VER_E_ALLOW_ONLY
    {
        if (RT1711P_DEV_ID_E != device_id)
        {
            uart_ch(port); uart_str("NOT VER E_0x"); uart_ch(device_id >> 8); uart_ch(device_id);
        
            enable_port = 0;       
        }        
    }
    #endif
}


//--------------------------------------------------------------------------

bit tcpm_is_busy(void)
{
//return 1;

    // alert pin or combined busy flag or hdd busy flag(rising from usb side) 
    if(combined_busy || hd_busy || ( ! (aGPIO_IO_REG[1] & 0x02)))
    {
        // busy
        return 1;
    }
    else
    {
        // idle
        return 0;        
    }
}

//--------------------------------------------------------------------------

void tcpm_init(void)
{
    // UART debug initial
U    dbg_initial();
    
    // initial global varibles
#if (GPIO6_FREE)
    vbus_detected = 0;
#endif

#if SEND_GET_SNK_CAP_REQUEST_ONCE_OPT
    dpm_get_snk_cap_request_sent = 0;
#endif

//--------------------------------------------------------

	msg_Received = 0;
	msg_Transmit = 0;
	msg_trans_NG = 0;
	msg_Disc = 0;
	msg_trans_OK = 0;
	VBUS_Alarm_Hi = 0;
	VBUS_Alarm_Lo = 0;
	ErrorRecovery_Timer_Flag = 0;
//--------------------------------------------------------


	i2c_nak = 0;
	i2c_timeout = 0;
    ac_inserted = 0;
    ac_attached = 0;
    adapter_contract  = 0;
    tcpm_reject_turn_on_disk = 0;
    usb_req_turn_on_disk = 0;
    sink_from_computer_25mv = 0;
    is_data_role_ufp = 0;
    
    // power adapter
    barrel_adapter_25mv = 0;
    type_c_adapter_25mv = 0;
    
    // busy flag, combine all port 
    pe_all_busy = 0;
    tx_all_busy = 0;
    
    // the power suppied from adapter port
    adapter_pwr_100mw = calc_power_100mw(VOL20V_25MV, CUR3A0_10MA);
    
    // reset local hw
    reset_local_hw();
	port=0;
    // reset all port   
    //for(port=0; port<PORT_COUNT; port++)
    {
        // clear port-x variables
        clear_xdata16(MEM_ADDR_PORTX, MEM_ADDR_SIZE);  

        // modify the timer when boot and opeating as sink
        modify_sink_wait_cap = 1;

        {
            enable_port = 1;
            partner_type = PARTNER_COMPUTER;
            usb_comm_capable = 1;
            //external_power = 1;  // todo, 需由 port1 是否接上 adapter 來決定
            //support_power_swap = 0;
            //support_data_swap = 0;  // always be UFP       
            
            // init TCPC
            init_tcpc();
            
            bak_pe_state[0] = _PE_SNK_Unattached;
            bak_tx_state[0] = tx_wait_for_message_request;
            
            // copy port-x to port-0 
            copy_xdata16(MEM_ADDR_PORTX, MEM_ADDR_PORT0, MEM_ADDR_SIZE);                 
        }
		
    } 
 
    // disable power path
    rt1711_turn_off_all_nmos();
    rt1711_disable_sys_5v_boost();
    rt1711_disable_vbus_buck();
    rt1711_disable_hdd_5v_buck();
    rt1711_disable_hdd_12v_buck();
    
    
    // AC State Machine
    ac_state = ac_unattached;

    // HD State Machine
    hd_state = hd_clr_busy_wait_turn_on;
	_PE_state = _PE_SNK_Unattached;
	_msg_state = _Protocol_Lay_Standby;
}

//--------------------------------------------------------------------------
unsigned char tErrorRecovery_1ms_r(void)
{
	return aMCU_DWCNT14_1MS;
}
void tErrorRecovery_1ms_w(unsigned char value)
{
	aMCU_DWCNT14_1MS = value;
}
unsigned char tSenderResponse_1ms_r(void)
{
	return aMCU_DWCNT14_1MS;
}
void tSenderResponse_1ms_w(unsigned char value)
{
	aMCU_DWCNT14_1MS = value;
}
unsigned char tCCDebounce_10ms_r(void)
{
	return aMCU_DWCNT18_10MS;
}
void tCCDebounce_10ms_w(unsigned char value)
{
	aMCU_DWCNT18_10MS = value;
}
unsigned char tTypeCSinkWaitCap_10ms_r(void)
{
	return aMCU_DWCNT18_10MS;
}
void tTypeCSinkWaitCap_10ms_w(unsigned char value)
{
	aMCU_DWCNT18_10MS = value;
}
unsigned char tPSTransition_10ms_r(void)
{
	return aMCU_DWCNT18_10MS;
}
void tPSTransition_10ms_w(unsigned char value)
{
	aMCU_DWCNT18_10MS = value;
}

void tcpm_main(void)
{	 
	if(!(aGPIO_IO_REG[1] & 0x02))										// alert if GPIO9 is pulled to low
	{
		alert_handle();													// hanle all albert condition	
	}
	
    _PolicyEngine_task();
	_Protocol_Lay_task();
	U dbg_print_one_character();										// only print one character once	    
}
void _PolicyEngine_task(void)
{
	_PE_state();
}
void _Protocol_Lay_task(void)
{
	_msg_state();
}
void _Protocol_Lay_Standby(void)
{
	unsigned char i;
	if(msg_Received)
	{
		read_tcpc_reg(0, TCPC_REG_RECEIVE_BYTE_COUNT, 4);	    
	    rx_frame_type.SOP = i2c_data[1] & 0x07;							// read RX_BUF_FRAME_TYPE, bit0~2
	    
	    // read RX_BUF_HEADER_BYTE
	    rx_header.MsgType       =  i2c_data[2]       & 0x0F;
	    rx_header.PortDataRole  = (i2c_data[2] >> 5) & 0x01;
	    rx_header.SpecRevision  = (i2c_data[2] >> 6) & 0x03;
	    rx_header.PortPowerRole =  i2c_data[3]       & 0x01;
	    rx_header.MsgID         = (i2c_data[3] >> 1) & 0x07;
	    rx_header.NumOfDataObj  = (i2c_data[3] >> 4) & 0x07;
	
	    // build rx_msg_type, for quickly identify and reduce code size
	    if(rx_header.NumOfDataObj)  									// data message
	    {
	        rx_msg_type = rx_header.MsgType + 16;						// 16~31 stores Data Message
	    }
	    else
	    {
	        rx_msg_type = rx_header.MsgType;							// 0~15 stores Control Message
	    }
	    for(i=0; i<rx_header.NumOfDataObj; i++)
	    {
	        read_tcpc_reg(0, i * 4 + TCPC_REG_RX_BUF_OBJ1_BYTE, 4);
	        swap_4_byte(i2c_data, (unsigned char xdata*)&rx_object[i]);                     
	    }
		msg_Received = 0;	
	}
	if(msg_Transmit)
	{
		ds("msg tx");
        U8_write_tcpc_reg(TCPC_REG_TRANSMIT , 0x30);					//SOP and retry 3 times.
        _msg_state = _PRL_Rx_Wait_for_PHY_message;
	}
}
void _PRL_Rx_Wait_for_PHY_message(void)
{
    if(msg_Disc)														//set retry timer and re-send 
    { 
        ds("disc");
        msg_Disc = 0;
        //prl_message_fail = 0x01;
    }
    else if(msg_trans_NG)												// should info PE error
    { 
        ds("fail");
        msg_trans_NG = 0;
        msg_Transmit = 0;
        _msg_state = _Protocol_Lay_Standby;
    }
    else if(msg_trans_OK)
    {
        ds("sus");
        msg_Transmit = 0;
        msg_trans_OK = 0;
        MessageIDCounter++;
        if(MessageIDCounter>7)
        {
            MessageIDCounter= 0;
        }
        _msg_state = _Protocol_Lay_Standby;
    }
}
void _PRL_send_msg_message(void)
{
	unsigned char i;
    // argument: tx_header, tx_frame_type, tx_object                         
    i2c_data[0] = 2 + (4 * tx_header.NumOfDataObj);						// fill TRANSMIT_BUFFER, TCPC1.0 p60
    
    // fill TX_BUF_HEADER
    i2c_data[1] =  tx_header.MsgType            + 
                  (tx_header.PortDataRole << 5) + 
                  (tx_header.SpecRevision << 6);
                  
    i2c_data[2] =  tx_header.PortPowerRole      + 
                  (tx_header.MsgID        << 1) + 
                  (tx_header.NumOfDataObj << 4);                                                                   
        
    
    write_tcpc_reg(0, TCPC_REG_TRANSMIT_BYTE_COUNT, 3);					// send TRANSMIT_BUFFER and TX_BUF_HEADER
    
    
    for(i=0; i<tx_header.NumOfDataObj; i++)								// send TX_BUFFER_DATA_OBJECTS if need
    {
        swap_4_byte((unsigned char xdata*)&tx_object[i], &i2c_data[0]);
        write_tcpc_reg(0, i * 4 + TCPC_REG_TX_BUF_OBJ1_BYTE, 4);
    }
    
    
    tx_frame_type.RetryCounter = 3;										// must be 3, PD2 V1.1 p195
    
    // fill TRANSMIT, TCPC1.0 p60
    i2c_data[0] =  tx_frame_type.SOP + (tx_frame_type.RetryCounter << 4);
                  
    
    
    write_tcpc_reg(0, TCPC_REG_TRANSMIT, 1);							// send TRANSMIT
        
	_msg_state = _Protocol_Lay_Standby;
}
void alert_handle(void)
{
	//U16_temp = U16_read_tcpc_reg(TCPC_REG_ALERT);						// read alert status
	read_tcpc_reg(0, TCPC_REG_ALERT, 2);
    alert0 = i2c_data[0];
    alert1 = i2c_data[1];
	U ds("a"); db(alert1); db(alert0); 
	read_tcpc_reg(0, TCPC_REG_ALERT, 2);
	if(alert0 & 0x01)	 												// CC Status
	{
		cc_status = U8_read_tcpc_reg(TCPC_REG_CC_STATUS);       
	    cc1_state  =  cc_status & 0x03;
	    cc2_state  = (cc_status & 0x0C) >> 2;
		U ds("CC"); db(cc_status);
		alert_reg_clr(0x01, 0x00);	
	}
	if(alert0 & 0x02)	 												// Power Status
	{  
        pwr_status = U8_read_tcpc_reg(TCPC_REG_POWER_STATUS);
		vconn_present = vbus_present = 0;
		U ds("PW");	db(pwr_status);
		if(pwr_status & 0x02)
            vconn_present = 1;
		if(pwr_status & 0x04)
            vbus_present = 1;
		alert_reg_clr(0x02, 0x00);			
	}
	if(alert0 & 0x04)	 												// Receive SOP* Message Status
	{
	//	U ds("Rx");
		msg_Received = 1;
		alert_reg_clr(0x04, 0x00);
	}
	if(alert0 & 0x08)	 												// Received Hard Reset
	{
	 	U ds("HR");		
		alert_reg_clr(0x08, 0x00);
	}
	if(alert0 & 0x10)	 												// Transmit SOP* Message Failed
	{
		U ds("FAIL");
		msg_trans_NG = 1;
		alert_reg_clr(0x10, 0x00);
	}
	if(alert0 & 0x20)	 												// Transmit SOP* Message Discared
	{
		U ds("DISCARD");
		msg_Disc = 1;
		alert_reg_clr(0x20, 0x00);
	}
	if(alert0 & 0x40)	 												// Transmit SOP* Message Successful
	{
		msg_trans_OK = 1;
		alert_reg_clr(0x40, 0x00);
	}
	if(alert0 & 0x80)	 												// VBUS Voltage Alarm Hi
	{
		U ds("HI");
		VBUS_Alarm_Hi = 1;
		tcpc_reg_set_bits(0, TCPC_REG_POWER_CONTROL, 0x20);				// disable voltage alarm
		alert_reg_clr(0x80, 0x00);
	}
	if(alert1 & 0x01)	 												// VBUS Voltage Alarm Lo
	{
		U ds("LO");
		VBUS_Alarm_Lo = 1;
		tcpc_reg_set_bits(0, TCPC_REG_POWER_CONTROL, 0x20);				// disable voltage alarm
		alert_reg_clr(0x00, 0x01);		
	}
	if(alert1 & 0x02)	 												// Fault
	{		
        fault_status = U8_read_tcpc_reg(TCPC_REG_FAULT_STATUS);			// read fault status
		U ds("FAU"); db(fault_status);
		alert_reg_clr(0x00, 0x02);
		if(fault_status & 0x02)
		{
			tcpc_reg_set_bits(0, TCPC_REG_FAULT_STATUS, 0x02);
		}
		else if(fault_status & 0x04)
		{
			tcpc_reg_set_bits(0, TCPC_REG_FAULT_STATUS, 0x04);
		}
		else if(fault_status & 0x08)
		{
			tcpc_reg_set_bits(0, TCPC_REG_FAULT_STATUS, 0x08);
		}
		else if(fault_status & 0x10)
		{
			tcpc_reg_set_bits(0, TCPC_REG_FAULT_STATUS, 0x10);
		}
	}
	if(alert1 & 0x04)	 												// Rx Buffer Overflow
	{
		U ds("OF");  													//OverFlow
		alert_reg_clr(0x00, 0x04);
	}
	if(alert1 & 0x08)	 												// VBUS Sink Disconnect Detected
	{
		U ds("DI");
		alert_reg_clr(0x00, 0x08);
	}
	//alert_reg_clr(i2c_data[0], i2c_data[1]); 							// Write 1 to clear alert
}
void _PE_SNK_Hard_Reset(void)
{
	U ds("k:HR");
	HardResetCounter++;
	     
    tcpm_hard_reset();													// clear    
    
    rt1711_disable_vbus();												// disable vbus if PR_SWAP fail from sink to source
    turn_off_vconn();               
    
    hrst_sent = 0;														// send hard reset  
    _msg_state = tx_hr_request_hard_reset;
	_PE_state = _PE_SNK_Trans_to_default;
}
void _PE_SNK_Trans_to_default(void)
{
	U ds("k:dft");
	explicit_contract = 0;												// Implicit Contract
	pd_connected = 0;													// PD = not Connected
	//_PE_state = _PE_SNK_Startup;
}
void _PE_ErrorRecovery(void)
{
	if(ErrorRecovery_Timer_Flag)
	{
		ErrorRecovery_Timer_Flag = 0;
		U   ds("e_r");   
	    rt1711_disable_vbus();
	    turn_off_vconn();
	   
	    //vbus_present_25mv = SAFE0V_25MV;								// update immediately(not wait voltage at 0v)
	    //disable_tcpc_receiver();
	    //enable_cc_handle = 0;    
	
	    disable_usb_gpio6_pull_up();									// for usb vbus sense    
	    U8_write_tcpc_reg(TCPC_REG_ROLE_CONTROL, 0x0F);					// high-impedance to ground on CC
	    cc1_state = cc2_state = CC_OPEN;
	    	    
	    tErrorRecovery_1ms_w(25);										// tErrorRecovery
	}  
    //if(!tErrorRecovery_1ms_r())										// wait time out
	//	_PE_state = _PE_SNK_Unattached;
}

void _PE_SNK_Unattached(void)
{
	U ds("k:Un");
	U8_write_tcpc_reg(TCPC_REG_ROLE_CONTROL, 0x0A);						// Set role control = Sink(Rd)	
    tcpc_reg_clr_bit(TCPC_REG_POWER_CONTROL, 0x04);						// cannot force discharge in sink
	if((CC_OPEN != cc1_state) || (CC_OPEN != cc2_state))
	{
		tCCDebounce_10ms_w(15);											// 100~200ms   
		_PE_state = _PE_SNK_AttachWait;  
	}
}

void _PE_SNK_AttachWait(void)
{
	if(tCCDebounce_10ms_r())
   	{
		if(vbus_present)
		{
			U ds("k:Wa");
			_PE_state = _PE_SNK_Attached;
		}
	}
	else
	{
		if((CC_OPEN == cc1_state) && (CC_OPEN == cc2_state))
			_PE_state = _PE_SNK_Unattached;	
	}
}
void _PE_SNK_Attached(void)
{
	U ds("k:ed");
	if(CC_OPEN != cc1_state)
    {
        U8_write_tcpc_reg(TCPC_REG_TCPC_CONTROL, 0x00);  				// BMC in cc1, vconn in cc2    
        set_usb3_typec_tx1_rx1();                        
    }
    else
    {
        U8_write_tcpc_reg(TCPC_REG_TCPC_CONTROL, 0x01);  				// BMC in cc2, vconn in cc1 
        set_usb3_typec_tx2_rx2();                        
    }
	_PE_state = _PE_SNK_Startup;

}
void _PE_SNK_Startup(void)
{
	U ds("k:Sr");

	tcpm_soft_reset();													// ResetProtocolLayer
    _PE_state = _PE_SNK_Discovery;
}
void _PE_SNK_Discovery(void)
{
	if(vbus_present)
	{
	 	ds("k:5V");
		aMCU_DWCNT17_10MS = 1;											// Richtek says alarm lo would average vbus voltage in 6ms
		_PE_state = _PE_SNK_DiscoveryWait;
	}
}

void _PE_SNK_DiscoveryWait(void)
{ 
	if(!aMCU_DWCNT17_10MS)
	{	
		U ds("k:DyW");
		tTypeCSinkWaitCap_10ms_w(62);									// 320~620ms
		U8_write_tcpc_reg(TCPC_REG_MESSAGE_HEADER_INFO, 0x02);			// Enable receive detect message header   
		U8_write_tcpc_reg(TCPC_REG_RECEIVE_DETECT, 0x21);				// Disable SOP message, something error 
		alarm_lo_threshold_25mv = VOL4V0_25MV;
        alarm_hi_threshold_25mv = VOL5V5_25MV;
        port = 0;
		config_voltage_alarm();
		enable_usb_gpio6_pull_up(); 	
		_PE_state = _PE_SNK_Wait_for_Cap;
	}
}

void _PE_SNK_Wait_for_Cap(void)
{
	unsigned char i;
	if(tTypeCSinkWaitCap_10ms_r())
	{		
		if(MSG_SOURCE_CAPABILITIES == rx_msg_type) 
		{
			U ds("m ");
			U ds(msg_id2string[tx_msg_type]);
  
	        HardResetCounter = 0;										// reset HardResetCounter to zero      	        
	        pd_connected = 1;											// PD = Connected
			dpm_parser_src_pdo(0);
    
		    partner_supp_dr_swap   = cmm_pdo_list[4];
		    partner_usb_commun     = cmm_pdo_list[5];
		    partner_external_power = cmm_pdo_list[6];
		    partner_supp_pr_swap   = cmm_pdo_list[8];
		
		    // default request 5v 
		    max_cur_10ma = cmm_pdo_list[0];
		    max_vol_50mv = cmm_pdo_list[1]; 
		    pdo0_10ma    = max_cur_10ma;   
		    requested_obj_pos = 0;			
	        for(i=1; i<rx_header.NumOfDataObj; i++)						// parser others power data object
	        {	            
	            dpm_parser_src_pdo(i);									// parser Power Data Object, from low to high	    	            
	            cur_10ma = cmm_pdo_list[0];								// Maximum Current in 10mA units
	            vol_50mv = cmm_pdo_list[1];								// Voltage in 50mV units    
					            
	            if(vol_50mv > max_vol_50mv)								// select max voltage
	            {
	                max_vol_50mv = vol_50mv;
	                max_cur_10ma = cur_10ma;
	                requested_obj_pos = i;
	            }
	        }
			CapabilityMismatch = 0;
		    requested_obj_pos++;		    
		    
		    req_opr_vol_25mv = req_opr_vol_50mv * 2;					// update voltage with unit 25mv		    
			U ds("v"); dw(req_opr_vol_25mv);  		
		
		    // decide Request's Power Data Object 
		    cmm_pdo_list[0] = req_max_opr_cur_10ma;						// MaximumOperatingCurrent_10mA
		    cmm_pdo_list[1] = req_opr_cur_10ma;							// OperatingCurrent_10mA
		    cmm_pdo_list[2] = 0;                    					// Reserved
		    cmm_pdo_list[3] = 1;										// NoUSBSuspend
		    cmm_pdo_list[4] = usb_comm_capable;							// USBCommunicationsCapable
		    cmm_pdo_list[5] = CapabilityMismatch;						// CapabilityMismatch
		    cmm_pdo_list[6] = 0;										// GiveBackFlag
		    cmm_pdo_list[7] = requested_obj_pos;						// ObjectPosition
		    cmm_pdo_list[8] = 0;										// Reserved

			_PE_state = _PE_SNK_Evaluate_Cap;			
		}
	}	
	else																// time out
	{
		U ds("k:WTo");
		if (HardResetCounter > HARDRESET_COUNT)
		{
			if(pd_connected)
				_PE_state = _PE_ErrorRecovery;
		}
		else
			_PE_state = _PE_SNK_Hard_Reset;	
	} 
}
void _PE_SNK_Evaluate_Cap(void)
{
	U ds("k:EvC");
	explicit_contract = 0;												// Implicit Contract
    tx_frame_type.SOP = TX_SOP_0P;										// decide Request Message's SOP  
    
    set_tx_data_header(MSG_REQUEST, 1);									// decide Request Message's Header
    
    shift_tab = req_pdo_shift;
    list_count = sizeof(req_pdo_shift);
    dpm_combine_cmm_list_to_tx_obj(0);
                
    // backup, re-send if port partner send Wait
    xdata_copy(&tx_frame_type, &req_frame_type, sizeof(req_frame_type));
    xdata_copy(&tx_header,     &req_header,     sizeof(req_header));
    xdata_copy(&tx_object[0],  &req_object,     sizeof(req_object));	// Request Message only have 1 data object

	_PE_state = _PE_SNK_Select_Cap;
}
void _PE_SNK_Select_Cap(void)
{
	U ds("k:sel");  
    rx_msg_type = 0;
	pd_connected = 1;
	tSenderResponse_1ms_w(30);
    _msg_state = tx_construct_message;									// 24~30ms        
    _PE_state = _PE_SNK_Select_Cap_Wait; 
}
void _PE_SNK_Select_Cap_Wait(void)
{
	if(tSenderResponse_1ms_r())
	{
		switch(rx_msg_type)
		{
			case MSG_ACCEPT:
			{
				U ds("k:got");
				port = 0;		        
		        get_present_vbus(port);									// partner start to change voltage
		        if(req_opr_vol_25mv > vbus_present_25mv)
		        {
		            // increase voltage, to monitor volage cannot over request voltage+10%
		            alarm_hi_threshold_25mv = req_opr_vol_25mv * 11 / 10;
		        }
		        else
		        {
		            // decrease voltage, to monitor volage cannot over present voltage+10%
		            alarm_hi_threshold_25mv = vbus_present_25mv * 11 / 10;
		        }               
		        alarm_lo_threshold_25mv = VOL4V0_25MV;					// keep same setting
		        config_voltage_alarm();      
		                
		        rx_msg_type = 0;
				_PE_state = _PE_SNK_Transition_Sink;
				break;
			}
			case MSG_WAIT:	case MSG_REJECT:
			{
				if(explicit_contract)
				{
					_PE_state = _PE_SNK_Ready;
				}
				else
				{
					_PE_state = _PE_SNK_Wait_for_Cap;
				}
				break;
			}
			default:
			{
				CapabilityMismatch = 1;
				//_PE_state = ?;
				break;
			}

		}	
	}
	else															// time out
	{
		U ds("k:TO");
		_PE_state = _PE_SNK_Hard_Reset;
	}
}

void _PE_SNK_Transition_Sink(void)
{
	explicit_contract = 1;
	U ds("k:TS");

	tPSTransition_10ms_w(50);									// 450~550ms	
	_PE_state = _PE_SNK_Transition_Sink_Wait;
}
void _PE_SNK_Transition_Sink_Wait(void)
{
	if(tPSTransition_10ms_r())
	{
		if (MSG_PS_RDY == rx_msg_type)
		{
			U ds("k:Rdy");
			_PE_state = _PE_SNK_Ready;
		}	
	}
	else														// Time out
	{
		U ds("k:TO");
		_PE_state = _PE_SNK_Hard_Reset;	
	}
}
void _PE_SNK_Ready(void)
{	
	explicit_contract = 1;
	if(MSG_GET_SNK_CAP == rx_msg_type)
    {      
		U ds("k:gsc");
        _PE_state = _PE_SNK_Give_Sink_Cap;     
    }
	if(MSG_SOURCE_CAPABILITIES == rx_msg_type)
		_PE_state = _PE_SNK_Evaluate_Cap;
}
void _PE_SNK_Get_Sourced_Cap(void)
{
	// do something ?
}
void _PE_SNK_Give_Sink_Cap(void)
{
	U ds("k:GSC");
	port = 0;
	rx_msg_type = 0; 	
	dpm_build_snk_cap_msg();										// send a Sink_Capabilities Message
	_PE_state = _PE_SNK_Ready;
	_msg_state = tx_construct_message;
}
// Sink Only ---------------------------------------------------------------

void tcpm_read_tcpc_reg(unsigned char port_num, unsigned char buf[])
{
    unsigned short k;
    unsigned char bak;

U uart_str("\r\napp:read_tcpc_reg");

    // protect
    if(port_num >= PORT_COUNT)
    {
        return;
    }
    
    // backup
    bak = port;
    
    // change to specific
    port = port_num;   

    // read all TCPC register
    for(k=0; k<256; k+=4)
    {       
        read_tcpc_reg(port, k, 4);
        xdata_copy(i2c_data, &buf[k], 4);
    }
    
    // restore
    port = bak;    
}

void tcpm_read_tcpm_state(unsigned char port_num, unsigned char buf[])
{
U uart_str("\r\napp:read_tcpm_state");
    
    // protect
    if(port_num >= PORT_COUNT)
    {
        return;
    }
    
    // pe_state
    xdata_copy(&bak_pe_state[port_num], &buf[0], 2);  //buf[0]: bit15~8, buf[1]: bit7~0 
    
    // tx_state
    xdata_copy(&bak_tx_state[port_num], &buf[2], 2);  //buf[2]: bit15~8, buf[3]: bit7~0   
}

//--------------------------------------------------------------------------

bit tcpm_allow_fw_upate(void)
{
	return 1;
}

//--------------------------------------------------------------------------

bit  tcpm_is_pd_power_enough(void)
{
    return 1;
}

//--------------------------------------------------------------------------

bit  tcpm_check_i2c_tout(void)
{
	return i2c_timeout;
}

//--------------------------------------------------------------------------

bit  tcpm_check_i2c_nak(void)
{
	return i2c_nak;
}

//--------------------------------------------------------------------------

bit  tcpm_is_ufp(void)
{
//    return 1;
    return is_data_role_ufp;
}

//--------------------------------------------------------------------------

void tcpm_turn_off_hdd_power(void)
{
U ds("u_of");

    // task busy (rising from usb side) 
    hd_busy = 1;
    
    // clear flag
    usb_req_turn_on_disk = 0;
}

//--------------------------------------------------------------------------

void tcpm_turn_on_hdd_power(void)
{  
U ds("u_on");
    
    // task busy (rising from usb side) 
    hd_busy = 1;
    
    // rising flag
    usb_req_turn_on_disk = 1;
    
    // turn on hdd power in state: hd_wait_new_src_cap_sent    
}

//--------------------------------------------------------------------------

bit tcpm_turn_off_hdd_power_done(void)
{
    if(hd_busy || (!(aGPIO_IO_REG[1] & 0x02)) /*|| tcpm_is_busy()*/)
    {
        return 1;
    }
    return 0;
}
//--------------------------------------------------------------------------
#if (GPIO6_FREE)
bit tcpm_vbus_detect(void)
{
    return vbus_detected;
}
#endif
//--------------------------------------------------------------------------

void tcpm_assert_tcpc_gpio(unsigned char port_index, unsigned char gpio_index)
{
    // output high
    write8_tcpc_reg(port_index, RT1711P_REG_GPIO0+gpio_index, 0x1E);
    
    aMCU_DWCNT1_1US=10;
    while(aMCU_DWCNT1_1US);
    
    // output low
    write8_tcpc_reg(port_index, RT1711P_REG_GPIO0+gpio_index, 0x1C);
}

//--------------------------------------------------------------------------

#ifdef UART
void dbg_initial(void)
{
    dbg_buf_index = 0;
    dbg_print_index = 0;    
}

void dbg_print_one_character(void)
{
    // only print one character once
    if(dbg_buf_index)
    {
        dbg_busy = 1;
        
        // print
        uart_out(dbg_buf[dbg_print_index]);
        dbg_print_index++;

        // all data in buffer has been printed    
        if(dbg_print_index >= dbg_buf_index)
        {
            dbg_buf_index = 0;
            dbg_print_index = 0;    
            dbg_busy = 0;
        }
    }
}

void ds(char *string)
{
    if((dbg_buf_index+2) >= DBG_MAX_BUF_INDEX)
        return;

    dbg_busy = 1;
   
    dbg_buf[dbg_buf_index]='\n';            dbg_buf_index++;
    dbg_buf[dbg_buf_index]=hex_table[port]; dbg_buf_index++; 

    while(*string)
    {
        if(dbg_buf_index >= DBG_MAX_BUF_INDEX)
            return;
        
        dbg_buf[dbg_buf_index] = *string;
        dbg_buf_index++;  // place here for reduce LIB_CODE
        string++;         // place here for reduce LIB_CODE
    }
}

void db(unsigned char value)
{   
    if((dbg_buf_index+2) >= DBG_MAX_BUF_INDEX)
        return;
    
    dbg_buf[dbg_buf_index] = hex_table[value/0x10]; 
    dbg_buf_index++;
    dbg_buf[dbg_buf_index] = hex_table[value%0x10];  
    dbg_buf_index++;  
}

void dw(unsigned short value)
{       
    if((dbg_buf_index+4) >= DBG_MAX_BUF_INDEX)
        return;

    db(value / 0x100);
    db(value % 0x100);        
}
#endif

void usb_enter_lowest_power(void)
{
    // spend 5.08mA
#if 1
            tcpc_reg_set_bits(0, RT1711P_REG_AUTOIDLE_EN,0x08);
            tcpc_reg_set_bits(1, RT1711P_REG_AUTOIDLE_EN,0x08);
#endif

    
            aREGU_CTRL_REG1 |= REGULATOR_IDDQ;

            // Switch regulator output voltage
            aREGU_CTRL_REG0 &= ~SW_REGU_LEVEL_MASK;
            aREGU_CTRL_REG0 |= REGU_1P075V;
    
            // Set LDO 3.3V output to 2.9V
            // - Clear [0x70BF] bit [7:4] to 0
            aREGU_CTRL_REG1 &= ~0xF0;
                    
            aUSB3_LTSSM_CTRL1 &= (~0x10);

            // write 1 to bit 3
            aSUS_CTRL_REG |= 0x08;
                        
            // write 1 to bit 1
            aSUS_CTRL_REG |= 0x02;

            // write 1 to bit 0
            aSUS_CTRL_REG |= 0x01;
U ds("k:LPWR");
            _nop_();
            _nop_();
            _nop_();
            _nop_();
            _nop_();
    
            _nop_();
            _nop_();
            _nop_();
            _nop_();
            _nop_();
            
U ds("k:WAKE");         
            // if wakeup by HW. Let RT1711P leave idel mode.
            tcpc_reg_clr_bits(0, RT1711P_REG_AUTOIDLE_EN,0x08);
            tcpc_reg_clr_bits(1, RT1711P_REG_AUTOIDLE_EN,0x08);    
} 

//--------------------------------------------------------------------------

#pragma RESTORE    // restore previous optimization level

//--------------------------------------------------------------------------

#endif

//--------------------------------------------------------------------------
