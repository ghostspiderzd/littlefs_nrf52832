/******************************Copyright (c)***********************************
*                Jiangsu Zhihai Electronic Technology Co., Ltd.
*                      Research & Development Department
-------------------------------------------------------------------------------
* @file    BLE_SET.h
* @author  Gu Dongdong
* @date    2018-10-16  
*******************************************************************************/
#ifndef BLE_SET_H
#define BLE_SET_H


#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "bsp.h"
#include "ble_gap.h"
#include "nrf_drv_saadc.h"

#define NRF_LOG_MODULE_NAME "CM18"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "ble_volmeas.h"

extern   uint32_t  PACK_NUM;
 
#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define MAX_LEN 50

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define ADVERTISING_LED_PIN             BSP_BOARD_LED_0                             /**< Is on when device is advertising. */
#define CONNECTED_LED_PIN               BSP_BOARD_LED_1                             /**< Is on when device has connected. */

#define LEDBUTTON_LED_PIN               BSP_BOARD_LED_2                             /**< LED to be toggled with the help of the LED Button Service. */
#define LEDBUTTON_BUTTON_PIN            BSP_BUTTON_0                                /**< Button that will trigger the notification event with the LED Button Service */

#define DEVICE_NAME                     "CM22"                                  /**< Name of device. Will be included in the advertising data. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED       /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            6                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         6                                           /**< Size of timer operation queues. */

#define VOLMEAS_READ_INTERVAL           APP_TIMER_TICKS(2, APP_TIMER_PRESCALER)  /**< ADC²ÉÑù¼ä¸ô 10ms¶ÁÈ¡Ò»´Îshort*/
#define SonPack_Send_INTERVAL           APP_TIMER_TICKS(7, APP_TIMER_PRESCALER)  /**< 20ms¼ä¸ô¶¨Ê±Æ÷*/
#define PACKING_TIM_INTERVAL            APP_TIMER_TICKS(7, APP_TIMER_PRESCALER)  /**< 20ms¼ä¸ô¶¨Ê±Æ÷*/
#define PWR_ON_OFF_INTERVAL          		APP_TIMER_TICKS(PWR_SAMP_TIME, APP_TIMER_PRESCALER)  /**< 50ms¼ä¸ô¶¨Ê±Æ÷*/
#define BLE_LED_ADV_INTERVAL          	APP_TIMER_TICKS(500, APP_TIMER_PRESCALER)  /**< 50ms¼ä¸ô¶¨Ê±Æ÷*/

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(8, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(30, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (1 second). */

#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory time-out (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_GPIOTE_MAX_USERS            1                                           /**< Maximum number of users of the GPIOTE handler. */
#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(5, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define PROTOCOL_VERSION				0x20
#define DATAPACKET_TYPE					0x02

#define PARAMETER_START					0xfefe

#define FRAME_HEADER_LEN				8


typedef enum
{
		BLE_LED_IDLE=0,
    BLE_LED_ADVERTISING,                /**< See \ref BSP_INDICATE_ADVERTISING.*/
    BLE_LED_CONNECTED,                  /**< See \ref BSP_INDICATE_CONNECTED.*/
} LED_STATE;

typedef struct
{
	uint8_t len;
	uint8_t data[MAX_LEN];
}data_struct;
extern data_struct gdata_struct;

typedef enum 
{
	HOST_CMD_START = 0x1000,
	HOST_CMD_STOP = 0x2000,	
	HOST_CMD_HUMPER = 0x3000,		//äººä½“å‚æ•°
	HOST_CMD_USERID = 0x4000,
	HOST_CMD_CLOCK  = 0x5000,
	HOST_CMD_REF_BP = 0x6000,		//æ ‡å®šè¡€åŽ‹
}HOSTCMD_ENUM;
extern HOSTCMD_ENUM gHostcmd;

typedef struct
{
	unsigned short type;
	unsigned short len;
	unsigned char *pbuff;
}TLVdata_struct;
extern TLVdata_struct gTLCdata;

typedef struct
{
	unsigned char height;
	unsigned char weight;
	unsigned char arterial_length;
	unsigned char gender;
	unsigned char age;
}human_parameters;

typedef struct 
{
	unsigned char year;
	unsigned char month;
	unsigned char day;
	unsigned char hour;
	unsigned char minute;
	unsigned char second;
}clock_struct;

typedef struct
{
	short ref_sys;	//æ”¶ç¼©åŽ‹
	short ref_dia;	//èˆ’å¼ åŽ‹
}ref_bp_struct;

typedef struct
{
	unsigned int 		user_id;					//ç”¨æˆ·ID
	human_parameters 	human_par;				//äººä½“å‚æ•°
	unsigned int 		clock;
	ref_bp_struct		ref_bp;					//æ ‡å®šçš„è¡€åŽ‹		
}controlcmd_data_strict;
extern controlcmd_data_strict gControlcmd;




extern void CM18_BLE_SERVICES_init(void);
void cmd_data(void);
void TLVdata_parsing(unsigned char *pTLVdatapacket,unsigned short packetlen);

#endif

