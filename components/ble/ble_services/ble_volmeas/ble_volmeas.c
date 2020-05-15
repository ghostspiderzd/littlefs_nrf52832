#include "ble_volmeas.h"
#include "ble_srv_common.h"
#include "sdk_common.h"
#include "boards.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h" 
#include "nrf_delay.h"
#include "nrf_drv_timer.h"
#include "BLE_SET.h"
uint32_t  PACK_NUM=0;

uint32_t saadc_callback_Count=0;
 
extern void application_timers_start(void);
extern void SonPackSend_timers_start(void);
 
extern void application_timers_stop(void);
extern void SonPackSend_timers_stop(void);
extern nrf_drv_timer_t TIMER_ADC;
/********************************************************************************************************
** 变量名称: SystemFlag
** 功能描述: 系统标志
********************************************************************************************************/
 unsigned int  SystemFlag = 0;
//____________________________________________________________________________________________
 

/**@brief Function for handling the Connect event.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_connect(ble_volmeass_t * p_lbs, ble_evt_t * p_ble_evt)
{
    p_lbs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_disconnect(ble_volmeass_t * p_lbs, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_lbs->conn_handle = BLE_CONN_HANDLE_INVALID;
}

static void on_write(ble_volmeass_t * p_lbs, ble_evt_t * p_ble_evt)
{}

void ble_volmeass_on_ble_evt(ble_volmeass_t * p_lbs, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_lbs, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
 
            on_disconnect(p_lbs, p_ble_evt);
            break;
            
        case BLE_GATTS_EVT_WRITE:
				on_write(p_lbs, p_ble_evt);   	 
				PACK_NUM=0;
        //sonPACK_NUM=0;
				saadc_callback_Count=0;
//				application_timers_start();//启动定时器
				//nrf_drv_timer_enable(&TIMER_ADC);//启动定时器
        break;
				
				case BLE_GATTS_EVT_HVC:

        break;
 
				case BLE_GATTS_EVT_SC_CONFIRM:
 
            break;
				
				case BLE_EVT_TX_COMPLETE://发送完成事件
 
            break;
 
        default:
            // No implementation needed.
            break;
    }
}
/**********************************************************************************************
 * 描  述 : 添加THI特性(thi是温度、湿度、人体红外检测的缩写)
 * 参  数 : 
 *          
 * 返回值 : 成功返回NRF_SUCCESS，否则返回错误代码
 ***********************************************************************************************/ 
static uint32_t volmeas_char_add(ble_volmeass_t * p_lbs, const ble_volmeass_init_t * p_volmeass_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
	  char_md.p_cccd_md         = NULL;
    char_md.p_cccd_md         = &cccd_md;

    ble_uuid.type = p_lbs->uuid_type;
    ble_uuid.uuid = VOLMEASS_UUID_VOL_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    //BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
		/*于11月28日修改，修改该长度防止最后一帧数据自动填充*/
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;

		attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
		attr_char_value.max_len      = 20;
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_lbs->service_handle,
              &char_md,&attr_char_value,&p_lbs->mpu_char_handles);
}

static uint32_t volmeas_char_add_2(ble_volmeass_t * p_lbs, const ble_volmeass_init_t * p_volmeass_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.write = 1;
    char_md.char_props.write_wo_resp= 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
	  char_md.p_cccd_md         = NULL;
    char_md.p_cccd_md         = &cccd_md;

    ble_uuid.type = p_lbs->uuid_type;
    ble_uuid.uuid = VOLMEASS_UUID_VOL_CHAR_2;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
		/*于11月28日修改，修改该长度防止最后一帧数据自动填充*/
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;

    attr_char_value.init_len     = 1;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = 20;
    //attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_lbs->service_handle,
              &char_md,&attr_char_value,&p_lbs->led_char_handles);
}


uint32_t ble_volmeass_init(ble_volmeass_t * p_volmeass, const ble_volmeass_init_t * p_volmeass_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure.
    p_volmeass->conn_handle       = BLE_CONN_HANDLE_INVALID;
    p_volmeass->data_handler = cmd_data;

    // Add service.
    ble_uuid128_t base_uuid = {VOLMEASUUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_volmeass->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_volmeass->uuid_type;
    ble_uuid.uuid = VOLMEASS_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_volmeass->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add characteristics.
    err_code = volmeas_char_add(p_volmeass, p_volmeass_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    //添加特征值2
    err_code = volmeas_char_add_2(p_volmeass, p_volmeass_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    return NRF_SUCCESS;
}

/********************************************************************************************************
** 函数名称: InitDataPack
** 功能描述: 初始化数据包
** 输　  入: 无    
** 输　  出: 无
** 返    回: 无
** 全局变量: 无
** 调用模块: 无
********************************************************************************************************/
void InitDataPack(DataPacket_t *pack)
{
	pack->CodeVers              =0x0102;    /*表示单导联&表示系统数据消息*/   
    pack->Length                =0x7803;    /*表示包长度字节  918*/
	pack->Packetserialnum		=0x00000000;	/*序列号*/
	pack->userid				=0x00000000;	/*用户ID*/
	//pack->start_timer			=0x00000000;	/*开始时间*/
	pack->Crc                   =0x0000;    /*表示CRC校验—闲置*/
    pack->IdentPrtc             =0x2000;    /*表示预留区域版本V2.0*/
	

	pack->EcgData.Type          =0x1000;
	pack->EcgData.Length        =0xC400;    /*ECG数据96*2个字节+4字节=196个字节*/
	
//	pack->ResData.Type          =0x3100;
//	pack->ResData.Length        =0x2401;    /*RESP数据96*3个字节+4字节=288+4个字节*/
	
	pack->AccData.Type          =0x2000;
	pack->AccData.Length        =0xC400;    /*ACC数据32*3*2个字节+4字节=196个字节*/
			
	pack->GyrData.Type          =0x2100;
	pack->GyrData.Length        =0xC400;    /*Gyr数据32*3*2个字节+4字节=196个字节*/
 	
	pack->TemData.Type          =0x1B00;
	pack->TemData.Length        =0x0600;    /*体温数据2个字节+4字节=6个字节*/
  pack->TemData.Value         =0x2525;
	
	pack->BatData.Type          =0x3000;
	pack->BatData.Length        =0x0600;    /*电量数据2个字节+4字节=6个字节*/
  pack->BatData.Value         =0x4500;     
	/*于11月29日修改 根据数据协议修改数据类型*/
	pack->RefData.Type          =0x3200;
	pack->RefData.Length        =0x0600;    /*0mv基准数据2个字节+4字节=6个字节*/
  pack->RefData.Value         =0x0002; 
	
	/*于12月3日修改 根据数据协议修改数据类型*/
	pack->RefoData.Type          =0x3100;
	pack->RefoData.Length        =0x0600;    /*1mv基准数据2个字节+4字节=6个字节*/
//	pack->RefoData.Value         =0x2b00; 
	pack->RefoData.Value         =0xB101; 
//  pack->RefoData.Value         =0xa200; 

	pack->ErrData.Type          =0x4000;
	pack->ErrData.Length        =0x0600;    /*ERR数据2个字节+4字节=6个字节*/
	pack->ErrData.Value         =0x0000; 	/*630*/
	
	/*增加协议指令、心率、血氧、舒张压、收缩压、呼吸频率、R波位置、运动状态、步频*/
	pack->PPGData.Type			=0x4100;
	pack->PPGData.Length		=0xC400;	/*PPG数据96*2个字节+4字节=196个字节*/
	
	pack->HeartRateData.Type	= 0x4200;
	pack->HeartRateData.Length	= 0x0600;
	pack->HeartRateData.Value	= 0x0064;	/*心率数据2个字节+4字节=6个字节*/

	pack->BOData.Type			= 0x4300;
	pack->BOData.Length			= 0x0600;
	pack->BOData.Value			= 0x0062;	/*血氧数据2个字节+4字节=6个字节*/
	
	pack->DIATPData.Type		= 0x4400;
	pack->DIATPData.Length		= 0x0600;
	pack->DIATPData.Value		= 0x0078;	/*舒张压数据2个字节+4字节=6个字节*/

	pack->SYSTPData.Type		= 0x4500;
	pack->SYSTPData.Length		= 0x0600;
	pack->SYSTPData.Value		= 0x0046;	/*收缩压数据2个字节+4字节=6个字节*/

	pack->ResRateData.Type		= 0x4600;
	pack->ResRateData.Length	= 0x0600;
	pack->ResRateData.Value		= 0x003C;	/*呼吸频率数据2个字节+4字节=6个字节*/

	pack->RWarePosData.Type		= 0x4700;
	pack->RWarePosData.Length	= 0x0800;
	pack->RWarePosData.Value	= 0x12345678;	/*R波位置数据2个字节+4字节=6个字节*/

	pack->ExerciseStaData.Type		= 0x4800;
	pack->ExerciseStaData.Length	= 0x0600;
	pack->ExerciseStaData.Value		= 0x0002;	/*运动状态数据2个字节+4字节=6个字节*/

	pack->CadenceData.Type			= 0x4900;
	pack->CadenceData.Length		= 0x0600;
	pack->CadenceData.Value			= 0x001E;	/*步频数据2个字节+4字节=6个字节*/
												/*878*/
	pack->start_timer.Type			= 0x5000;
	pack->start_timer.Length		= 0x0800;	
	pack->start_timer.Value			= 0x00000000;
//	pack->fillData.Type					=0x7000;
//	pack->fillData.Length       =0x1000;		/*fill数据2个字节+2个字节=4个字节*/
    
}
 
/**********************************************************************************************
 * 描  述 : 使用notification将温湿度值、人体红外状态发送给主机
 * 参  数 : p_lbs[in]:指向thi服务结构体
 *        : dht11val[in]:DTH11温湿度传感器检测的温湿度值
 *        : infraredval[in]:人体红外状态
 * 返回值 : 成功返回NRF_SUCCESS，否则返回错误代码
 ***********************************************************************************************/ 
uint16_t ble_send_vol_data(ble_volmeass_t * p_volmeass, uint8_t *p_Volbuf, uint8_t count)
{
    ble_gatts_hvx_params_t params;
	 
    uint16_t len;
	  uint32_t err_code;
 
	  len = count;
		
	  if (p_volmeass->conn_handle != BLE_CONN_HANDLE_INVALID)
		{
			 memset(&params, 0, sizeof(params));
     		 params.type = BLE_GATT_HVX_NOTIFICATION;
     		 params.handle = p_volmeass->mpu_char_handles.value_handle;
			params.offset = 0;
      		params.p_data = p_Volbuf;
     		 params.p_len = &len;	 
      		return sd_ble_gatts_hvx(p_volmeass->conn_handle, &params);
		}
		else{
			err_code = NRF_ERROR_INVALID_STATE;
		}
    return err_code;
    
}

/********************************************END FILE*******************************************/

