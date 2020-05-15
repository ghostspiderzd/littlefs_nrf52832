/******************************Copyright (c)***********************************
*                Jiangsu Zhihai Electronic Technology Co., Ltd.
*                      Research & Development Department
-------------------------------------------------------------------------------
* @file    BLE_SET.c
* @author  Gu Dongdong
* @date    2018-10-16  
*******************************************************************************/
#include "BLE_SET.h"
#include "nrf_drv_timer.h"
#include "ble_gap.h"

unsigned char  gCmdflag = 0;

extern void application_timers_start(void);
extern void application_timers_stop(void);
extern volatile uint8_t Open_flag;
static uint16_t                m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
ble_volmeass_t                 m_volmeass; 

data_struct gdata_struct;	//接收蓝牙数据结构体

static void gap_params_init(void);
static void advertising_init(void);
static void services_init(void);
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt);
static void conn_params_error_handler(uint32_t nrf_error);
static void conn_params_init(void);
static void advertising_start(void);
static void on_ble_evt(ble_evt_t * p_ble_evt);
static void ble_evt_dispatch(ble_evt_t * p_ble_evt);
static void ble_stack_init(void);

extern adc_value_Big_buf_t                         adc_value_Big_buf;//双缓冲发送
extern adc_value_Big_buf_t                         adc_value_Res_buf;//双缓冲发送
extern uint8_t IMU_TICK,SonPackCon;
extern uint32_t saadc_callback_Count;
extern nrf_drv_timer_t TIMER_ADC;
extern uint32_t ADC_TICK;
extern uint16_t start_cont;
extern uint8_t data_first_flag,t_flag;
extern int ecg_new_cont;
extern uint16_t cont_ble,con_dely;

HOSTCMD_ENUM gHostcmd;
TLVdata_struct gTLCdata;
controlcmd_data_strict gControlcmd;

extern nrf_drv_timer_t TIMER_SPI;


//extern double sum;
/**@brief Function for CM18_BLE_SERVICES_init.
 */
void CM18_BLE_SERVICES_init(void)
{
	ble_stack_init();
	/*于7月15日修改保证定时器时钟源稳定*/
	sd_clock_hfclk_request();
	gap_params_init();
	services_init();
	advertising_init();
	conn_params_init();
	//?a?ú3é1|oó??DD1?2￥
	while(0 == Open_flag);
	advertising_start();
}
/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and doefifoSz = 0;s not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}
 
/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

	ble_gap_addr_t bleaddr;	//mac鍦板潃

	ble_advdata_manuf_data_t manuf_specific_data;
	//static unsigned char specific_data[6] = {0x11,0x22,0x33,0x44,0x55,0x66};

    int8_t tx_power_level = 4;
    ble_uuid_t adv_uuids[] = {{VOLMEASS_UUID_SERVICE, m_volmeass.uuid_type}};

	err_code = sd_ble_gap_addr_get(&bleaddr);
	manuf_specific_data.company_identifier = 0x0059;
	manuf_specific_data.data.size = 6;
	manuf_specific_data.data.p_data = bleaddr.addr;

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
	//  advdata.p_tx_power_level = &tx_power_level;
	
	

	//
	advdata.p_manuf_specific_data = &manuf_specific_data;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = adv_uuids;


    err_code = ble_advdata_set(&advdata, &scanrsp);
    APP_ERROR_CHECK(err_code);
}

void cmd_data(void)
{
	 data_struct *pdata = &gdata_struct;
	 volatile unsigned short temlen = 0;
	 if(pdata->data[0] == PROTOCOL_VERSION)
	 {
		if(pdata->data[1] == DATAPACKET_TYPE)
		{
			temlen = (pdata->data[2] << 8) | pdata->data[3];
			TLVdata_parsing(pdata->data + FRAME_HEADER_LEN, temlen - FRAME_HEADER_LEN);
		}
	 }	 
}


extern uint8_t guserdataflag;
extern uint8_t usertxbuff[20];
extern uint8_t guserconfig;

void TLVdata_parsing(unsigned char *pTLVdatapacket,unsigned short packetlen)
{
	controlcmd_data_strict *pControlcmd =  &gControlcmd;
	data_struct *pdata = &gdata_struct;
	volatile unsigned short len = 0;
	unsigned short type;
	do{
		if((pTLVdatapacket != NULL) && (packetlen > 4))
		{
			type = (pTLVdatapacket[0] << 8) | (pTLVdatapacket[1]);
			len = (pTLVdatapacket[2] << 8) | (pTLVdatapacket[3]);
			switch(type)
			{
				case HOST_CMD_START:
						if(len != 0x06) return ;
						if((pTLVdatapacket[4] << 8) | (pTLVdatapacket[5]) == PARAMETER_START)
						{
							//if(pControlcmd->user_id != 0)
							{
									gCmdflag = 1;	//开始
									//nrf_drv_timer_enable(&TIMER_ADC);
									SonPackSend_timers_start();
									nrf_drv_timer_enable(&TIMER_SPI);
							}
							
							//Packing_timers_start();
						}
				break;
				case HOST_CMD_STOP:
					if(len != 0x06) return ;
						if((pTLVdatapacket[4] << 8) | (pTLVdatapacket[5]) == PARAMETER_START)
						{
							gCmdflag = 0;	//停止
							
							//nrf_drv_timer_disable(&TIMER_ADC);
		            		SonPackSend_timers_stop();
							nrf_drv_timer_disable(&TIMER_SPI);
						}
				break;
				case HOST_CMD_HUMPER:
						if(len != 0x09) return ;
						//pControlcmd->human_par.height = pTLVdatapacket[4];
						//pControlcmd->human_par.weight = pTLVdatapacket[5];
						//pControlcmd->human_par.arterial_length = pTLVdatapacket[6];
						//pControlcmd->human_par.gender = pTLVdatapacket[7];
						ble_send_vol_data(&m_volmeass, pdata->data , pdata->len);
						
						usertxbuff[4] = pTLVdatapacket[4];
						usertxbuff[5] = pTLVdatapacket[5];
						usertxbuff[6] = pTLVdatapacket[6];
						usertxbuff[7] = pTLVdatapacket[7];
						usertxbuff[8] = pTLVdatapacket[8];
						guserdataflag++;
				break;
				case HOST_CMD_USERID:
						if(len != 0x08) return ;
						pControlcmd->user_id = (pTLVdatapacket[4] << 24) | (pTLVdatapacket[5] << 16)|(pTLVdatapacket[6] << 8)|(pTLVdatapacket[7]);
						//ble_send_vol_data(&m_volmeass, &pControlcmd->user_id , sizeof(pControlcmd->user_id));
				break;
				case HOST_CMD_CLOCK:
						if(len != 0x08) return;
						pControlcmd->clock = (pTLVdatapacket[4] << 24) | (pTLVdatapacket[5] << 16)|(pTLVdatapacket[6] << 8)|(pTLVdatapacket[7]);
						//ble_send_vol_data(&m_volmeass, pdata->data , pdata->len);
				break;
				case HOST_CMD_REF_BP:
						if(len != 0x08) return;
						//pControlcmd->ref_bp =(pTLVdatapacket[4] << 8) | (pTLVdatapacket[5]);
						//pControlcmd->ref_bp = (pTLVdatapacket[6] << 8) | (pTLVdatapacket[7]);
						guserdataflag++;
						ble_send_vol_data(&m_volmeass, pdata->data , pdata->len);
						usertxbuff[9] = pTLVdatapacket[4];
						usertxbuff[10] = pTLVdatapacket[5];
						usertxbuff[11] = pTLVdatapacket[6];
						usertxbuff[12] = pTLVdatapacket[7];
				default :
				break;
			}
			pTLVdatapacket += len;
			packetlen -=len;
		}
	}while(packetlen > 0);
}




/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_volmeass_init_t init;
    
    err_code = ble_volmeass_init(&m_volmeass, &init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;

    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
		Led_Display(BLE_LED_ADVERTISING);
}


/**@brief Function for handling the Application's BLE stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected\r\n");
            Led_Display(BLE_LED_CONNECTED);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
						/*DT??óú11-12￡?à??à???aá??óoó?á1?±??a??èí?t?¨ê±?÷￡?Dèòa?úá??ó×′ì???ê1?ü?a???¨ê±?÷*/
						/*于11月20日修改，重新连接后各状态位复位*/
						//adc_value_Big_buf.Byte_cnt = 0;
						//adc_value_Res_buf.Byte_cnt = 0;
						//IMU_TICK = 0;
						//ADC_TICK = 0;
						//SonPackCon = 0;
						//saadc_callback_Count=0;
						//ClrSystemFlag(Send_1ofPack_FLAG);
						//ClrSystemFlag(SendSonPackFlag);
						//ClrSystemFlag (ADCBuf_HT_Flag);
						//ClrSystemFlag (ADCBuf_TC_Flag);
						/*于11月26日修改添加哪个缓冲发包标志位*/
						//ClrSystemFlag(Send_2ofPack_FLAG);
						//ClrSystemFlag (Send_3ofPack_FLAG);
						/*修改张迪迪
							nrf_drv_timer_enable(&TIMER_ADC);
							SonPackSend_timers_start();
							Packing_timers_start();
						*/
				    		//PACK_NUM=0;
//						ecgDatalenp = 0;
//						sum = 0;
						//data_first_flag =0;
						//start_cont =0;
						//con_dely = 0;
						//t_flag = 0;
//            err_code = app_button_enable();
//            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected\r\n");
				
//            bsp_board_led_off(CONNECTED_LED_PIN);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            
				
//						application_timers_stop();
						/*修改张迪迪?*/
						//nrf_drv_timer_disable(&TIMER_ADC);
            			SonPackSend_timers_stop();
						nrf_drv_timer_disable(&TIMER_SPI);
						
            			advertising_start();
				
						start_cont =0;
						con_dely = 0;
						t_flag = 0;
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

//					case BLE_GATTS_EVT_WRITE:
//                nrf_gpio_pin_set(LED_3);
//            APP_ERROR_CHECK(err_code);
//            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
										
        default:
            // No implementation needed.
            break;		
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_volmeass_on_ble_evt(&m_volmeass, p_ble_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


