#ifndef BLE_LBS_H__
#define BLE_LBS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "BLE_SET.h"

#define VOLMEASUUID_BASE        {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, \
                                 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}
#define VOLMEASS_UUID_SERVICE     0x1526
#define VOLMEASS_UUID_VOL_CHAR    0x1527
#define VOLMEASS_UUID_VOL_CHAR_2    0x1528
															
															
															
/*******************************************************************************************************
** 功能描述: 系统标志操作宏定义
********************************************************************************************************/
#define GetSystemFlag(flag)        SystemFlag & (flag)
#define SetSystemFlag(flag)        SystemFlag |= (flag)
#define ClrSystemFlag(flag)        SystemFlag &= ~(flag)
#define ClrAllSystemFlag()         SystemFlag = 0x00000000
															
extern  unsigned int  SystemFlag;	
											
/*******************************************************************************************************
** 结构名称: AllSystemFlag
** 功能描述: 系统所有标志位 
********************************************************************************************************/
typedef enum 
{
	NoFlag							=0,
  /*No标志*/
  OpenDevFlag         =(1<<0), 
  /*开机标志*/
  BTLinkFlag          =(1<<1),
  /*蓝牙连接标志*/
	StartADCFlag        =(1<<2),
  /*ADC操作标志*/
	CloseDevFlag        =(1<<3),
  /*关机操作标志*/
	AllowCloseDevFlag   =(1<<4),
  /*关机操作标志*/
	ADCBuf_HT_Flag      =(1<<5),
	/*DMA1半溢标志*/
	ADCBuf_TC_Flag      =(1<<6),
	/*DMA1全溢标志*/
	CollectIMUFlag      =(1<<7),
	/*采集IMU数据标志*/
	IMUBuf_TC_Flag      =(1<<8),
	/*采集IMU数据标志*/
	IMUBuf_HT_Flag      =(1<<9),
	/*IMU数据池溢出标志*/
	IMUErrFlag          =(1<<10),
	/*发送分包标志*/
	SendSonPackFlag     =(1<<11),
	/*开始采集采集运动数据*/
	DisWaitFlag         =(1<<12),
 /*显示等待标志*/
	DisWorkFlag         =(1<<13),
 /*显示工作标志*/
 	DMA1_Flag_Ht4       =(1<<14),
	/*DMA1半溢标志*/
	DMA1_Flag_Tc4       =(1<<15),
	/*DMA1全溢标志*/
	CloseBuzzerFlag     =(1<<16),
	/*USB口插入充电标志*/
	USBInsert_Flag      =(1<<17),
	/*USB口未插入充电标志*/
	USBNotInsert_Flag   =(1<<18),
	/*USB口未插入充电标志*/
	TIM2_SendPack_FLAG  =(1<<19),
	Send_1ofPack_FLAG   =(1<<20),
	Send_2ofPack_FLAG   =(1<<21),
	Send_3ofPack_FLAG   = (1<<22),
	Read_Temp_FLAG      = (1<<23),
	Read_Bat_FLAG      	= (1<<24),
	Connect_Err_FLAG    = (1<<25),
	
}AllSystemFlags;															

/*******************************************************************************************************
** 功能描述: IO口操作宏定义
********************************************************************************************************/
#define  ECG_RESOLUTION         12u  //12bit or 8bit ECG 信号分辨率


#define  AxisNum                6u
#define  EcgDataNum             192u
#define  ResDataNum             288u
#define  MotDataNum             32u
#define  PPGDataNum				192u

/*******************************************************************************************************
** 结构名称: 数据包
** 功能描述:  
********************************************************************************************************/
typedef struct
{
	unsigned short	     Type;
	unsigned short 	     Length;	
	unsigned char 	     L1Value[EcgDataNum];	
}EcgData_t;		

typedef struct
{
	unsigned short	     Type;
	unsigned short 	     Length;	
	unsigned char 	     Value[PPGDataNum];	
}PPGData_t;		


typedef struct
{
	unsigned short	     Type;
	unsigned short 	     Length;	
	unsigned char 	     L1Value[ResDataNum];	
}ResData_t;		
typedef struct
{
	unsigned short       Type;
	unsigned short 	     Length;	
	         short 	     xValue[MotDataNum];	
	         short 	     yValue[MotDataNum];
	         short 	     zValue[MotDataNum];
}MotData_t;	

typedef struct
{
 short 	accx;	
 short 	accy;
 short	accz;
 short 	gyrx;	
 short 	gyry;
 short 	gyrz;
}accgyr_t;	

typedef struct
{
	unsigned short       Type;
	unsigned short	     Length;
	unsigned short	     Value;		
}OtherData_t;	

typedef struct
{
	unsigned short       Type;
	unsigned short	     Length;
	unsigned int	     Value;		
}OtherDataint_t;	


typedef struct
{
	unsigned short       Type;
	unsigned short	     Length;	
	char                 fill[14];
}fillData_t;	

typedef struct DataPackStrt
{
  	unsigned short	     CodeVers;
	unsigned short	     Length;
	unsigned int		 Packetserialnum;	//包序列号
	unsigned int		 userid;			//用户ID

  	unsigned short	     Crc;
  	unsigned short	     IdentPrtc;		//16
	EcgData_t            EcgData;		//196
//	ResData_t            ResData;
	MotData_t            AccData;		//196
	MotData_t            GyrData;		//196
	OtherData_t          BatData;		//8
	OtherData_t          RefData;		//8
	/*于12月3日添加1mv标志数据*/
	OtherData_t          RefoData;		//8
	OtherData_t          TemData;		//8
	OtherData_t          ErrData;		//8

	/*增加协议指令PPG、心率、血氧、舒张压、收缩压、呼吸频率、R波位置、运动状态、步频*/
	PPGData_t			 PPGData;		//196
	OtherData_t          HeartRateData;	//6
	OtherData_t          BOData;		//6
	OtherData_t          DIATPData;		//6
	OtherData_t          SYSTPData;		//6
	OtherData_t          ResRateData;	//6
	OtherDataint_t       RWarePosData;	//8
	OtherDataint_t		 start_timer;		//开始时间//8
	OtherData_t          ExerciseStaData;	//6
	OtherData_t          CadenceData;		//6

//	fillData_t					 fillData;
}DataPacket_t;



// Forward declaration of the ble_lbs_t type. 
typedef struct ble_volmeass_s ble_volmeass_t;

typedef void (*ble_lbs_led_write_handler_t) (ble_volmeass_t * p_lbs, uint8_t new_state);

typedef void (*data_handle_t)(void);

typedef struct
{
    ble_lbs_led_write_handler_t led_write_handler; /**< Event handler to be called when the LED Characteristic is written. */
} ble_volmeass_init_t;

/**@brief mpu6050 Service structure. This structure contains various status information for the service. */
struct ble_volmeass_s
{
    uint16_t                    service_handle;      /**< Handle of Mpu6050 Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    led_char_handles;    /**< Handles related to the LED Characteristic. */
    ble_gatts_char_handles_t    mpu_char_handles;    /**< Handles related to the mpu Characteristic. */
	  uint8_t                     uuid_type;           /**< UUID type for the LED Button Service. */
    uint16_t                    conn_handle;         /**< Handle of the current connection (as provided by the BLE stack). BLE_CONN_HANDLE_INVALID if not in a connection. */
    ble_lbs_led_write_handler_t led_write_handler;   /**< Event handler to be called when the LED Characteristic is written. */
    data_handle_t data_handler; 
};															

typedef struct
{
    uint8_t cnt;
	  uint8_t buf[20]; 
}adc_value_t;



typedef struct
{
	  uint32_t Byte_cnt;
    uint32_t *p1;
	  uint32_t *p2;
	  uint8_t	L1Value[EcgDataNum*2];	
}adc_value_Big_buf_t;
typedef struct
{
	  uint32_t Byte_cnt;
    uint32_t *p1;
	  uint32_t *p2;
	  uint8_t	L1Value[EcgDataNum*3];	
}adc_value_three_buf_t;
typedef struct
{
	  uint32_t Byte_cnt;
    uint32_t *p1;
	  uint32_t *p2;
		accgyr_t Value[MotDataNum*2];	
}imu_value_Big_buf_t;

 
uint32_t ble_volmeass_init(ble_volmeass_t * p_mpus, const ble_volmeass_init_t * p_mpus_init);
uint16_t ble_send_vol_data(ble_volmeass_t * p_volmeass, uint8_t *p_Volbuf, uint8_t count);
void ble_volmeass_on_ble_evt(ble_volmeass_t * p_lbs, ble_evt_t * p_ble_evt);
static uint32_t volmeas_char_add_2(ble_volmeass_t * p_lbs, const ble_volmeass_init_t * p_volmeass_init);

 

extern void InitDataPack(DataPacket_t *pack);//初始化数据包


#endif
