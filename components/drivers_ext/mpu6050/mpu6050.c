/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "twi_master.h"
#include "mpu6050.h"

/*��11��19���޸ģ����У׼ֵ*/
#define MPU6050_ADDRESS         0x69
#define MPU6050_GYRO_OUT        0x43
#define MPU6050_OUT         		0x3B
#define MPU6050_X_GRO_OFFSET    (-1)
#define MPU6050_Y_GRO_OFFSET    (1)
#define MPU6050_Z_GRO_OFFSET    (1)
#define MPU6050_X_ACC_OFFSET    (-1)
#define MPU6050_Y_ACC_OFFSET    (0)
#define MPU6050_Z_ACC_OFFSET    (1)
/*lint ++flb "Enter library region" */

#define ADDRESS_WHO_AM_I          (0x75U) // !< WHO_AM_I register identifies the device. Expected value is 0x68.
#define ADDRESS_SIGNAL_PATH_RESET (0x68U) // !<
/*��5��17���޸ģ���ԭ6������һ�ֽڸ�Ϊ�����ֽ�*/
#define _RAW_TO_TLV(x) ((((int)(x))*50)/16384)

#define _CH16(x)      ((unsigned short)(((x>>8)&0x00ff)|((x<<8)&0xff00)))
static const unsigned char expected_who_am_i = 0x68U; // !< Expected value to get from WHO_AM_I register.
static unsigned char       m_device_address;          // !< Device address in bits [7:1]


bool mpu6050_init(unsigned char device_address)
{   
  bool transfer_succeeded = true;
	
	unsigned char inData[8]={0x00,									//0x19
								0x00,												//0x1A
								0x03,												//0x6B
								0x11,												//0x1B
								0x00,												//0x6A
								0x32,												//0x37
								0x01,                       //0x38
	              0x00 };											//0x1C
 					
								
  m_device_address = (unsigned char)(device_address << 1);
	mpu6050_register_write(0x6b , 0x00);
	mpu6050_register_write(0x6b , 0x01);
	mpu6050_register_write(0x6c , 0x00);
	mpu6050_register_write(0x19 , 0x04);
	mpu6050_register_write(0x1a , 0x04);
	mpu6050_register_write(0x1b , 0x18);
	mpu6050_register_write(0x1c , 0x00);	
  // Do a reset on signal paths
//  unsigned char reset_value = 0x04U | 0x02U | 0x01U; // Resets gyro, accelerometer and temperature sensor signal paths
//  transfer_succeeded &= mpu6050_register_write(ADDRESS_SIGNAL_PATH_RESET, reset_value);
//  
//	
//	//����GYRO+
//	mpu6050_register_write(0x19 , 4);
//	mpu6050_register_write(0x1a , 4);
////	mpu6050_register_write(0x19 , inData[0]); //���ò�����    -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
////	mpu6050_register_write(0x1A , inData[1]); //CONFIG        -- EXT_SYNC_SET 0 (���þ��������) ; default DLPF_CFG = 0 => (��ͨ�˲�)ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
//	mpu6050_register_write(0x6B , inData[2]); //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
//	mpu6050_register_write(0x1B , inData[3]); //gyro���� ����  ����2000��ÿ��
//	mpu6050_register_write(0x6A , inData[4]);	// 0x6A�� I2C_MST_EN  ���ó�0  Ĭ��Ϊ0 6050  ʹ����IIC 	
//	// 0x37�� ����������ߵ�ƽ�жϣ�һֱ����ߵ�ƽֱ���ж�������κζ�ȡ����������ж� ʹ�� pass through ���� ֱ����6050 ��ȡ5883����
//	mpu6050_register_write(0x37,  inData[5]);	
//	mpu6050_register_write(0x38,  inData[6]); //ʹ��data ready ����		
//	//���� ACC
//  mpu6050_register_write(0x1C,inData[7]);         //ACC����  ���� +-2G s 									
  // Read and verify product ID
  transfer_succeeded &= mpu6050_verify_product_id();

  return transfer_succeeded;
}

bool mpu6050_verify_product_id(void)
{
    unsigned char who_am_i;

    if (mpu6050_register_read(ADDRESS_WHO_AM_I, &who_am_i, 1))
    {
        if (who_am_i != expected_who_am_i)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        return false;
    }
}

bool mpu6050_register_write(unsigned char register_address, unsigned char value)
{
    unsigned char w2_data[2];

    w2_data[0] = register_address;
    w2_data[1] = value;
    return twi_master_transfer(m_device_address, w2_data, 2, TWI_ISSUE_STOP);
}

bool mpu6050_register_read(unsigned char register_address, unsigned char * destination, unsigned char number_of_bytes)
{
    bool transfer_succeeded;
    transfer_succeeded  = twi_master_transfer(m_device_address, &register_address, 1, TWI_DONT_ISSUE_STOP);
    transfer_succeeded &= twi_master_transfer(m_device_address|TWI_READ_BIT, destination, number_of_bytes, TWI_ISSUE_STOP);
    return transfer_succeeded;
}

//void MPU6050_ReadAccGyr(char *pX , char *pY , char *pZ,eAccGyrFlags eStatus)
//{
//  unsigned char buf[6];    			
//	if (eStatus == eGYR )
//	{
//		mpu6050_register_read(MPU6050_GYRO_OUT,  buf, 6);
//	}else{
//		mpu6050_register_read(MPU6050_OUT,  buf, 6);
//	}

//  *pX = ((buf[0] << 8) | buf[1])>>4;
//	if(*pX & 0x80) *pX-=256;
//		
//	*pY= ((buf[2] << 8) | buf[3])>>4;
//  if(*pY & 0x80) *pY-=256;
//	
//  *pZ = ((buf[4] << 8) | buf[5])>>4;
//	if(*pZ & 0x80) *pZ-=256;
//}
//			   
void MPU6050_ReadGyr(short *pX , short *pY , short *pZ )
{
  unsigned char tmp[6];  	
  short raw[3];
  mpu6050_register_read(MPU6050_GYRO_OUT,  tmp, 6);
	raw[0] = (short)(tmp[0] << 8 | tmp[1]);
	raw[1] = (short)(tmp[2] << 8 | tmp[3]);
	raw[2] = (short)(tmp[4] << 8 | tmp[5]);
//  *pX = 0-(char) buf[0]+MPU6050_X_GRO_OFFSET;
//  *pY = (char) buf[2]+MPU6050_Y_GRO_OFFSET;
//	*pZ = (char) buf[4]+MPU6050_Z_GRO_OFFSET;
	
	*pX = _CH16(raw[0]);
  *pY = _CH16(raw[1]);
	*pZ = _CH16(raw[2]);
	
	/*��2019-4-09�޸���������mpu6050x���z�᷽����CM18�෴*/
//	*pX = (char) buf[0]+MPU6050_X_GRO_OFFSET;
//  *pY = (char) buf[2]+MPU6050_Y_GRO_OFFSET;
//	*pZ = 0-(char) buf[4]+MPU6050_Z_GRO_OFFSET;
}
			   

void MPU6050_ReadAcc( short *pX , short  *pY , short *pZ )
{
    
	unsigned char tmp[6];  	
  short raw[3];
	mpu6050_register_read(MPU6050_OUT, tmp, 6);
	raw[0] = (short)(tmp[0] << 8 | tmp[1]);
	raw[1] = (short)(tmp[2] << 8 | tmp[3]);
	raw[2] = (short)(tmp[4] << 8 | tmp[5]);
//	*pX = _CH16((short)(32678+raw[0]));
//  *pY = _CH16((short)(32678+raw[1]));
//	*pZ = _CH16((short)(32678+raw[2]));	
	
	*pX = _CH16((short)(128-_RAW_TO_TLV(raw[0])));
  *pY = _CH16((short)(128-_RAW_TO_TLV(raw[1])));
	*pZ = _CH16((short)(128+_RAW_TO_TLV(raw[2])));
	
//  *pX = (unsigned char)((128-(char) buf[0])+MPU6050_X_ACC_OFFSET);
//  *pY = (unsigned char)((128+(char) buf[2])+MPU6050_Y_ACC_OFFSET);
//	*pZ = (unsigned char)((128+(char) buf[4])+MPU6050_Z_ACC_OFFSET);
	
//	*pX = (unsigned char)((128+buf[0]*50/64));
//  *pY = (unsigned char)((128-buf[2]*50/64));
//	*pZ = (unsigned char)((128-buf[4]*50/64));
	/*��2019-4-09�޸���������mpu6050x���z�᷽����CM18�෴*/
//	*pX = (unsigned char)((128+(char) buf[0])+MPU6050_X_ACC_OFFSET);
//  *pY = (unsigned char)((128+(char) buf[2])+MPU6050_Y_ACC_OFFSET);
//	*pZ = (unsigned char)((128-(char) buf[4])+MPU6050_Z_ACC_OFFSET);
 
}
 


/*lint --flb "Leave library region" */
