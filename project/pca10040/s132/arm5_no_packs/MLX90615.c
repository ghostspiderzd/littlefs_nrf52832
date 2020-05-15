#include <stdbool.h>
#include <stdint.h>
#include "twi_master.h"
#include "mlx90615.h"
/*读出数值处理*/
float CalcTemp(unsigned int value)
{
	float temp;
	
	temp=(value*0.02)-273.15;
	
	return temp;
}
/*读取温度值*/
float mlx90615_read(void)
{
	unsigned char 	SlaveAddress; 			//Contains device address
	unsigned char	command;	  			//Contains the access command
	unsigned int 	data_temp;		  			//Contains data value	
	float			t;						//Contains calculated temperature in degrees Celsius	
		
	SlaveAddress=SA<<1;						//Set device address  
	command=RAM_Access|RAM_Tobj1; 	        //Form RAM access command + RAM address 			
	
	
////	data_temp=MemRead(SlaveAddress,command); //Read memory
//	t=CalcTemp(data_temp);					//Calculate temperature
//	delay_mlx(DEL1SEC);						//Wait 1 second 


	return t;


}

bool mlx90615_register_read(unsigned char register_address, unsigned char * destination, unsigned char number_of_bytes)
{
    bool transfer_succeeded;
    transfer_succeeded  = twi_mlx90615_transfer(0xb6, &register_address, 1, TWI_DONT_ISSUE_STOP);
    transfer_succeeded &= twi_mlx90615_transfer(0xb6|TWI_READ_BIT, destination, number_of_bytes, TWI_ISSUE_STOP);
    return transfer_succeeded;
}
bool mlx90615_register_write(unsigned char register_address, unsigned char value)
{
    unsigned char w2_data[2];

    w2_data[0] = register_address;
    w2_data[1] = value;
    return twi_mlx90615_transfer(0xb6, w2_data, 2, TWI_ISSUE_STOP);
}
/*CRC校验*/
//*********************************************************************************************
//									CALCULATION PEC PACKET	
//*********************************************************************************************
//Name:			PEC_calculation
//Function:		Calculates the PEC of received bytes
//Parameters:	unsigned char pec[]			
//Return:		pec[0]-this byte contains calculated crc value
//Comments: 	Refer to "System Managment BUS(SMBus) specification Version 2.0" and
//				AN "SMBus comunication with MLX90614"
//*********************************************************************************************
//unsigned char PEC_calculation(unsigned char pec[])
//{
//	unsigned char 	crc[6];
//	unsigned char	BitPosition=47;
//	unsigned char	shift;
//	unsigned char	i;
//	unsigned char	j;
//	unsigned char	temp;

//	do{
//		crc[5]=0;				/* Load CRC value 0x000000000107 */
//		crc[4]=0;
//		crc[3]=0;
//		crc[2]=0;
//		crc[1]=0x01;
//		crc[0]=0x07;
//		BitPosition=47;			/* Set maximum bit position at 47 */
//		shift=0;
//				
//		//Find first 1 in the transmited message
//		i=5;					/* Set highest index */
//		j=0;
//		while((pec[i]&(0x80>>j))==0 && i>0){
//			BitPosition--;
//			if(j<7){
//				j++;
//			}
//			else{
//				j=0x00;
//				i--;
//			}
//		}/*End of while */
//		
//		shift=BitPosition-8;	/*Get shift value for crc value*/
//		
//		
//		//Shift crc value 
//		while(shift){
//			for(i=5; i<0xFF; i--){
//				if((crc[i-1]&0x80) && (i>0)){
//					temp=1;
//				}
//				else{
//					temp=0;
//				}
//				crc[i]<<=1;
//				crc[i]+=temp;
//			}/*End of for*/
//			shift--;
//		}/*End of while*/
//		
//		
//		//Exclusive OR between pec and crc		
//		for(i=0; i<=5; i++){
//			pec[i] ^=crc[i];
//		}/*End of for*/
//	}while(BitPosition>8);/*End of do-while*/
//	
//	return pec[0];
//}/*End of PEC_calculation*/

