/*******************************************************************************
-------------------------------------------------------------------------------
* @file    BLE_CRC.C
* @author  Ye Junjie
* @date    2018-11-16  
*******************************************************************************/
#include "BLE_CRC.h"

typedef unsigned char width_t;  
#define WIDTH (8 * sizeof(width_t))  
#define TOPBIT (1 << (WIDTH - 1)) 
#define POLYNOMIAL          0x31 
/* 
* An array containing the pre-computed intermediate result for each 
* possible byte of input. This is used to speed up the computation. 
*/
//static width_t crcTable[256]; 

//void crcInit(void)  
//{  
//    int remainder;  
//    int dividend;  
//    int bit;  
//    /* Perform binary long division, a bit at a time. */  
//    for(dividend = 0; dividend < 256; dividend++)  
//    {  
//        /* Initialize the remainder.  */  
//        remainder = dividend;  
//        /* Shift and XOR with the polynomial.   */  
//        for(bit = 0; bit < 8; bit++)  
//        {  
//            /* Try to divide the current data bit.  */  
//            if(remainder & TOPBIT)  
//            {  
//                remainder = (remainder << 1) ^ POLYNOMIAL;  
//            }  
//            else  
//            {  
//                remainder = remainder << 1;  
//            }  
//        }  
//        /* Save the result in the table. */  
//        crcTable[dividend] = remainder;  
//    }  
//} /* crcInit() */
//unsigned char crctes[440];
unsigned char crc8(unsigned char *addr, int num, unsigned char crc)  
{  
    int i;  
    for (; num > 0; num--)              /* Step through bytes in memory */  
    {  
//				crctes[num-1]=*addr;
        crc =crcTable[crc ^ (*addr++)];     /* Fetch byte from memory, XOR into CRC top byte*/         
    }                               /* Loop until num=0 */  
    return(crc);                    /* Return updated CRC */  
} 