#include "DATA_FILTER.h"
//extern int ecg_data_buf[302];
extern int EcgData[100],ecgbuf[300],dataLen,ecgDatalenp;
/*中值滤波*/
void medianFilter()
{
//	if(NULL  == EcgData)
//    {
//        return ;
//    }
//    int i = 0;
//    int medianValue = 0;
//	  int ecgDatalen = ecgDatalenp;
//   /*开窗*/
//			while(ecgDatalen < WIN_LEN && i< dataLen)
//    {
//        ecgbuf[ecgDatalen] = EcgData[i];
//        ecgDatalen++;
//        i++;
//    }
//    
//    if(ecgDatalen < WIN_LEN)
//    {
//        int  j=0;
//        int sum = 0;
//        for(j = 0;j<ecgDatalen;j++)
//        {
//            sum += ecgbuf[j];
//        }
//        medianValue = (sum/ecgDatalen+0.5);
//        
//        for(j = 0;j<dataLen;j++)
//				{
//            EcgData[j] = EcgData[j]+2000 - medianValue;
//        }
//        
//    }
//    else
//    {
//        int k = 0;
//        for(k = 0;k<dataLen;k++)
//        {
//            int  j=0;
//            int sum = 0;
//            for(j = 0;j<ecgDatalen-1;j++)   //移位
//            {
//                ecgbuf[j] = ecgbuf[j+1];
//                sum += ecgbuf[j];
//            }
//            ecgbuf[j] = EcgData[k];
//            sum += ecgbuf[j];
//            
//            medianValue = (sum/ecgDatalen+0.5);
//            
//            EcgData[k] = EcgData[k]+2000 - medianValue;
//        }
//        
//    }   
//		ecgDatalenp = ecgDatalen;
}



//void medianFilter(int *EcgData,int dataLen, int *ecgbuf,int ecgDatalenp)
//{
////	if(NULL  == EcgData)
////    {
////        return ;
////    }
//    int i = 0;
//    int medianValue = 0;
//	  int ecgDatalen = ecgDatalenp;
//   /*开窗*/
//    while(ecgDatalen < WIN_LEN && i< dataLen)
//    {
//        ecgbuf[ecgDatalen] = EcgData[i];
//        ecgDatalen++;
//        i++;
//    }
//    
//    if(ecgDatalen < WIN_LEN)
//    {
//        int  j=0;
//        int sum = 0;
////        for(j = 0;j<ecgDatalen;j++)
////        {
////            sum += ecg_data_buf[j];
////        }
//        medianValue = (sum/ecgDatalen+0.5);
//        
//        for(j = 0;j<dataLen;j++)
//				{
//            EcgData[j] = EcgData[j] - medianValue;
//        }
//        
//    }
//    else
//    {
//        int k = 0;
//        for(k = 0;k<dataLen;k++)
//        {
//            int  j=0;
//            int sum = 0;
//            for(j = 0;j<ecgDatalen-1;j++)   //移位
//            {
//                ecgbuf[j] = ecgbuf[j+1];
//                sum += ecgbuf[j];
//            }
//            ecgbuf[j] = EcgData[k];
//            sum += ecgbuf[j];
//            
//            medianValue = (sum/ecgDatalen+0.5);
//            
//            EcgData[k] = EcgData[k] - medianValue;
//        }
//        
//    }   
//		ecgDatalenp = ecgDatalen;
//}

void GetBaseLine(int *data,int dataLen,int * databuf,int *databuflenp)
{
//    if(NULL  == data)
//    {
//        return ;
//    }
    int i = 0;
    int medianValue = 0;
    int databuflen = *databuflenp;
   
    while(databuflen < WIN_LEN && i< dataLen)
    {
        databuf[databuflen] = data[i];
        databuflen++;
        i++;
    }
    
    if(databuflen < WIN_LEN)
    {
        int  j=0;
        double sum = 0;
        for(j = 0;j<databuflen;j++)
        {
            sum += databuf[j];
        }
        medianValue = (sum/databuflen+0.5);
        
        for(j = 0;j<dataLen;j++)
   
     {
            data[j] = medianValue;
        }
        
    }
    else
    {
        int k = 0;
        for(k = 0;k<dataLen;k++)
        {
            int  j=0;
            double sum = 0;
            for(j = 0;j<databuflen-1;j++)   //移位
            {
                databuf[j] = databuf[j+1];
                sum += databuf[j];
            }
            databuf[j] = data[k];
            sum += databuf[j];
            
            medianValue = (sum/databuflen+0.5);
            
            data[k] = medianValue;
        }
        
    }
		*databuflenp = databuflen ;
}

//  GetBaseLine(data,dataLen,m_respbuf,m_respDatalen);   
//    medianFilter(data,dataLen,m_baseDatabuf,m_baseDatalen); 
	
void VAL_TO_T(int *ecgbuff,uint8_t * ecgtl)
{
		uint8_t i;
		for(i = 0;i<96;i++)
		{
				ecgtl[2*i+1]= (ecgbuff[i]&0xff);
				ecgtl[2*i] = (ecgbuff[i]>>8)&0xff;
				
		}
	
}


