#ifndef DATA_FILTER_H
#define DATA_FILTER_H
#include "stdint.h"

#define WIN_LEN 300
void medianFilter();
//void medianFilter(int *EcgData,int dataLen, int *ecgbuf,int ecgDatalenp);
void GetBaseLine(int *data,int dataLen,int * databuf,int *databuflenp);
void VAL_TO_T(int *ecgbuff,uint8_t * ecgtl);
#endif
