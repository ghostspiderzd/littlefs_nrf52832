#ifndef _FS_DRIVER_H
#define _FS_DRIVER_H


#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"

#define FILE_NAME   "NORDIC.TXT"
#define TEST_STRING "SD card example.\r\n"

























//static void fatfs_example();
void fatfs_test(void);



#endif