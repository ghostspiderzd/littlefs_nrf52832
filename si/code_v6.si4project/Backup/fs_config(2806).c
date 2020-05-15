
#include "Flash_driver.h"
#include "fs_config.h"
#include "lfs.h"
#include "lfs_util.h"



lfs_t g_lfs;
lfs_file_t file;
uint8_t        lfs_read_buf[256] = {0};
uint8_t        lfs_prog_buf[256] = {0};
uint8_t        lfs_lookahead_buf[256] = {0};
uint8_t        lfs_file_buf[256] = {0};
uint32_t lfs_free_spcae_size = 0;


int user_provided_block_device_read(const struct lfs_config *c, lfs_block_t block,
                lfs_off_t off, void *buffer, lfs_size_t size) {

       // ASSERT(block < c->block_count);
        spi_flash_read_data_52832((uint8_t *)buffer, (block * c->block_size + off), size);
        return 0;
}

int user_provided_block_device_prog(const struct lfs_config *c, lfs_block_t block,
                lfs_off_t off, const void *buffer, lfs_size_t size) {

       // ASSERT(block < c->block_count);
        spi_flash_Write_NoCheck((uint8_t *)buffer, (block * c->block_size + off), size);
        return 0;
}

int user_provided_block_device_erase(const struct lfs_config *c, lfs_block_t block) {
       // ASSERT(block < c->block_count);
        spi_flash_erase_addr(block * c->block_size);
        return 0;
}

int user_provided_block_device_sync(const struct lfs_config *c) {
        return 0;
}

// configuration of the filesystem is provided by this struct
const struct lfs_config cfg = {

	.context = NULL,
    // block device operations
    .read  = user_provided_block_device_read,
    .prog  = user_provided_block_device_prog,
    .erase = user_provided_block_device_erase,
    .sync  = user_provided_block_device_sync,

    // block device configuration
      .read_size       = 256,
  .prog_size       = 256,
  .block_size      = 4096,
  .block_count     = 4096, // The block count is actually 4096, but I lowered it to test. (Failed.)
  .cache_size      = 256,
  .lookahead_size  = 256,
  .block_cycles    = 200,

  .read_buffer      = (void*)lfs_read_buf,
  .prog_buffer      = (void*)lfs_prog_buf,
  .lookahead_buffer = (void*)lfs_lookahead_buf,
    
	//.read_buffer = lfs_read_buf,
	//.prog_buffer = lfs_prog_buf,
	//.lookahead_buffer = lfs_lookahead_buf,
	//.file_max = 256,
};


void spi_flash_littlefs_init(void) {
        // mount the filesystem
        int err = lfs_mount(&g_lfs, &cfg);

    // reformat if we can't mount the filesystem
    // this should only happen on the first boot
    if(err) {
               // spi_flash_erase_chip();
        err = lfs_format(&g_lfs, &cfg);
        err = lfs_mount(&g_lfs, &cfg);
    }

       // NRF_LOG_INFO("spi flash littlefs done");
}

void spi_flash_littlefs_test(void) {

    // read current count
    int i;
    uint32_t buf_len = 0;
        uint8_t test_buf[1024] = {'\0'};
		uint32_t boot_count = 0;
		volatile int err = 0;

    	spi_flash_littlefs_init();
	/*
        //lfs_file_open(&g_lfs, &file, "boot_count",  LFS_O_WRONLY | LFS_O_TRUNC);
        //NRF_LOG_INFO("lfs_file_size: %d", lfs_file_size(&g_lfs, &file));
        //lfs_file_close(&g_lfs, &file);

        lfs_file_open(&g_lfs, &file, "boot_count",  LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND);
        for (i = 0; i < 1024; i++) {
                lfs_file_write(&g_lfs, &file, (const void*)"0123456789", strlen("0123456789"));
        }
        lfs_file_close(&g_lfs, &file);

        lfs_file_open(&g_lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
        buf_len = lfs_file_size(&g_lfs, &file);
        NRF_LOG_INFO("lfs_file_size: %d", buf_len);
        lfs_file_seek(&g_lfs, &file, buf_len-1024, LFS_SEEK_SET);
    	lfs_file_read(&g_lfs, &file, (void*)test_buf, sizeof(test_buf));
        lfs_file_close(&g_lfs, &file);
       // NRF_LOG_HEXDUMP_INFO(test_buf, 7);

        // release any resources we were using
        lfs_unmount(&g_lfs);
        */

		
    err = lfs_file_open(&g_lfs, &file, "/boot_count", LFS_O_RDWR | LFS_O_CREAT);
    err = lfs_file_read(&g_lfs, &file, &boot_count, sizeof(boot_count));

    // update boot count
    boot_count += 1;
    err = lfs_file_rewind(&g_lfs, &file);
    err = lfs_file_write(&g_lfs, &file, &boot_count, sizeof(boot_count));

    // remember the storage is not updated until the file is closed successfully
    err = lfs_file_close(&g_lfs, &file);

    // release any resources we were using
    err = lfs_unmount(&g_lfs);
}


