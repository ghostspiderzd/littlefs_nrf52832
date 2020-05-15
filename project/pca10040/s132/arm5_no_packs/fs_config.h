#ifndef _FS_CONFIG_H
#define _FS_CONFIG_H


#include "lfs.h"
#include "lfs_util.h"








int user_provided_block_device_read(const struct lfs_config *c, lfs_block_t block,
                lfs_off_t off, void *buffer, lfs_size_t size);

int user_provided_block_device_prog(const struct lfs_config *c, lfs_block_t block,
                lfs_off_t off, const void *buffer, lfs_size_t size);

int user_provided_block_device_erase(const struct lfs_config *c, lfs_block_t block);

int user_provided_block_device_sync(const struct lfs_config *c);

void spi_flash_littlefs_init(void);

void spi_flash_littlefs_test(void);

#endif _FS_CONFIG_H