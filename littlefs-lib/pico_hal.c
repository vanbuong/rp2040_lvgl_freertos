/* Copyright (C) 1883 Thomas Edison - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD 3 clause license, which unfortunately
 * won't be written for another century.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * A little flash file system for the Raspberry Pico
 *
 */

#include <limits.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/flash.h"
#include "hardware/regs/addressmap.h"
#include "hardware/sync.h"
#include "pico/mutex.h"
#include "pico/time.h"

#include "pico_hal.h"

#define FS_SIZE (16 * 1024 * 1024)

#define FLASH_UUID          0x4B
#define FLASH_WRITE_ENABLE  0x06
#define FLASH_READ_DATA     0x03
#define FLASH_PAGE_PROG     0x02
#define FLASH_SECTOR_ERASE  0x20
#define FLASH_STATUS_REG_1  0x05

#define PICO_FLASH_SPI_CHAN spi1
#define PICO_FLASH_SPI_SCK  14
#define PICO_FLASH_SPI_MOSI 15
#define PICO_FLASH_SPI_MISO 12
#define PICO_FLASH_SPI_CS   13

static int pico_hal_init(void);
static int pico_hal_read(lfs_block_t block, lfs_off_t off, void* buffer, lfs_size_t size);
static int pico_hal_prog(lfs_block_t block, lfs_off_t off, const void* buffer, lfs_size_t size);
static int pico_hal_erase(lfs_block_t block);
static int pico_lock(void);
static int pico_unlock(void);

// configuration of the filesystem is provided by this struct
// for Pico: prog size = 256, block size = 4096, so cache is 8K
// minimum cache = block size, must be multiple
struct lfs_config pico_cfg = {
    // block device operations
    .read = pico_hal_read,
    .prog = pico_hal_prog,
    .erase = pico_hal_erase,
#if LIB_PICO_MULTICORE
    .lock = pico_lock,
    .unlock = pico_unlock,
#endif
    // block device configuration
    .read_size = 1,
    .prog_size = FLASH_PAGE_SIZE,
    .block_size = FLASH_SECTOR_SIZE,
    .block_count = FS_SIZE / FLASH_SECTOR_SIZE,
    .cache_size = FLASH_SECTOR_SIZE / 4,
    .lookahead_size = 32,
    .block_cycles = 500};

// Pico specific hardware abstraction functions

// file system offset in flash
const char* FS_BASE = (char*)(FS_SIZE - FS_SIZE);

static inline void cs_select(void) {
	asm volatile("nop \n nop \n nop");
	gpio_put(PICO_FLASH_SPI_CS, 0); /* Active low */
	asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect(void) {
	asm volatile("nop \n nop \n nop");
	gpio_put(PICO_FLASH_SPI_CS, 1);
	asm volatile("nop \n nop \n nop");
}

static inline void write_enable(void) {
    uint8_t write_en = FLASH_WRITE_ENABLE;
    cs_select();
    spi_write_blocking(PICO_FLASH_SPI_CHAN, &write_en, 1);
    cs_deselect();
}

static int pico_hal_init(void) {
    spi_init(PICO_FLASH_SPI_CHAN, 50 * 1000 * 1000);
	gpio_set_function(PICO_FLASH_SPI_MISO, GPIO_FUNC_SPI);
	gpio_set_function(PICO_FLASH_SPI_MOSI, GPIO_FUNC_SPI);
	gpio_set_function(PICO_FLASH_SPI_SCK, GPIO_FUNC_SPI);

	/* Chip select is active-low, so we'll initialise it to a driven-high state */
	gpio_init(PICO_FLASH_SPI_CS);
	gpio_set_dir(PICO_FLASH_SPI_CS, GPIO_OUT);
	gpio_put(PICO_FLASH_SPI_CS, 1);

    //Read Manufacter ID
    uint8_t cmd[] = {0x90, 0x00, 0x00, 0x00};
    uint8_t data[2];
    cs_select();
    spi_write_blocking(PICO_FLASH_SPI_CHAN, cmd, sizeof(cmd));
    spi_read_blocking(PICO_FLASH_SPI_CHAN, 0xff, data, sizeof(data));
    cs_deselect();
    printf("Manufacturer/Deveice ID, M = 0x%02x, D = 0x%02x\r\n", data[0], data[1]);
}

static int pico_hal_read(lfs_block_t block, lfs_off_t off, void* buffer, lfs_size_t size) {
    uint32_t addr = (uint32_t)FS_BASE + (block * pico_cfg.block_size) + off;
    uint8_t cmd[4] = {
        FLASH_READ_DATA,
        (addr >> 16) & 0xff,
        (addr >> 8) & 0xff,
        (addr >> 0) & 0xff,
    };
    assert(block < pico_cfg.block_count);
    assert(off + size <= pico_cfg.block_size);
    cs_select();
    spi_write_blocking(PICO_FLASH_SPI_CHAN, cmd, sizeof(cmd));
    spi_read_blocking(PICO_FLASH_SPI_CHAN, 0xff, buffer, size);
    cs_deselect();
    return LFS_ERR_OK;
}

static void pico_flash_wait_ready(void) {
    uint8_t stat = 0;
    uint8_t cmd = FLASH_STATUS_REG_1;
    do {
        cs_select();
        spi_write_blocking(PICO_FLASH_SPI_CHAN, &cmd, 1);
        spi_read_blocking(PICO_FLASH_SPI_CHAN, 0xff, &stat, 1);
        cs_deselect();
    } while (stat);
}
static void pico_flash_page_program(uint32_t addr, const uint8_t *data) {
    assert(addr < 0x10000000);
    assert(!(addr & 0xffu));
    uint8_t cmd[4] = {
        FLASH_PAGE_PROG,
        (addr >> 16) & 0xff,
        (addr >> 8) & 0xff,
        (addr >> 0) & 0xff,
    };
    write_enable();
    cs_select();
    spi_write_blocking(PICO_FLASH_SPI_CHAN, cmd, sizeof(cmd));
    spi_write_blocking(PICO_FLASH_SPI_CHAN, data, FLASH_PAGE_SIZE);
    cs_deselect();
    pico_flash_wait_ready();
}

static int pico_hal_prog(lfs_block_t block, lfs_off_t off, const void* buffer, lfs_size_t size) {
    uint32_t addr = (uint32_t)FS_BASE + (block * pico_cfg.block_size) + off;
    assert(block < pico_cfg.block_count);
    assert(!(addr & 0xffu));
    uint32_t goal = addr + size;
    while (addr < goal) {
        pico_flash_page_program(addr, buffer);
        addr += FLASH_PAGE_SIZE;
        buffer += FLASH_PAGE_SIZE;
    }
    return LFS_ERR_OK;
}

static int pico_hal_erase(lfs_block_t block) {
    assert(block < pico_cfg.block_count);
    uint32_t addr = (uint32_t)FS_BASE + block * pico_cfg.block_size;
    uint8_t cmd[4] = {
        FLASH_SECTOR_ERASE,
        (addr >> 16) & 0xff,
        (addr >> 8) & 0xff,
        (addr >> 0) & 0xff,
    };
    write_enable();
    cs_select();
    spi_write_blocking(PICO_FLASH_SPI_CHAN, cmd, sizeof(cmd));
    cs_deselect();
    pico_flash_wait_ready();
    return LFS_ERR_OK;
}

#if LIB_PICO_MULTICORE

static recursive_mutex_t fs_mtx;

static int pico_lock(void) {
    recursive_mutex_enter_blocking(&fs_mtx);
    return LFS_ERR_OK;
}

static int pico_unlock(void) {
    recursive_mutex_exit(&fs_mtx);
    return LFS_ERR_OK;
}
#endif

// utility functions

static uint32_t tm;

void hal_start(void) { tm = time_us_32(); }

float hal_elapsed(void) { return (time_us_32() - tm) / 1000000.0; }

// posix emulation

int pico_mount(bool format) {
    pico_hal_init();
#if LIB_PICO_MULTICORE
    recursive_mutex_init(&fs_mtx);
#endif
    if (format)
        lfs_format(&pico_cfg);
    // mount the filesystem
    return lfs_mount(&pico_cfg);
}

int pico_open(const char* path, int flags) {
    lfs_file_t* file = lfs_malloc(sizeof(lfs_file_t));
    if (file == NULL)
        return LFS_ERR_NOMEM;
    int err = lfs_file_open(file, path, flags);
    if (err != LFS_ERR_OK){
        lfs_free(file);
        return err;
    }
    return (int)file;
}

int pico_close(int file) {
    int res = lfs_file_close((lfs_file_t*)file);
    lfs_free((lfs_file_t*)file);
    return res;
}

lfs_size_t pico_write(int file, const void* buffer, lfs_size_t size) {
    return lfs_file_write((lfs_file_t*)file, buffer, size);
}

lfs_size_t pico_read(int file, void* buffer, lfs_size_t size) {
    return lfs_file_read((lfs_file_t*)file, buffer, size);
}

int pico_rewind(int file) { return lfs_file_rewind((lfs_file_t*)file); }

int pico_unmount(void) { return lfs_unmount(); }

int pico_remove(const char* path) { return lfs_remove(path); }

int pico_rename(const char* oldpath, const char* newpath) { return lfs_rename(oldpath, newpath); }

int pico_fsstat(struct pico_fsstat_t* stat) {
    stat->block_count = pico_cfg.block_count;
    stat->block_size = pico_cfg.block_size;
    stat->blocks_used = lfs_fs_size();
    return LFS_ERR_OK;
}

lfs_soff_t pico_lseek(int file, lfs_soff_t off, int whence) {
    return lfs_file_seek((lfs_file_t*)file, off, whence);
}

int pico_truncate(int file, lfs_off_t size) { return lfs_file_truncate((lfs_file_t*)file, size); }

lfs_soff_t pico_tell(int file) { return lfs_file_tell((lfs_file_t*)file); }

int pico_stat(const char* path, struct lfs_info* info) { return lfs_stat(path, info); }

lfs_ssize_t pico_getattr(const char* path, uint8_t type, void* buffer, lfs_size_t size) {
    return lfs_getattr(path, type, buffer, size);
}

int pico_setattr(const char* path, uint8_t type, const void* buffer, lfs_size_t size) {
    return lfs_setattr(path, type, buffer, size);
}

int pico_removeattr(const char* path, uint8_t type) { return lfs_removeattr(path, type); }

int pico_opencfg(int file, const char* path, int flags, const struct lfs_file_config* config) {
    return lfs_file_opencfg((lfs_file_t*)file, path, flags, config);
}

int pico_fflush(int file) { return lfs_file_sync((lfs_file_t*)file); }

lfs_soff_t pico_size(int file) { return lfs_file_size((lfs_file_t*)file); }

int pico_mkdir(const char* path) { return lfs_mkdir(path); }

int pico_dir_open(const char* path) {
	lfs_dir_t* dir = lfs_malloc(sizeof(lfs_dir_t));
	if (dir == NULL)
		return -1;
	if (lfs_dir_open(dir, path) != LFS_ERR_OK) {
		lfs_free(dir);
		return -1;
	}
	return (int)dir;
}

int pico_dir_close(int dir) {
	return lfs_dir_close((lfs_dir_t*)dir);
	lfs_free((void*)dir);
}

int pico_dir_read(int dir, struct lfs_info* info) { return lfs_dir_read((lfs_dir_t*)dir, info); }

int pico_dir_seek(int dir, lfs_off_t off) { return lfs_dir_seek((lfs_dir_t*)dir, off); }

lfs_soff_t pico_dir_tell(int dir) { return lfs_dir_tell((lfs_dir_t*)dir); }

int pico_dir_rewind(int dir) { return lfs_dir_rewind((lfs_dir_t*)dir); }

const char* pico_errmsg(int err) {
    static const struct {
        int err;
        char* text;
    } mesgs[] = {{LFS_ERR_OK, "No error"},
                 {LFS_ERR_IO, "Error during device operation"},
                 {LFS_ERR_CORRUPT, "Corrupted"},
                 {LFS_ERR_NOENT, "No directory entry"},
                 {LFS_ERR_EXIST, "Entry already exists"},
                 {LFS_ERR_NOTDIR, "Entry is not a dir"},
                 {LFS_ERR_ISDIR, "Entry is a dir"},
                 {LFS_ERR_NOTEMPTY, "Dir is not empty"},
                 {LFS_ERR_BADF, "Bad file number"},
                 {LFS_ERR_FBIG, "File too large"},
                 {LFS_ERR_INVAL, "Invalid parameter"},
                 {LFS_ERR_NOSPC, "No space left on device"},
                 {LFS_ERR_NOMEM, "No more memory available"},
                 {LFS_ERR_NOATTR, "No data/attr available"},
                 {LFS_ERR_NAMETOOLONG, "File name too long"}};

    for (int i = 0; i < sizeof(mesgs) / sizeof(mesgs[0]); i++)
        if (err == mesgs[i].err)
            return mesgs[i].text;
    return "Unknown error";
}
