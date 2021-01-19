/**
 * @file sst26_driver.h
 * @author Aaron Helmore
 * @brief 
 * @version 0.1
 * @date 2020-12-21
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#pragma once    // Lets see how this works

#include "stm32l4xx_hal.h"

#include <stdbool.h>

// STM32L4xx HAL handle for Quad SPI
extern QSPI_HandleTypeDef hqspi;

typedef enum SST26Status {
    SST26_OK    = 0,
    SST26_ERROR = 1
} SST26Status_t;

typedef enum SST26SingleCMD {
    SST26_NOP       = (uint8_t)0x00,    // No operation
    SST26_RST       = (uint8_t)0x99,    // Reset memory
    SST26_EQIO      = (uint8_t)0x38,    // Enable quad I/O
    SST26_RSTQIO    = (uint8_t)0xFF,    // Reset quad I/O
    SST26_WREN      = (uint8_t)0x06,    // Write enable
    SST26_WRDI      = (uint8_t)0x04,    // Write disable
    SST26_WRSU      = (uint8_t)0xB0,    // Suspend program/erase
    SST26_WRRE      = (uint8_t)0x30,    // Resume program/erase
    SST26_DPD       = (uint8_t)0xB9,    // Deep power down
    SST26_ULBPR     = (uint8_t)0x98     // Global block protection unlock
} SST26SingleCMD_t;

typedef enum SST26BurstLength {
    SST_8B_BURST    = (uint8_t)0x00,
    SST_16B_BURST   = (uint8_t)0x01,
    SST_32B_BURST   = (uint8_t)0x02,
    SST_64B_BURST   = (uint8_t)0x03
} SST26BurstLength_t;

typedef enum SST26ReadRegCMD {
    SST26_RDST      = (uint8_t)0x05,    // Read status register
    SST26_RDCR      = (uint8_t)0x35     // Read configuration register
} SST26ReadRegCMD_t;

// Device Parameters
#define SST26_PAGE_SIZE     256         // 256B pages
#define SST26_SECTOR_SIZE   4096        // 4kB sectors
#define SST26_FLASH_SIZE    16777216    // 16MB flash size

#define SST26_ADDR_BITS     QSPI_ADDRESS_24_BITS  // 24 bit addresses

#define SST26_QSPI_TIMEOUT  100         // 100ms timeout on qspi

#define SST26_QSPI_INSTRUCTION_MODE QSPI_INSTRUCTION_4_LINES
#define SST26_QSPI_ADDRESS_MODE
#define SST26_QSPI_ALT_BYTE_MODE
#define SST26_QSPI_DATA_MODE
#define SST26_QSPI_DDR_MORE
#define SST26_QSPI_SIO_MODE

// Configuration Instructions
#define SST26_RSTEN     0x66 // Reset enable
#define SST26_WRSR      0x01 // Write status register


// Read Instructions
#define SST26_READ      0x03 // Read memory
#define SST26_HS_READ   0x0B // Read memory at higher speed
#define SST26_SQOR      0x6B // SPI quad ouptut read
#define SST26_SQIOR     0xEB // SPI quad I/O read
#define SST26_SDOR      0x3B // SPI dual output read
#define SST26_SDIOR     0xBB // SPI dual I/O read  
#define SST26_SB        0xC0 // Set burst length
#define SST26_RBSQI     0x0C // SQI read burst with wrap
#define SST26_RBSPI     0xEC // SPI read burst with wrap

// Identification Instructions
#define SST26_JEDEC_ID  0x9F // JEDEC-ID read
#define SST26_QUAD_J_ID 0xAF // Quad I/O J-ID read
#define SST26_SFDP      0x5A // Serial flash discoverable parameters

// Write Intructions
#define SST26_SE        0x20 // Erase sector (4kB)
#define SST26_BE        0xD8 // Erase block (64kB, 32kB or 8kB)
#define SST26_CE        0xC7 // Erase chip
#define SST26_PPSPI     0x02 // SPI page program
#define SST26_PPSQI     0x32 // SQI page program


// Protection Instructions
#define SST26_RBPR      0x72 // Read block-protection register
#define SST26_WBPR      0x42 // Write block-protection register
#define SST26_LBPR      0x8D // Lock down block-protection register
#define SST26_NVWLDR    0xE8 // Non-volatile write lock-down register

#define SST26_RSID      0x88 // Read security ID
#define SST26_PSID      0xA5 // Program user security ID area
#define SST26_LSID      0x85 // Lockout security ID programming

// Power Saving Instructions
#define SST26_RDPD      0xAB // Release from deep power-down mode

// Bit positions of status register bits
#define SST26_SR_BUSY   0   // BUSY bit position
#define SST26_SR_WEL    1   // Write enable latch position
#define SST26_SR_WSE    2   // Write suspend erase status position

/* NOTE: Many commands are currently untested, tested cmd's are:
    - EQIO
    - RSTQIO
    - WREN
    - WRDI
    - ULBPR

    - RDST
    - RDCR

*/



int sst26_init();

int sst26_cmd(SST26SingleCMD_t cmd, bool useSQI);

int sst26_read_reg(SST26ReadRegCMD_t, uint8_t *, bool);

int sst26_sqi_read(uint32_t address, uint8_t *data, uint32_t len);

// TODO: Test this
int sst26_sqi_read_dma(uint32_t, uint8_t *, uint32_t);

// TODO: Test this
int sst26_sqi_set_burst(SST26BurstLength_t);

// TODO: Test this
int sst26_sqi_read_burst(uint32_t, uint8_t *, uint32_t);

// TODO: Test this
int sst26_qspi_jid_read(uint8_t *jid);

int sst26_sqi_sector_erase(uint32_t address);

int sst26_sqi_block_erase(uint32_t address);

int sst26_sqi_chip_erase(void);

int sst26_sqi_page_program(uint32_t address, uint8_t *data, uint32_t len);