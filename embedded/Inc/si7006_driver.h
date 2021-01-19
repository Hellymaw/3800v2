/**
 * @file si7006_driver.h
 * @author Aaron Helmore
 * @brief A STM32L4XX HAL based driver for the Si7006 temperature and relative humidity sensor
 * @version 0.1
 * @date 2020-12-10
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#pragma once

#include "stm32l4xx_hal.h"

#include <stdint.h>

// I2C HAL handle
extern I2C_HandleTypeDef hi2c1;

// Structure to hold Si7006 raw data
typedef struct Si7006DataRaw {
    uint16_t relativeHumidity;
    uint16_t temperature;
} Si7006DataRaw_t;

// Structure to hold Si7006 converted data
typedef struct Si7006Data {
    float relativeHumidity;
    float temperature;
} Si7006Data_t;

// Enumeration for status of Si7006 command
typedef enum Si7006Status {
    SI7006_OK       = 0,
    SI7006_ERROR    = 1
} Si7006Status;

// Measurement resolution values
typedef enum Si7006Resolution {
    SI7006_RES_RH12B_T14B = 0x00,
    SI7006_RES_RH8B_T12B = 0x01,
    SI7006_RES_RH10B_T13B = 0x80,
    SI7006_RES_RH11B_T11B = 0x81
} Si7006Resolution;

#define SI7006_STM32_I2C_HAL_TIMEOUT        100

#define SI7006_USE_HOLD_MASTER              1

// I2C slave address for Si7006 temp/RH sensor (7b)
#define SI7006_I2C_SLAVE_ADDR               (0x40 << 1) // Shift 1 due to HAL

// Measure relative humidity (RH) commands for hold/no-hold master mode
#define SI7006_MEASURE_RH_HOLD_MASTER       0xE5
#define SI7006_MEASURE_RH_NO_HOLD_MASTER    0xF5

// Measure temperature commands for hold/no-hold master mode
#define SI7006_MEASURE_TEMP_HOLD_MASTER     0xE3
#define SI7006_MEASURE_TEMP_NO_HOLD_MASTER  0xF3

// Used to read a temp value after performing a RH measurement
#define SI7006_READ_TEMP_FROM_PREV_RH       0xE0 

// Read and write commands for user register 1
#define SI7006_WRITE_USER_REG               0xE6
#define SI7006_READ_USER_REG                0xE7

// Read and write for heater control register
#define SI7006_WRITE_HEATER_CTRL_REG        0x51
#define SI7006_READ_HEATER_CTRL_REG         0x11

// 1st and 2nd bytes for reading Electronic ID 1st byte
#define SI7006_READ_ID_BYTE_1               0xFA0F

// 1st and 2nd bytes for reading Electronic ID 2nd byte
#define SI7006_READ_ID_BYTE_2               0xFCC9

// 1st and 2nd bytes for reading firmware revision
#define SI7006_READ_FIRMWARE_REV_1          0x84
#define SI7006_READ_FIRMWARE_REV_2          0xB8

// Reset the Si7006
#define SI7006_RESET                        0xFE

// Bit positions of user register 1 options
#define SI7006_USER_REG_HTRE                0x04    // Heater enable

// Bit masks for user register 1
#define SI7006_USER_REG_RES_MASK            0x81    // Mask for resolution
#define SI7006_USER_REG_VDDS_MASK           0x40    // VDD status: 0 == OK

// Bit mask for heater control register 1
#define SI7006_HEATER_CTRL_REG_MASK         0x0F

// Electronic ID for engineering samples
#define SI7006_ID_ENGG_SAMPLE_1             0x00
#define SI7006_ID_ENGG_SAMPLE_2             0xFF

// Electronic ID for regular Si7006
#define SI7006_ID_REGULAR                   0x06

// Firmware version 1.0 register value
#define SI7006_FIRMWARE_VER_1_0             0xFF

// Firmware version 2.0 register value
#define SI7006_FIRMWARE_VER_2_0             0x20

// Low-level interface to start a measurement on the Si7006
int si7006_measurement(uint8_t, uint8_t *);

// Perform a relative humidity measurement
int si7006_measure_humidity(Si7006DataRaw_t *);

// Perform a temperature measurement
int si7006_measure_temperature(Si7006DataRaw_t *);

// Perform a relativity measurement and read the temperature measurement
int si7006_measure_humidity_and_temperature(Si7006DataRaw_t *);

// Reset the device
int si7006_reset(void);

// Read a register of the device
int si7006_read_register(uint16_t, uint8_t *);

// Write a register of the device
int si7006_write_register(uint16_t, uint8_t);

// Set the measurement resolution
int si7006_set_resolution(Si7006Resolution);

// Enable the onboard heater
int si7006_enable_heater(uint8_t);

// Disable the onboard heater
int si7006_disable_heater(void);

// Read the Si7006 Serial Number
int si7006_read_serial_number(uint64_t *);

float si7006_convert_humidity(uint16_t);

float si7006_convert_temperature(uint16_t);

void si7006_convert_measurements(Si7006Data_t *, Si7006DataRaw_t);