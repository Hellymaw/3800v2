/**
 * @file si7006_driver.c
 * @author Aaron Helmore
 * @brief A STM32L4XX HAL based driver for the Si7006 temperature and relative humidity sensor
 * @version 0.1
 * @date 2020-12-10
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "si7006_driver.h"

/**
 * @brief provides a low-level interface to start a measurement on the SI7006
 * 
 * @param command: command for the measurement to be made
 * @param data: storage variable for measurement being made 
 * @return int: SI7006_OK on success, SI7006_ERROR on failure 
 */
int si7006_measurement(uint8_t command, uint8_t *data) {

    if (HAL_I2C_Mem_Read(&hi2c1, SI7006_I2C_SLAVE_ADDR, command, 1, data, 2, SI7006_STM32_I2C_HAL_TIMEOUT))
        return SI7006_ERROR;

    return SI7006_OK;
}

/**
 * @brief start a Si7006 relative humidity measurement
 * 
 * @param sensorData: container for measurement data 
 * @return int: SI7006_OK on success, SI7006_ERROR on failure 
 */
int si7006_measure_humidity(Si7006DataRaw_t *sensorData) {
    uint8_t data[2] = {0};

    if (si7006_measurement((SI7006_USE_HOLD_MASTER) ? SI7006_MEASURE_RH_HOLD_MASTER : SI7006_MEASURE_RH_NO_HOLD_MASTER, data))
        return SI7006_ERROR;

    sensorData->relativeHumidity = (uint16_t)data[0] << 8 | data[1];

    return SI7006_OK;
}

/**
 * @brief start a Si7006 temperature measurement
 * 
 * @param sensorData: container for measurement data
 * @return int: SI7006_OK on success, SI7006_ERROR on failure 
 */
int si7006_measure_temperature(Si7006DataRaw_t *sensorData) {
    uint8_t data[2] = {0};

    if (si7006_measurement((SI7006_USE_HOLD_MASTER) ? SI7006_MEASURE_TEMP_HOLD_MASTER : SI7006_MEASURE_TEMP_NO_HOLD_MASTER, data))
        return SI7006_ERROR;

    sensorData->temperature = (uint16_t)data[0] << 8 | data[1];

    return SI7006_OK;
}

/**
 * @brief starts a Si7006 relative humidity measurement and reads the temperature measurement
 * 
 * @param sensorData: container for measurement data
 * @return int: SI7006_OK on success, SI7006_ERROR on failure  
 */
int si7006_measure_humidity_and_temperature(Si7006DataRaw_t *sensorData) {
    uint8_t data[2] = {0};

    if (si7006_measure_humidity(data))
        return SI7006_ERROR;

    sensorData->relativeHumidity = (uint16_t)data[0] << 8 | data[1];

    if (si7006_measurement(SI7006_READ_TEMP_FROM_PREV_RH, data))
        return SI7006_ERROR;

    sensorData->temperature = (uint16_t)data[0] << 8 | data[1];

    return SI7006_OK;
}

/**
 * @brief resets the Si7006
 * 
 * @return int: SI7006_OK on success, SI7006_ERROR on failure  
 */
int si7006_reset(void) {
    uint8_t command = SI7006_RESET;

    if (HAL_I2C_Master_Transmit(&hi2c1, SI7006_I2C_SLAVE_ADDR, &command, 1, SI7006_STM32_I2C_HAL_TIMEOUT))
        return SI7006_ERROR;

    return SI7006_OK;
}

/**
 * @brief reads a Si7006 register
 * 
 * @param registerAddr: Si7006 register address to be read
 * @param registerData: container for the register dataa
 * @return int: SI7006_OK on success, SI7006_ERROR on failure 
 */
int si7006_read_register(uint16_t registerAddr, uint8_t *registerData) {
    
    if (HAL_I2C_Mem_Read(&hi2c1, SI7006_I2C_SLAVE_ADDR, registerAddr, 1, registerData, 1, SI7006_STM32_I2C_HAL_TIMEOUT))
        return SI7006_ERROR;

    return SI7006_OK;
}

/**
 * @brief writes to a Si7006 register
 * 
 * @param registerAddr: Si7006 register address to write to 
 * @param data: the data to be wrote to the register
 * @return int: SI7006_OK on success, SI7006_ERROR on failure 
 */
int si7006_write_register(uint16_t registerAddr, uint8_t data) {

    if (HAL_I2C_Mem_Read(&hi2c1, SI7006_I2C_SLAVE_ADDR, registerAddr, 1, &data, 1, SI7006_STM32_I2C_HAL_TIMEOUT))
        return SI7006_ERROR;
    
    return SI7006_OK;
}

/**
 * @brief sets the measurement resolution of the Si7006
 * 
 * @param resolution: the measurement resolution to set the Si7006 to 
 * @return int: SI7006_OK on success, SI7006_ERROR on failure 
 */
int si7006_set_resolution(Si7006Resolution resolution) {
    uint8_t registerValue = 0;

    if (si7006_read_register(SI7006_READ_USER_REG, &registerValue))
        return SI7006_ERROR;
    
    registerValue &= ~SI7006_USER_REG_RES_MASK; // Clear resolution bits
    registerValue |= (resolution & SI7006_USER_REG_RES_MASK); // Set resolution

    if (si7006_write_register(SI7006_READ_USER_REG, &registerValue))
        return SI7006_ERROR;

    return SI7006_OK;
}

/**
 * @brief Enables the Si7006 internal heater and sets the heater current draw
 * 
 * @param heaterCurrent: 4b register value for heater current 
 * @return int: SI7006_OK on success, SI7006_ERROR on failure
 */
int si7006_enable_heater(uint8_t heaterCurrent) {
    uint8_t registerValue = 0;

    if (si7006_read_register(SI7006_READ_HEATER_CTRL_REG, &registerValue))
        return SI7006_ERROR;

    registerValue &= ~SI7006_HEATER_CTRL_REG_MASK;
    registerValue |= (heaterCurrent & SI7006_HEATER_CTRL_REG_MASK);

    if (si7006_write_register(SI7006_WRITE_HEATER_CTRL_REG, &registerValue));
        return SI7006_ERROR;

    if (si7006_read_register(SI7006_READ_USER_REG, &registerValue))
        return SI7006_ERROR;

    registerValue |= SI7006_USER_REG_HTRE;

    if (si7006_write_register(SI7006_WRITE_USER_REG, &registerValue));
        return SI7006_ERROR;

    return SI7006_OK;
}

/**
 * @brief Disable Si7006 internal heater
 * 
 * @return int: SI7006_OK on success, SI7006_ERROR on failure
 */
int si7006_disable_heater(void) {
    uint8_t registerValue = 0;

    if (si7006_read_register(SI7006_READ_USER_REG, &registerValue))
        return SI7006_ERROR;

    registerValue &= ~SI7006_USER_REG_HTRE;

    if (si7006_write_register(SI7006_WRITE_USER_REG, &registerValue));
        return SI7006_ERROR;

    return SI7006_OK;
}

/**
 * @brief Reads the SI7006 electronic ID number, NOTE: Doesn't compute CRC currently
 * 
 * @param serialNumber: Container for electronic ID
 * @return int: SI7006_OK on success, SI7006_ERROR on failure
 */
int si7006_read_serial_number(uint64_t *serialNumber) {
    uint8_t registerValue[8] = {0};

    // Read the first 4 bytes of ID & their CRC 
    if (HAL_I2C_Mem_Read(&hi2c1, SI7006_I2C_SLAVE_ADDR, SI7006_READ_ID_BYTE_1, 2, registerValue, 8, SI7006_STM32_I2C_HAL_TIMEOUT))
        return SI7006_ERROR;
    
    // TODO: Nicer implementation using CRC checking
    *serialNumber = (registerValue[0] << 56) | (registerValue[2] << 48) | (registerValue[4] << 40) | (registerValue[6] << 32);
    
    // Read last 4 bytes of ID & their CRC
    if (HAL_I2C_Mem_Read(&hi2c1, SI7006_I2C_SLAVE_ADDR, SI7006_READ_ID_BYTE_2, 2, registerValue, 6, SI7006_STM32_I2C_HAL_TIMEOUT))
        return SI7006_ERROR;
    
    *serialNumber |= (registerValue[0] << 24) | (registerValue[1] << 16) | (registerValue[3] << 8) | (registerValue[4]);
}

float si7006_convert_humidity(uint16_t humidity) {
    
    return (125.0 * humidity) / 65536 - 6;
}

float si7006_convert_temperature(uint16_t temperature) {

    return (175.72 * temperature) / 65536 - 46.85;
}

void si7006_convert_measurements(Si7006Data_t* convertedData, Si7006DataRaw_t measurements) {
    
    convertedData->relativeHumidity = si7006_convert_humidity(measurements.relativeHumidity);
    convertedData->temperature = si7006_convert_temperature(measurements.temperature);

}