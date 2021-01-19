/**
 * @file sst26_driver.c
 * @author Aaron Helmore
 * @brief 
 * @version 0.1
 * @date 2020-12-21
 * 
 * @copyright Copyright (c) 2020
 * 
 */

// TODO: Finish all functions

#include "sst26_driver.h"

/**
 * @brief Initialise the STM32L4xx HAL QSPI with the correct address size
 * 
 * @return int: 0 on success, 1 on failure
 */
int sst26_init() {
    hqspi.Init.FlashSize = 24;
    hqspi.Init.FifoThreshold = 16;
    
    // Re-init QSPI with flash size and FIFO size
    if (HAL_QSPI_Init(&hqspi))
        return SST26_ERROR;

    // Enable Quad IO
    if (sst26_cmd(SST26_EQIO, false))
        return SST26_ERROR;

    // Unlock global write protection
    if (sst26_cmd(SST26_ULBPR, true))
        return SST26_ERROR;

    return SST26_OK;
}

/**
 * @brief Executes a SST26 command that only requires one instruction cycle
 * 
 *  Currently supports:
 *      - SST26_NOP
 *      - SST26_RST
 *      - SST26_EQIO
 *      - SST26_RSTQIO
 *      - SST26_WREN
 *      - SST26_WRDI
 *      - SST26_WRSU
 *      - SST26_WRRE
 *      - SST26_DPD
 * 
 * NOTE: SST26_RST performs both RSTEN and RST
 * NOTE: SST_EQIO will ALWAYS use qspiMode == false
 * 
 * @param command: The command to execute
 * @param qspiMode; Whether 4 line instructions should be used
 * @return int: 0 on success, 1 on failure
 */
int sst26_cmd(SST26SingleCMD_t command, bool qspiMode) {
    QSPI_CommandTypeDef qspiCommand = {0};

    if (command == SST26_RST)
        qspiCommand.Instruction = SST26_RSTEN;
    else 
        qspiCommand.Instruction = command;

    // Set instruction mode, NOTE: SST26_EQIO only allows SPI
    if (qspiMode && (command != SST26_EQIO))
        qspiCommand.InstructionMode = QSPI_INSTRUCTION_4_LINES;
    else 
        qspiCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;

    if (HAL_QSPI_Command(&hqspi, &qspiCommand, SST26_QSPI_TIMEOUT))
        return SST26_ERROR;

    if (command == SST26_RST) {
        qspiCommand.Instruction = command;

        if (HAL_QSPI_Command(&hqspi, &qspiCommand, SST26_QSPI_TIMEOUT))
            return SST26_ERROR;
    }

    return SST26_OK;
}

/**
 * @brief Read a SST26 register
 * 
 * NOTE: currently supports:
 *              - SST26_RDST
 *              - SST26_RDCR
 * 
 * @param command: Command to read the desired register
 * @param data: The contents of the read register
 * @param qspiMode: Should QSPI be used for the command
 * @return int: 0 on success, 1 on failure
 */
int sst26_read_reg(SST26ReadRegCMD_t command, uint8_t *data, bool qspiMode) {
    QSPI_CommandTypeDef qspiCommand = {0};

    qspiCommand.Instruction = command;
    qspiCommand.NbData = 1;

    if (qspiMode) {
        qspiCommand.InstructionMode = QSPI_INSTRUCTION_4_LINES;
        qspiCommand.DataMode = QSPI_DATA_4_LINES;
        qspiCommand.AlternateBytes = QSPI_ALTERNATE_BYTES_NONE;
        
        qspiCommand.DummyCycles = 2; // NOTE: STM32 dummy cycle is 1 sqpi_clk
    } else {
        qspiCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
        qspiCommand.DataMode = QSPI_DATA_1_LINE;
    }

    if (HAL_QSPI_Command(&hqspi, &qspiCommand, SST26_QSPI_TIMEOUT))
        return SST26_ERROR;

    if (HAL_QSPI_Receive(&hqspi, data, SST26_QSPI_TIMEOUT))
        return SST26_ERROR;

    return SST26_OK;
}

/**
 * @brief Perform a SQI fast read
 * 
 * NOTE: If Quad I/O has not been enabled, SST26_HS_READ will execute in SPI
 * 
 * @param address: Address to start reading from
 * @param data: Read data
 * @param num: Number of bytes to read
 * @return int: 0 on success, 1 on failure
 */
int sst26_sqi_read(uint32_t address, uint8_t *data, uint32_t num) {
    QSPI_CommandTypeDef qspiCommand = {0};

    qspiCommand.Instruction = SST26_HS_READ;
    qspiCommand.Address = address;
    qspiCommand.NbData = num;

    qspiCommand.InstructionMode = QSPI_INSTRUCTION_4_LINES;
    qspiCommand.AddressMode = QSPI_ADDRESS_4_LINES;
    qspiCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_4_LINES;
    qspiCommand.DataMode = QSPI_DATA_4_LINES;

    qspiCommand.AddressSize = SST26_ADDR_BITS;
    qspiCommand.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
    
    qspiCommand.AlternateBytes = 0x00;
    qspiCommand.DummyCycles = 4;

    if (HAL_QSPI_Command(&hqspi, &qspiCommand, SST26_QSPI_TIMEOUT))
        return SST26_ERROR;

    if (HAL_QSPI_Receive(&hqspi, data, SST26_QSPI_TIMEOUT))
        return SST26_ERROR;

    return SST26_OK;
}

int sst26_sqi_read_dma(uint32_t address, uint8_t *data, uint32_t num) {
    QSPI_CommandTypeDef qspiCommand = {0};

    qspiCommand.Instruction = SST26_HS_READ;
    qspiCommand.Address = address;
    qspiCommand.NbData = num;

    qspiCommand.InstructionMode = QSPI_INSTRUCTION_4_LINES;
    qspiCommand.AddressMode = QSPI_ADDRESS_4_LINES;
    qspiCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_4_LINES;
    qspiCommand.DataMode = QSPI_DATA_4_LINES;

    qspiCommand.AddressSize = SST26_ADDR_BITS;
    qspiCommand.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
    
    qspiCommand.DummyCycles = 4;

    if (HAL_QSPI_Command(&hqspi, &qspiCommand, SST26_QSPI_TIMEOUT))
        return SST26_ERROR;

    if (HAL_QSPI_Receive_DMA(&hqspi, data))
        return SST26_ERROR;

    return SST26_OK;
}

/**
 * @brief Set the burst size for burst read commands
 * 
 * @param burstLength: The length to set the burst to
 * @return int: 0 on success, 1 on failure
 */
int sst26_sqi_set_burst(SST26BurstLength_t burstLength) {
    QSPI_CommandTypeDef qspiCommand = {0};

    qspiCommand.Instruction = SST26_SB;
    qspiCommand.NbData = 1;

    qspiCommand.InstructionMode = QSPI_INSTRUCTION_4_LINES;
    qspiCommand.AddressMode = QSPI_ADDRESS_NONE;
    qspiCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    qspiCommand.DataMode = QSPI_DATA_4_LINES;

    if (HAL_QSPI_Command(&hqspi, &qspiCommand, SST26_QSPI_TIMEOUT))
        return SST26_ERROR;

    if (HAL_QSPI_Transmit(&hqspi, &burstLength, SST26_QSPI_TIMEOUT))
        return SST26_ERROR;

    return SST26_OK;
}

int sst26_sqi_read_burst(uint32_t address, uint8_t *data, uint32_t num) {
    QSPI_CommandTypeDef qspiCommand = {0};

    qspiCommand.Instruction = SST26_RBSQI;
    qspiCommand.Address = address;
    qspiCommand.NbData = num;

    qspiCommand.InstructionMode = QSPI_INSTRUCTION_4_LINES;
    qspiCommand.AddressMode = QSPI_ADDRESS_4_LINES;
    qspiCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    qspiCommand.DataMode = QSPI_DATA_4_LINES;

    qspiCommand.DummyCycles = 6;

    if (HAL_QSPI_Command(&hqspi, &qspiCommand, SST26_QSPI_TIMEOUT))
        return SST26_ERROR;

    if (HAL_QSPI_Receive(&hqspi, data, SST26_QSPI_TIMEOUT))
        return SST26_ERROR;

    return SST26_OK;
}

/**
 * @brief Read the J-ID using QSPI
 * 
 * NOTE: Data is required to be at least 3 bytes large
 * 
 * @param data: J-ID that was read. MUST BE >=3 IN LENGTH
 * @return int: 0 on success, 1 on failure
 */
int sst26_qspi_jid_read(uint8_t *data) {
    QSPI_CommandTypeDef qspiCommand = {0};

    qspiCommand.Instruction = SST26_QUAD_J_ID;
    qspiCommand.NbData = 3;

    qspiCommand.InstructionMode = QSPI_INSTRUCTION_4_LINES;
    qspiCommand.AddressMode = QSPI_ADDRESS_NONE;
    qspiCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    qspiCommand.DataMode = QSPI_DATA_4_LINES;

    qspiCommand.DummyCycles = 2;

    if (HAL_QSPI_Command(&hqspi, &qspiCommand, SST26_QSPI_TIMEOUT))
        return SST26_ERROR;

    if (HAL_QSPI_Receive(&hqspi, &data, SST26_QSPI_TIMEOUT))
        return SST26_ERROR;

    return SST26_OK;
}

// TODO: Poll BUSY flag to know when to return
/**
 * @brief Erase a sector
 * 
 * @param address: The address of the sector to erase
 * @return int: 0 on success, 1 on failure
 */
int sst26_sqi_sector_erase(uint32_t address) {
    QSPI_CommandTypeDef qspiCommand = {0};
    uint8_t statusReg = 0;

    qspiCommand.Instruction = SST26_SE;
    qspiCommand.Address = address;

    qspiCommand.InstructionMode = QSPI_INSTRUCTION_4_LINES;
    qspiCommand.AddressMode = QSPI_ADDRESS_4_LINES;
    qspiCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    qspiCommand.DataMode = QSPI_DATA_NONE;

    qspiCommand.AddressSize = SST26_ADDR_BITS;

    // Enable write before clearing 
    if (sst26_cmd(SST26_WREN, true))
        return SST26_ERROR;

    if (HAL_QSPI_Command(&hqspi, &qspiCommand, SST26_QSPI_TIMEOUT))
        return SST26_ERROR;

    do {
        if (sst26_read_reg(SST26_RDST, &statusReg, true))
            return SST26_ERROR;
    } while (statusReg & (1 << SST26_SR_BUSY));

    return SST26_OK;
}

// TODO: poll BUSY flag
/**
 * @brief Erase a block
 * 
 * @param address: The address of the block to erase 
 * @return int: 0 on success, 1 on failure
 */
int sst26_sqi_block_erase(uint32_t address) {
    QSPI_CommandTypeDef qspiCommand = {0};
    uint8_t statusReg = 0;

    qspiCommand.Instruction = SST26_BE;
    qspiCommand.Address = address;

    qspiCommand.InstructionMode = QSPI_INSTRUCTION_4_LINES;
    qspiCommand.AddressMode = QSPI_ADDRESS_4_LINES;
    qspiCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    qspiCommand.DataMode = QSPI_DATA_NONE;

    qspiCommand.AddressSize = SST26_ADDR_BITS;

    // Enable write before clearing
    if (sst26_cmd(SST26_WREN, true))
        return SST26_ERROR;

    if (HAL_QSPI_Command(&hqspi, &qspiCommand, SST26_QSPI_TIMEOUT))
        return SST26_ERROR;

    do {
        if (sst26_read_reg(SST26_RDST, &statusReg, true))
            return SST26_ERROR;
    } while (statusReg & (1 << SST26_SR_BUSY));

    return SST26_OK;
}

// TODO: Poll BUSY flag
/**
 * @brief Erase the entire chip
 * 
 * @return int: 0 on success, 1 on failure
 */
int sst26_sqi_chip_erase(void) {
    QSPI_CommandTypeDef qspiCommand = {0};
    uint8_t statusReg = 0;

    qspiCommand.Instruction = SST26_CE;

    qspiCommand.InstructionMode = QSPI_INSTRUCTION_4_LINES;
    qspiCommand.AddressMode = QSPI_ADDRESS_NONE;
    qspiCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    qspiCommand.DataMode = QSPI_DATA_NONE;

    // Enable write before clearing
    if (sst26_cmd(SST26_WREN, true))
        return SST26_ERROR;

    if (HAL_QSPI_Command(&hqspi, &qspiCommand, SST26_QSPI_TIMEOUT))
        return SST26_ERROR;

    do {
        if (sst26_read_reg(SST26_RDST, &statusReg, true))
            return SST26_ERROR;
    } while (statusReg & (1 << SST26_SR_BUSY));

    return SST26_OK;
}

// TODO: Poll BUSY flag
/**
 * @brief Program up to a page of the SST26
 * 
 * NOTE: If the given data overlaps or exceeds the page boundary, the SST26
 *       will wrap around to the start of the page and program the bytes there
 * 
 * @param address: The address to write to
 * @param data: The data to program
 * @param num: How many bytes to program
 * @return int 
 */
int sst26_sqi_page_program(uint32_t address, uint8_t *data, uint32_t num) {
    
    if (num > 256)
        return SST26_ERROR;
    
    QSPI_CommandTypeDef qspiCommand = {0};
    uint8_t statusReg = 0;

    qspiCommand.Instruction = SST26_PPSPI;
    qspiCommand.Address = address;
    qspiCommand.NbData = num;

    qspiCommand.InstructionMode = QSPI_INSTRUCTION_4_LINES;
    qspiCommand.AddressMode = QSPI_ADDRESS_4_LINES;
    qspiCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    qspiCommand.DataMode = QSPI_DATA_4_LINES;

    qspiCommand.AddressSize = QSPI_ADDRESS_24_BITS;

    // Enable write before clearing
    if (sst26_cmd(SST26_WREN, true))
        return SST26_ERROR;

    if (HAL_QSPI_Command(&hqspi, &qspiCommand, SST26_QSPI_TIMEOUT))
        return SST26_ERROR;

    if (HAL_QSPI_Transmit(&hqspi, data, SST26_QSPI_TIMEOUT))
        return SST26_ERROR;

    do {
        if (sst26_read_reg(SST26_RDST, &statusReg, true))
            return SST26_ERROR;
    } while (statusReg & (1 << SST26_SR_BUSY));

    return SST26_OK;
}