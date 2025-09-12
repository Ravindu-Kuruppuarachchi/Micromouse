#include "VL53L0X.h"
#include "stdio.h"

// Define I2C handle (adjust as needed)
I2C_HandleTypeDef hi2c1;

// Default VL53L0X I2C address is 0x29
#define CurrentAddress 0x29

// Register map and initialization commands
#define VL53L0X_REG_SYSRANGE_START 0x00
#define VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x71
#define VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP 0x46

// Reference Registers (from datasheet)
#define REG_REFERENCE_C0 0xC0
#define REG_REFERENCE_C1 0xC1
#define REG_REFERENCE_C2 0xC2
#define REG_REFERENCE_51 0x51
#define REG_REFERENCE_61 0x61
// Write function to send data to a specific register
//HAL_StatusTypeDef VL53L0X_WriteReg(uint8_t sensor_addr, uint8_t reg, uint8_t value)
//{
//    uint8_t data[2] = {reg, value};
//    return HAL_I2C_Master_Transmit(&hi2c1, sensor_addr, data, 2, HAL_MAX_DELAY);
//}

HAL_StatusTypeDef WriteReg(uint8_t deviceAddress, uint8_t reg, uint8_t value)
{
    uint8_t array[2];
    array[0] = reg;    // Register address
    array[1] = value;  // Value to write
    if(HAL_I2C_Master_Transmit(&hi2c1, (deviceAddress << 1), array, 2, HAL_MAX_DELAY)!= HAL_OK){
    	return HAL_ERROR;
    }
    else{
    	return HAL_OK;
    }

}

// Read function to receive data from a specific register
HAL_StatusTypeDef ReadReg(uint8_t deviceAddress, uint8_t reg, uint8_t *value)
{
    if (HAL_I2C_Master_Transmit(&hi2c1, (deviceAddress << 1)|1, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_I2C_Master_Receive(&hi2c1, (deviceAddress << 1)|1, value, 1, HAL_MAX_DELAY);
}

// Function to set timing budget
HAL_StatusTypeDef SetTimingBudget(uint8_t sensor_addr, uint32_t timing_budget_us)
{
    uint16_t timeout_value;

    // Convert timing budget to sensor-compatible timeout value
    if (timing_budget_us < 20000)
        timing_budget_us = 20000; // Minimum allowed is 20ms
    else if (timing_budget_us > 200000)
        timing_budget_us = 200000; // Maximum allowed is 200ms

    timeout_value = (timing_budget_us / 2) - 1;

    // Set MSRC timeout
    if (WriteReg(sensor_addr, VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP, (timeout_value >> 8) & 0xFF) != HAL_OK)
        return HAL_ERROR;

    if (WriteReg(sensor_addr, VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP + 1, timeout_value & 0xFF) != HAL_OK)
        return HAL_ERROR;

    // Set Final Range timeout
    if (WriteReg(sensor_addr, VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, (timeout_value >> 8) & 0xFF) != HAL_OK)
        return HAL_ERROR;

    if (WriteReg(sensor_addr, VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI + 1, timeout_value & 0xFF) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

// Function to initialize VL53L0X sensor with timing budget
HAL_StatusTypeDef VL53L0X_Init(uint8_t sensor_addr)
{
    uint8_t value = 0;

    // Step 1: Validate I2C interface using reference registers
    if (ReadReg(sensor_addr, REG_REFERENCE_C0, &value) != HAL_OK || value != 0xEE)
    {
        return HAL_ERROR; // Validation failed for REG_REFERENCE_C0
    }
    if (ReadReg(sensor_addr, REG_REFERENCE_C1, &value) != HAL_OK || value != 0xAA)
    {
        return HAL_ERROR; // Validation failed for REG_REFERENCE_C1
    }
    if (ReadReg(sensor_addr, REG_REFERENCE_C2, &value) != HAL_OK || value != 0x10)
    {
        return HAL_ERROR; // Validation failed for REG_REFERENCE_C2
    }

    // Step 2: Write default/reset values to other reference registers (if needed)
    if (WriteReg(sensor_addr, REG_REFERENCE_51, 0x00) != HAL_OK) // Reset 0x51 to default
    {
        return HAL_ERROR;
    }
    if (WriteReg(sensor_addr, REG_REFERENCE_61, 0x00) != HAL_OK) // Reset 0x61 to default
    {
        return HAL_ERROR;
    }

    // Step 3: Start measurement (default configuration)
    if (WriteReg(sensor_addr, 0x00, 0x01) != HAL_OK) // Start the measurement
    {
        return HAL_ERROR;
    }

    return HAL_OK; // Initialization successful
}

HAL_StatusTypeDef SetAddress(uint8_t NewAddress){
	if(WriteReg(CurrentAddress, 0x8A, NewAddress & 0x7F)!=HAL_OK){
		return HAL_ERROR;
	}
	else {
		return HAL_OK;
	}
	//HAL_Delay(10);
}

uint16_t readDistance(uint8_t deviceAddress) {
    uint8_t reg = 0x1E;          // RESULT_RANGE_STATUS register address
    uint8_t buffer[2] = {0};     // Buffer to store received data
    uint16_t distance = 0;       // Variable to hold the distance value

    // Trigger a new measurement
    uint8_t startMeasurementCmd = 0x01;  // Trigger new measurement
    	if (HAL_I2C_Mem_Write(&hi2c1, (deviceAddress << 1), 0x00, 1, &startMeasurementCmd, 1, HAL_MAX_DELAY) != HAL_OK) {
    		return 0xFFFF;  // Error
        }

    // Transmit the register address
    if (HAL_I2C_Master_Transmit(&hi2c1, (deviceAddress << 1), &reg, 1, HAL_MAX_DELAY) != HAL_OK) {
        // Handle error (e.g., return 0 or a specific error code)
        return 0xFFFF;  // Return 0xFFFF to indicate an error
    }

    // Receive the 2-byte distance data
    if (HAL_I2C_Master_Receive(&hi2c1, (deviceAddress << 1) | 1, buffer, 2, HAL_MAX_DELAY) != HAL_OK) {
        // Handle error (e.g., return 0 or a specific error code)
        return 0xFFFF;  // Return 0xFFFF to indicate an error
    }

    // Combine MSB and LSB to form the 16-bit distance value
    distance = (buffer[0] << 8) | buffer[1];

    return distance;
}
