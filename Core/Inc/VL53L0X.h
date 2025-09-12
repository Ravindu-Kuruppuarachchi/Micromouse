#ifndef VL53L0X_H_
#define VL53L0X_H_

#include "main.h"

extern I2C_HandleTypeDef hi2c1;

HAL_StatusTypeDef writeReg(uint8_t deviceAddress, uint8_t reg, uint8_t value);
HAL_StatusTypeDef ReadReg(uint8_t deviceAddress, uint8_t reg, uint8_t *value);
HAL_StatusTypeDef SetTimingBudget(uint8_t sensor_addr, uint32_t timing_budget_us);
HAL_StatusTypeDef VL53L0X_Init(uint8_t sensor_addr);
HAL_StatusTypeDef SetAddress(uint8_t NewAddress);
uint16_t readDistance(uint8_t deviceAddress);

#endif /* VL53L0X_H_ */
