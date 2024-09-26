/*
 * pcf8574.c
 *
 *  Created on: Sep 25, 2024
 *      Author: A
 */
#include "main.h"
#include "pcf8574.h"

extern I2C_HandleTypeDef hi2c2;  // Replace with your I2C handle

uint8_t PCF8574_Init()
{
    uint8_t data = 0x81;  // 设置所有引脚为高电平（输入状态）

    // 将初始状态写入 PCF8574
    if (HAL_I2C_Master_Transmit(&hi2c2, PCF8574_WRITE_REG, &data, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}


uint8_t PCF8574_ReadPin(uint16_t GPIO_Pin, GPIO_PinState *PinState) {
    uint8_t data;
    if (HAL_I2C_Master_Receive(&hi2c2, PCF8574_READ_REG, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }
    *PinState = (data & (1 << GPIO_Pin)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    return HAL_OK;
}

uint8_t PCF8574_WritePin(uint16_t GPIO_Pin, GPIO_PinState PinState) {
    uint8_t data;

    // 先读取当前状态
    if (HAL_I2C_Master_Receive(&hi2c2, PCF8574_READ_REG, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    // 根据输入状态更新相应的引脚
    if (PinState == GPIO_PIN_SET) {
        data |= (1 << GPIO_Pin); 	// 设置引脚
    } else {
        data &= ~(1 << GPIO_Pin); 	// 清除引脚
    }

    // 发送新的状态
    if (HAL_I2C_Master_Transmit(&hi2c2, PCF8574_WRITE_REG, &data, 1, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}


