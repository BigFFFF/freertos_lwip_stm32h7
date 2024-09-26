/*
 * pcf8574.h
 *
 *  Created on: Sep 25, 2024
 *      Author: A
 */

#ifndef BSP_COMPONENTS_PCF8574_PCF8574_H_
#define BSP_COMPONENTS_PCF8574_PCF8574_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

#define PCF8574_ADDRESS 	0x20
#define PCF8574_WRITE_REG  	(PCF8574_ADDRESS << 1)
#define PCF8574_READ_REG   	((PCF8574_ADDRESS << 1) | 0x01)
#define PCF8574_IO_NUM		8

/* PCF8574各个IO的功能 */
#define BEEP_IO         	0		// 蜂鸣器控制引脚
#define AP_INT_IO       	1   	// AP3216C中断引脚
#define DCMI_PWDN_IO    	2    	// DCMI的电源控制引脚
#define USB_PWR_IO      	3    	// USB电源控制引脚
#define EX_IO      			4    	// 扩展IO,自定义使用
#define MPU_INT_IO      	5   	// MPU9250中断引脚
#define RS485_RE_IO     	6    	// RS485_RE引脚
#define ETH_RESET_IO    	7    	// 以太网复位引脚

/* PCF8574_Exported_Functions */
uint8_t PCF8574_Init();
uint8_t PCF8574_ReadPin(uint16_t GPIO_Pin, GPIO_PinState *PinState);
uint8_t PCF8574_WritePin(uint16_t GPIO_Pin, GPIO_PinState PinState);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif
#endif /* BSP_COMPONENTS_PCF8574_PCF8574_H_ */
