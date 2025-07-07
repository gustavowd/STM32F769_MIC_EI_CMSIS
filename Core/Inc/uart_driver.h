/*
 * uart_driver.h
 *
 *  Created on: Oct 4, 2023
 *      Author: gustavo
 */

#ifndef INC_UART_DRIVER_H_
#define INC_UART_DRIVER_H_

#include <stdint.h>
#include <stddef.h>
#include "main.h"

extern UART_HandleTypeDef huart1;

int32_t uart_init(void);
size_t uart_write(uint8_t *string, size_t size);


#endif /* INC_UART_DRIVER_H_ */
