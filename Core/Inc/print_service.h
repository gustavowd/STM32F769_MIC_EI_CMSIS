/*
 * print_service.h
 *
 *  Created on: Oct 6, 2023
 *      Author: gustavo
 */

#ifndef INC_PRINT_SERVICE_H_
#define INC_PRINT_SERVICE_H_

#include <stdint.h>
#include <stddef.h>
#include "FreeRTOS.h"

BaseType_t print_service_init(UBaseType_t uxPriority);
size_t print_service_send(char *string, size_t size, TickType_t timeout);


#endif /* INC_PRINT_SERVICE_H_ */
