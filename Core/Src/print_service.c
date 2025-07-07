/*
 * print_service.c
 *
 *  Created on: Oct 6, 2023
 *      Author: gustavo
 */


#include "main.h"
#include "print_service.h"
#include "message_buffer.h"
#include "semphr.h"
#include "uart_driver.h"

MessageBufferHandle_t xMessageBuffer;
SemaphoreHandle_t print_service_mutex;

static void Print_Task(void *param){
	char buffer[768];
	uint8_t qtd = 0;
	while(1)
	{
		qtd = xMessageBufferReceive(xMessageBuffer, (void *)buffer, sizeof(buffer) - 1, portMAX_DELAY);
		if (uart_write((uint8_t *)buffer, qtd) != qtd) {
			uart_write((uint8_t *)"Print Error!\r\n", 14);
		}
	}
}


size_t print_service_send(char *string, size_t size, TickType_t timeout){
	// Descobre o tamanho da string, caso não informado
	size_t nbytes = 0;
	if (!size){
		char *tmp = string;

		while(*tmp++){
			size++;
		}
	}
    if (print_service_mutex != NULL){
		if (xSemaphoreTake(print_service_mutex, timeout) == pdTRUE){
			nbytes = xMessageBufferSend(xMessageBuffer, string, size, timeout);
			xSemaphoreGive(print_service_mutex);
		}
    }
    return nbytes;
}

BaseType_t print_service_init(UBaseType_t uxPriority){
	if (xMessageBuffer == NULL) {
		xMessageBuffer = xMessageBufferCreate(768);
	}

	if (print_service_mutex == NULL) {
		print_service_mutex = xSemaphoreCreateMutex();
	}
	int ret = uart_init();

	if ((xMessageBuffer == NULL) || (print_service_mutex == NULL) || (ret < 0)){
		return pdFALSE;
	}

	xTaskCreate(Print_Task, "Print Task", 512, NULL, uxPriority, NULL);
	return pdTRUE;
}
