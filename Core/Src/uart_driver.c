/*
 * uart_driver.c
 *
 *  Created on: Oct 4, 2023
 *      Author: gustavo
 */
#include "uart_driver.h"
#include "FreeRTOS.h"
#include "semphr.h"

// Declares a semaphore structure for the UART
SemaphoreHandle_t sem_uart;
QueueHandle_t queue_uart;

// Declares a mutex structure for the UART
//SemaphoreHandle_t mutex_uart_tx;

char uart_data;

int32_t uart_init(void)
{
	int32_t ret = 0;
	if (sem_uart == NULL) {
		sem_uart = xSemaphoreCreateBinary();
	}

	queue_uart = xQueueCreate(32, sizeof(char));

    if((sem_uart == NULL)  || (queue_uart == NULL)){
        ret = -1;
    }
#if 0
    else
    {
    	mutex_uart_tx = xSemaphoreCreateMutex();
        if( mutex_uart_tx == NULL ){
            vSemaphoreDelete( sem_uart);
            ret = -1;
        }
    }
#endif

    HAL_UART_Receive_IT(&huart1, (uint8_t *)&uart_data, 1);
    return ret;
}

size_t uart_write(uint8_t *string, size_t size){
    size_t nbytes = 0;
	if (HAL_UART_Transmit_IT(&huart1, string, size) == HAL_OK) {
		// Wait indefinitely the finish of the string transmission by the UART port
		xSemaphoreTake(sem_uart, portMAX_DELAY);
		nbytes = size;
	}

    return nbytes;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	signed portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(sem_uart, &pxHigherPriorityTaskWoken);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(queue_uart, &uart_data, &pxHigherPriorityTaskWoken);
	HAL_UART_Receive_IT(huart, (uint8_t *)&uart_data, 1);
	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

void uart_get_char(char *data, TickType_t timeout){
	xQueueReceive(queue_uart, data, timeout);
}


