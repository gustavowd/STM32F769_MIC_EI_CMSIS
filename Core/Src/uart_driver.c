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

// Declares a mutex structure for the UART
//SemaphoreHandle_t mutex_uart_tx;

int32_t uart_init(void)
{
	int32_t ret = 0;
	if (sem_uart == NULL) {
		sem_uart = xSemaphoreCreateBinary();
	}

    if( sem_uart == NULL ){
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
    return ret;
}

size_t uart_write(uint8_t *string, size_t size){
    size_t nbytes = 0;
    //if (mutex_uart_tx != NULL){
		//if (xSemaphoreTake(mutex_uart_tx, portMAX_DELAY) == pdTRUE){
			if (HAL_UART_Transmit_IT(&huart1, string, size) == HAL_OK) {
				// Wait indefinitely the finish of the string transmission by the UART port
				xSemaphoreTake(sem_uart, portMAX_DELAY);
				nbytes = size;

				//xSemaphoreGive(mutex_uart_tx);
			}
		//}
	//}

    return nbytes;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	signed portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(sem_uart, &pxHigherPriorityTaskWoken);
}

