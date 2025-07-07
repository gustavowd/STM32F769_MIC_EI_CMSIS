/*
 * edge_impulse.h
 *
 *  Created on: Oct 9, 2023
 *      Author: gustavo
 */

#ifndef INC_EDGE_IMPULSE_H_
#define INC_EDGE_IMPULSE_H_

#ifdef __cplusplus
extern "C" {
#endif

#define FEATURES_SIZE			48000

extern SemaphoreHandle_t sem_new_data;

void Task_edge_impulse(void *param);
int raw_feature_set_data(float *data, uint32_t len);

#ifdef __cplusplus
}
#endif


#endif /* INC_EDGE_IMPULSE_H_ */
