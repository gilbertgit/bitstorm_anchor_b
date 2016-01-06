/*
 * task_ble_serial.h
 *
 *  Created on: Dec 17, 2015
 *      Author: titan
 */

#ifndef TASK_BLE_SERIAL_H_
#define TASK_BLE_SERIAL_H_

extern QueueHandle_t xBleQueue;
extern uint16_t xBleMonitorCounter;

void config_ble();
void hello_ble();

void task_ble_serial_start();
void ble_usart_tx(uint8_t data[], int size);

#endif /* TASK_BLE_SERIAL_H_ */
