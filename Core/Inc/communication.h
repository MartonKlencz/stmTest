/*
 * communication.h
 *
 *  Created on: Jan 22, 2024
 *      Author: klenc
 */

#ifndef INC_COMMUNICATION_H_
#define INC_COMMUNICATION_H_


#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <stdbool.h>
#include "globals.h"

//extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;
extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_tim2_up;
extern SPI_HandleTypeDef hspi1;
extern TimerSettingHandle timerSetting;

extern uint32_t Line_LUT[NS];

extern StimulationControllerHandle stimulationController;
extern volatile void enablePanel(uint8_t address);

//***** private variables

bool startParseInput = false;

bool UART_timeOutOccured;


//data array from UART
uint8_t Rx_data[RX_SIZE];

//spi handler for switch control
SPIHandler spiHandler;


//this is in units of timeBetweenBursts * D_ARR * CLK / NS [us]
extern uint16_t timeBetweenBursts;

uint8_t SPIdata[2] = {0b00000000, 0b00000000};



uint8_t enableSPITransmit = 0;



//***** functions
void parseUserInput();

bool checkCRC();

void UART_handleTimeOut();


#endif /* INC_COMMUNICATION_H_ */
