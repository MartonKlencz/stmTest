//
// Created by klenc on 01/02/2024.
//

#ifndef L476_DAC_STIMCONTROL_H
#define L476_DAC_STIMCONTROL_H



#include <stdio.h>
#include <stddef.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "globals.h"


/*
 *  extern functions and variables
 */

extern void Error_Handler(void);

extern DMA_HandleTypeDef hdma_tim2_up;

extern TIM_HandleTypeDef htim2;

extern DAC_HandleTypeDef hdac1;

extern SPIHandler spiHandler;


bool debugPin = 0;

//functions

/*
 *  offsets the data elements in the array by TIMING_PADDING / 2
 */
void offsetData(uint32_t* array);

/*
 * fills the provided array with the bell curve with the appropriate values, called in generateStimulationPattern()
 */
void fillWithBell(uint32_t* array, size_t size, double avg, double dev, double amp, double offset);


/*
 * performs the appropriate bit swaps needed to control the GPIOB (therefore the DAC)
 */
uint16_t swapBits(uint16_t data);


/*
 * parses user input from UART
 */
void parseUserInput();

/*
 * this calls fillWithBell and also generates the flat region of the stimulation pattern
 */
 void generateStimulationPattern();

/*
 * initializes the default wave timings
 */
void initializeStimparams();

void initializeTimerSettings();


/*
 * start DMA with the appropriate parameters
 */
void start_DMA();


void enablePanel(uint8_t address);

/*
 * this is called in DMA1_Channel2_IRQHandler(void) when a transfer complete interrupt is generated
 */
inline void handleStimulation();


#endif //L476_DAC_STIMCONTROL_H
