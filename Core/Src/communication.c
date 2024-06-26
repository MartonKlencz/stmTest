/*
 * communication.c
 *
 *  Created on: Jan 22, 2024
 *      Author: klenc
 */
#include "communication.h"


bool checkCRC(uint8_t * data, uint16_t size)
{
    uint8_t sum = 0;
    for (int i = 0; i < size - 1; i++)
    {
        sum += data[i];
    }
    return sum == data[size - 1];
}

void parseUserInput()
{

	switch (Rx1_data[0])
	{

        case 0x00: //system reset signal
            //printf("system reset...\r\n");
            NVIC_SystemReset();
            break;

        case 0x01: //disable stimulation
            stimulationController.enableStimulation = false;
            //printf("stimulation disabled\r\n");
            //set default timer setting
            htim2.Instance->ARR = DEFAULT_TIMING;
            hdma_tim2_up.Instance->CMAR = (uint32_t) &Line_LUT;

            CLEAR_BIT(hdma_tim2_up.Instance->CCR, 0);
            hdma_tim2_up.Instance->CNDTR = NS;
            SET_BIT(hdma_tim2_up.Instance->CCR, 0);

            stimulationController.burstCounter = stimulationController.maxWaveNumber;
            stimulationController.burstPauseIndex = 0;

            stimulationInfoCommand = 0x03;

            break;

        case 0x02: //enable stimulation

            if (stimulationController.validIndeces > 0)
            {
                stimulationController.channelIndex = 0;

                //hdac1.Instance->DHR12R1 = (uint16_t)  (stimulationController.intensityMultiplier *
                //                                     (float) (stimulationController.channelOrderIntensity[stimulationController.channelIndex] & 0x0fff));

                //stimulationController.currentWaveState = upBump;

                stimulationController.enableStimulation = true;

                stimulationController.burstCounter = 0;
                stimulationController.burstPauseIndex = stimulationController.timeBetweenBursts - //this hack is here to ensure that the first stimulation spike can be shown by the sync signal
                		(uint16_t) ((400000.0) / (float) (timerSetting.D_ARR[0] * NS));

                stimulationController.continuousStimulation = Rx1_data[1] > 0x7f;

                if (stimulationController.continuousStimulation)
                {
                    //printf("continuous stimulation enabled\r\n");
                } else
                {
                    //printf("stimulating once\r\n");
                }
            }

            break;

        case 0x05: //'A' A wave clock frequency

            timerSetting.A_ARR[Rx1_data[3]] = (Rx1_data[1] << 8) | Rx1_data[2];

            //printf("A_ARR: %d\r\n", timerSetting.A_ARR);
            break;

        case 0x06: //'B' B line clock frequency

            // changed to: delay in microseconds

            ; //NS * (ARR) / 80MHz = t [us] (here, t is called 'delay')
            uint16_t delay = (Rx1_data[1] << 8) | Rx1_data[2];

            uint16_t ARR_value = (uint16_t) (80.0 * (double) delay / (double) NS);


            if (ARR_value <= 20)
            {
                ARR_value = 20;
                stimulationController.shortFlatNS = (uint16_t) (2 * delay); //NS = 80 MHz * t / ARR
                //if (stimulationController.shortFlatNS)
                stimulationController.skipDelay = true;
            } else
            {
                stimulationController.shortFlatNS = NS;

                stimulationController.skipDelay = false;
            }

            timerSetting.B_ARR[Rx1_data[3]] = ARR_value;

            break;

        case 0x07: //'C' C wave clock frequency
            timerSetting.C_ARR[Rx1_data[3]] = (Rx1_data[1] << 8) | Rx1_data[2];
            break;

        case 0x08: //'D' D wave clock frequency
            timerSetting.D_ARR[Rx1_data[3]] = (Rx1_data[1] << 8) | Rx1_data[2];
            break;

        case 0x09: //'N' number of waves in a burst
            stimulationController.maxWaveNumber = (Rx1_data[1] << 8) | Rx1_data[2];
            break;

        case 0x0a: //this was used for relay opening/closing, now it acts as GPIO control
            ;
            uint16_t tmp = Rx1_data[1] << 8 | Rx1_data[2];

            GPIOC->BSRR = (uint32_t) ((uint32_t) (~tmp) << 16 | tmp);
            break;

        case 0x0b: //before: 'm', default value: 50, sets the pause time between bursts
            stimulationController.timeBetweenBursts = (Rx1_data[1] << 8) | Rx1_data[2];
            break;

        case 0x0c: //PWM duty cycle

//            TIM3->CCR1 = (uint32_t) ((float) Rx_data[1] * TIM3->ARR/ 100);
            break;

        case 0x0d: //before: 'S', manual opening of analog switch, empty for now
// spi transmit data (for analog switch)
//            if (tmpValue == 17)
//            {
//                break;
//            }
//            if (tmpValue > 7)
//            {
//                SPIdata[0] = 1<<(tmpValue-8);
//                SPIdata[1] = 0;
//            } else
//            {
//                SPIdata[0] = 0;
//                SPIdata[1] = 1<<tmpValue;
//            }
//            enableSPITransmit = 1;

            break;
        case 0x0e: //before: 's', sets the order of the channels, have to redo
            ;
            uint8_t channelOrderIndex = Rx1_data[1]; //which index in the programorder

            uint16_t channelOrderIntensity = Rx1_data[2] << 8 | Rx1_data[3]; //what intensity

            uint16_t stimChannelToSet = Rx1_data[4] << 8 | Rx1_data[5]; //which channel

            uint8_t stimChannelNumberOfBurstsToSet = Rx1_data[6]; //number of bursts

            //verify channel indexes so that no short happens between the opamp an gnd
            if (((stimChannelToSet & 0x5555) & (stimChannelToSet >> 1)) == 0)
            {
				stimulationController.channelOrder[channelOrderIndex] = stimChannelToSet;

				stimulationController.channelOrderIntensity[channelOrderIndex] = channelOrderIntensity;

				stimulationController.channelRepeatNumber[channelOrderIndex] = stimChannelNumberOfBurstsToSet;
            }
            else
            {
            	uint8_t errorMSG[2] = {0xbb, 0xbb};
            	HAL_UART_Transmit(&huart2,  errorMSG, 2, 100);
            }


            break;
        case 0x0f: //before: 'v', number of valid indices in the channel ordering

            stimulationController.validIndeces = Rx1_data[1];

            break;

        case 0x10: //before: 'R', this was used to manually set the base scaler for the digital potentiometer
            //TODO
            ////I2Cdata[0] = tmpValue;
            ////enableI2Ctransmit = 1;
            //baseIntensity = tmpValue / 127.0f;
            break;

        case 0x11: //before: 'p', sets PWM frequency, this keeps the duty cycle the same
            ;
//            float prevDuty = (float) htim3.Instance->CCR1 / (float) htim3.Instance->ARR;
//
//            uint32_t tmp_ARR = (Rx_data[1] << 24) | (Rx_data[2] << 16) | (Rx_data[3] << 8) | Rx_data[4];
//
//            TIM3->ARR = tmp_ARR;
//            TIM3->CCR1 = (uint32_t) (prevDuty * (float) tmp_ARR);
            break;
        case 0x12:
        	//unused
//            if (stimulationController.enableStimulation)
//            {
//                stimulationController.panelSwitchAddress = Rx_data[1];
//                stimulationController.panelSwitchRequest = true;
//            } else
//            {
//                //enablePanel(Rx_data[1]);
//            }

            break;
        case 0x13:
            ;
            uint16_t data = Rx1_data[1];

            //hdac1.Instance->DHR12R1 = dacData & 0x0fff;

            stimulationController.intensityMultiplier = (float) data / 127.0f;
            break;
        case 0x14:
        	//debug purposes for now
        	stimulationInfoCommand = Rx1_data[1];
        	break;
        case 0x15:
        	if (Rx1_data[1] > 127)
        	{
        		GPIOC->BSRR = (uint32_t) GPIO_PIN_7;
        		GPIOC->BSRR = (uint32_t) GPIO_PIN_9;
        	} else
        	{

        		GPIOC->BRR = (uint32_t) GPIO_PIN_7;
        		GPIOC->BRR = (uint32_t) GPIO_PIN_9;
        	}
        	break;

        case 0x16:
        	HAL_UART_Transmit(&huart2,  Rx1_data, 8, 100);
        	break;
        default:
            //printf("unknown command, disabling stimulation\r\n");

            stimulationController.enableStimulation = false;
            htim2.Instance->ARR = DEFAULT_TIMING;
            hdma_tim2_up.Instance->CMAR = (uint32_t) &Line_LUT;

            CLEAR_BIT(hdma_tim2_up.Instance->CCR, 0);
            hdma_tim2_up.Instance->CNDTR = NS;
            SET_BIT(hdma_tim2_up.Instance->CCR, 0);

            stimulationController.burstCounter = stimulationController.maxWaveNumber;
            stimulationController.burstPauseIndex = 0;

            stimulationInfoCommand = 0x03;

            break;
	}
}

void sendStimProgramInfo(uint8_t command)
{

	uint8_t data[7] = {0x69};
	uint8_t size = 1; //holds the number of valid data bytes (excluding the CHKSUM, but including the header)
	switch (command)
	{
	case 0x02: //START STIMULATION
		data[1] = 0x02;
		size = 2;
		break;

	case 0x03: //STOP STIMULATION
		data[1] = 0x03;
		size = 2;
		break;

	case 0x04: //LIVE STIMULATION UPDATE
		data[1] = 0x04;
		data[2] = (uint8_t) (stimulationController.channelOrder[stimulationController.channelIndex] >> 8);
		data[3] = (uint8_t) (stimulationController.channelOrder[stimulationController.channelIndex] & 0xff);

		uint16_t amplitude = (uint16_t)  (stimulationController.intensityMultiplier *
                (float) (stimulationController.channelOrderIntensity[stimulationController.channelIndex] & 0x0fff));
		data[4] = (uint8_t) (amplitude >> 8);
		data[5] = (uint8_t) (amplitude & 0xff);

		size = 6;
		break;
	}

	//append CHKSUM
	uint8_t sum = 0;
	for (int i = 0; i < size + 1; i++)
	{
		sum += data[i];
	}

	data[size] = sum;

	HAL_UART_Transmit(&huart2, data, size + 1, 100);
}


void parseBatteryInfo()
{
	HAL_UART_Transmit(&huart1, Rx2_data, RX2_SIZE, 100);
}



void UART_handleTimeOut()
{
    //restart UART reception
    HAL_UART_Abort_IT(&huart1);
    HAL_UART_Receive_IT(&huart1, Rx1_data, RX1_SIZE);
    HAL_UART_Abort_IT(&huart2);
    HAL_UART_Receive_IT(&huart2, Rx2_data, RX2_SIZE);
    UART_timeOutOccured = false;

    //printf("UART timed out, resetting...\r\n");

    //clear interrupt bits, disable interrupt
    WRITE_REG(huart1.Instance->ICR, USART_ICR_RTOCF);
    CLEAR_BIT(huart1.Instance->CR2, USART_CR2_RTOEN);
    CLEAR_BIT(huart1.Instance->CR1, USART_CR1_RTOIE);

    WRITE_REG(huart2.Instance->ICR, USART_ICR_RTOCF);
    CLEAR_BIT(huart2.Instance->CR2, USART_CR2_RTOEN);
    CLEAR_BIT(huart2.Instance->CR1, USART_CR1_RTOIE);
}
