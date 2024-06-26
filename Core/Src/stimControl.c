//
// Created by klenc on 01/02/2024.
//

#include "stimControl.h"


/*
 * variables
 */



const double dev = 20.0; //deviation of bell curves

uint32_t A_wave[NS];
uint32_t C_wave[NS];
uint32_t Line_LUT[NS];
uint32_t Dummy_Cycle[2];


TimerSettingHandle timerSetting;

StimulationControllerHandle stimulationController;

void initializeTimerSettings()
{
    timerSetting.A_ARR[0] = 312;
    timerSetting.B_ARR[0] = 312;
    timerSetting.C_ARR[0] = 312;
    timerSetting.D_ARR[0] = 3125;

}

void initializeStimparams()
{
    stimulationController.enableStimulation = false;

    stimulationController.continuousStimulation = false;

    stimulationController.maxWaveNumber = 10;

    stimulationController.timeBetweenBursts = 50;

    stimulationController.channelSwitchTime = 4999;

    stimulationController.burstCounter = stimulationController.maxWaveNumber;

    stimulationController.burstPauseIndex = 0;

    stimulationController.currentChannelRepeatIndex = 0;

    stimulationController.currentWaveState = longFlat;

    stimulationController.shortFlatNS = NS;

    stimulationController.skipDelay = false;

    stimulationController.panelSwitchRequest = false;

    stimulationController.panelSwitchAddress = 0x80;

    stimulationController.intensityMultiplier = 1;

    uint16_t dummyData = swapBits(2047);
    Dummy_Cycle[0] = (~dummyData) << 16 | dummyData;
    Dummy_Cycle[1] = swapBits(0x9000);
}


void offsetData(uint32_t* array)
{
    uint32_t copyArray[NS];

    for (uint16_t i = 0; i < NS; i++)
    {
        copyArray[i] = array[i];
    }

    for (uint16_t i = TIMING_PADDING / 2; i < NS - TIMING_PADDING / 2; i++)
    {
        array[i] = copyArray[i - TIMING_PADDING / 2];
    }
}

void fillWithBell(uint32_t* array, size_t size, double avg, double dev, double amp, double offset)
{

    //TODO IMPORTANT check NS!!!!!
    int16_t Wave[NS / 2] = {
            60, 82, 112, 150, 199, 261, 339, 437, 556, 702, 876, 1082, 1324, 1604,
            1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800,
            1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800,
            1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800, 1604, 1324, 1082, 876, 702, 556,
            437, 339, 261, 199, 150, 112, 82, 60,
    };




    for (int i = 0; i + 1 < size; i += 2)
    {

        uint16_t gpioValue;

        //generate bell curve
        //gpioValue = amp * 1800 * exp(-((i - avg)*(i - avg)/(2*dev*dev))) + offset;

        gpioValue = amp * Wave[i / 2] + offset;
        if (gpioValue > 4095)
        {
            gpioValue = 4095;
        }
        if (gpioValue < 0)
        {
            gpioValue = 0;
        }

        //CS (bit #12) - PB5
        //LDAC (bit #13) - PB7
        //A/B (bit #14) - PB0
        //R/W (bit #15) - PB8

        //set control bits to 0, leave data bits as is
        gpioValue &= 0b0000111111111111;


        //swap bits according to the actual PCB routing
        gpioValue = swapBits(gpioValue);

        //1 in upper 16 bits: drive the corresponding pin low
        //1 in lower 16 bits: drive the corresponding bit high
        //0 anywhere: no change
        array[i] = (~gpioValue) << 16 | gpioValue;


        //array[i + 1] = (uint32_t) ((1 << 14) | (1 << 13)); //TODO, redo this - redone, have to test

        // set CS, R/W, A/B high (0b1001 0000 0000 0000)
        gpioValue = swapBits(0x9000);
        array[i + 1] =  gpioValue;
    }
}


uint16_t swapBits(uint16_t data)
{

    uint16_t result = 0;
    //unused
    //uint8_t pinOrder[16] = {14, 5, 4, 11, 8, 12, 2, 13, 15, 0, 7, 3, 1, 10, 9, 6};

    //this maps the index of a bit to the corresponding pin on the PCB
    // (i.e. bit 0 should be sent on pin PB9; bit 1 on pin PB12... etc.)

    /* single channel EEL:
     * uint8_t shiftTo[16] = {9, 12, 6, 11, 2, 1, 15, 10,
                           4, 14, 13, 3, 5, 7, 0, 8};*/

    // this is the pin configuration for the SMALL EEL PCB:
    uint8_t shiftTo[16] = {2, 11, 10, 12, 13, 14, 15, 9, 8, 7, 6, 3, 1, 5, 4, 0};

    //shift every bit to its place
    result |= (data & 1) << shiftTo[0];
    for (uint8_t i = 1; i < 16; i++)
    {
        data = data >> 1;
        result |= (((data & 1) > 0) << shiftTo[i]);
    }

    return result;
}


void generateStimulationPattern()
{
    fillWithBell(A_wave, NS, NS / 2.0, dev, 1, 2047);
    fillWithBell(C_wave, NS, NS / 2.0, dev, -1, 2047);

    for (int i = 0; i + 1 < NS; i+=2)
    {
        uint16_t gpioValue;


        // set CS, R/W, A/B high (0b1001 0000 0000 0000)
        gpioValue = swapBits(2047);
        Line_LUT[i] = (~gpioValue) << 16 | gpioValue; // set everything to 2047 (+ first 4 bits are control bits)

        gpioValue = swapBits(0x9000);
        Line_LUT[i + 1] = gpioValue;
    }

    offsetData(A_wave);
    offsetData(C_wave);

    //perform padding //TODO - shift the first n data points maybe? (because first elements also get padded over)
    uint16_t gpioValue;
    gpioValue = swapBits(2047);

    for (int i = 0; i < TIMING_PADDING / 2; i++)
    {
        A_wave[2 * i] = (~gpioValue) << 16 | gpioValue;
        A_wave[2 * i + 1] = swapBits(0x9000);

        A_wave[NS - 2 - 2 * i] = (~gpioValue) << 16 | gpioValue;
        A_wave[NS - 1 - 2 * i] = swapBits(0x9000);

        C_wave[2 * i] = (~gpioValue) << 16 | gpioValue;
        C_wave[2 * i + 1] = swapBits(0x9000);

        C_wave[NS - 2 - 2 * i] = (~gpioValue) << 16 | gpioValue;
        C_wave[NS - 1 - 2 * i] = swapBits(0x9000);
    }
}

void start_DMA()
{
    if (HAL_DMA_Start_IT(&hdma_tim2_up, (uint32_t) Line_LUT, (uint32_t)&(GPIOB->BSRR), NS) != HAL_OK)
    {
        Error_Handler();
    }
}

void enablePanel(uint8_t address)
{
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, address & 0x01);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, address & 0x02);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, address & 0x04);
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, address & 0x08);
//
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, address & 0x80);
}


void handleStimulation()
{
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, debugPin);
//    debugPin = !debugPin;
    //enableStimulation is controlled by user commands from UART
    if (!stimulationController.enableStimulation)
    {
    	GPIOC->BRR = (uint32_t) GPIO_PIN_4; //sync signal -> low
        return;
    }

    //burstCounter counts the number of bursts remaining after the enableStimulation flag is set to true
    //if it is 0, we skip timeBetweenBursts * D_ARR / 1.25 microseconds of stimulation
    if (stimulationController.burstCounter == 0)
    {
        if (stimulationController.burstPauseIndex < stimulationController.timeBetweenBursts) // == it's pausing time
        {
            htim2.Instance->ARR = timerSetting.D_ARR[stimulationController.channelIndex];

            stimulationController.currentWaveState = longFlat;
            stimulationController.burstPauseIndex++;

            if ((stimulationController.timeBetweenBursts - stimulationController.burstPauseIndex)
            		* timerSetting.D_ARR[stimulationController.channelIndex] * NS < 400000)
            {
            	GPIOC->BSRR = (uint32_t) GPIO_PIN_4;
            } else
            {
            	GPIOC->BRR = (uint32_t) GPIO_PIN_4;
            }

            return; //return, as we have nothing else to do
        } else //pause over, start a burst of waves
        {
            //reset pause counter
            stimulationController.burstPauseIndex = 0;

            stimulationController.currentWaveState = upBump;

            //set TIM2 clock to frequency needed for the A wave
            htim2.Instance->ARR = timerSetting.A_ARR[stimulationController.channelIndex];


            stimulationController.burstCounter = stimulationController.maxWaveNumber;

            //this counts the number of times that a channel should be repeated for (is held in stimulationController.channelRepeatNumber[channelIndex])
            stimulationController.currentChannelRepeatIndex++;
            CLEAR_BIT(DMA1_Channel2->CCR, 1);
            DMA1_Channel2->CNDTR = NS;
            SET_BIT(DMA1_Channel2->CCR, 1);

            if (stimulationController.currentChannelRepeatIndex >= stimulationController.channelRepeatNumber[stimulationController.channelIndex])
            {

                stimulationController.currentChannelRepeatIndex = 0;

                //go to next channel
                stimulationController.channelIndex++;
                if (stimulationController.channelIndex >= stimulationController.validIndeces)
                {
                    stimulationController.channelIndex = 0;

                    if (!stimulationController.continuousStimulation) {

                        //this means that only one stimulation was requested
                        stimulationController.enableStimulation = 0;

                        stimulationInfoCommand = 0x03;

                        CLEAR_BIT(DMA1_Channel2->CCR, 1);
                        DMA1_Channel2->CMAR = (uint32_t) &Line_LUT;
                        TIM2->ARR = DEFAULT_TIMING;
                        DMA1_Channel2->CNDTR = NS;
                        SET_BIT(DMA1_Channel2->CCR, 1);

                    	GPIOC->BRR = (uint32_t) GPIO_PIN_4;


                        stimulationController.burstCounter = stimulationController.maxWaveNumber;
                        stimulationController.burstPauseIndex = 0;
                        return;
                    }
                }
                GPIOC->BSRR = (uint32_t) GPIO_PIN_6;
                //enablePanel(stimulationController.channelOrder[stimulationController.channelIndex]);
                spiHandler.data[0] = stimulationController.channelOrder[stimulationController.channelIndex];
                spiHandler.enableTransit = true;

                stimulationInfoCommand = 0x04;
                GPIOC->BRR = (uint32_t) GPIO_PIN_6;

                hdac1.Instance->DHR12R1 = (uint16_t)  (stimulationController.intensityMultiplier *
                        (float) (stimulationController.channelOrderIntensity[stimulationController.channelIndex] & 0x0fff));
            }
        }
    }
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, debugPin);
//    debugPin = !debugPin;

    switch (stimulationController.currentWaveState)
    {
        case upBump:

            //disable channel
            CLEAR_BIT(DMA1_Channel2->CCR, 1);
            //update array pointer
            DMA1_Channel2->CMAR = (uint32_t) &A_wave;
            //set timer setting
            TIM2->ARR = timerSetting.A_ARR[stimulationController.channelIndex];
            //update DMA counter
            DMA1_Channel2->CNDTR = NS;
            //enable channel
            SET_BIT(DMA1_Channel2->CCR, 1);
            break;

        case shortFlat:

            CLEAR_BIT(DMA1_Channel2->CCR, 1);
            DMA1_Channel2->CMAR = (uint32_t) &Line_LUT;
            TIM2->ARR = timerSetting.B_ARR[stimulationController.channelIndex];
            DMA1_Channel2->CNDTR = stimulationController.shortFlatNS;
            SET_BIT(DMA1_Channel2->CCR, 1);

            break;

        case downBump:

            CLEAR_BIT(DMA1_Channel2->CCR, 1);
            DMA1_Channel2->CMAR = (uint32_t) &C_wave;
            TIM2->ARR = timerSetting.C_ARR[stimulationController.channelIndex];

            DMA1_Channel2->CNDTR = NS;
            SET_BIT(DMA1_Channel2->CCR, 1);

            break;

        case delay2:

            CLEAR_BIT(DMA1_Channel2->CCR, 1);
            DMA1_Channel2->CNDTR = 2;
            DMA1_Channel2->CMAR = (uint32_t) &Dummy_Cycle;
            TIM2->ARR = 10;
            SET_BIT(DMA1_Channel2->CCR, 1);

            break;

        case longFlat:

            CLEAR_BIT(DMA1_Channel2->CCR, 1);
            DMA1_Channel2->CMAR = (uint32_t) &Line_LUT;
            TIM2->ARR = timerSetting.D_ARR[stimulationController.channelIndex];
            DMA1_Channel2->CNDTR = NS;
            SET_BIT(DMA1_Channel2->CCR, 1);

//            if (stimulationController.panelSwitchRequest)
//            {
//                stimulationController.panelSwitchRequest = false;
//                //enablePanel(stimulationController.panelSwitchAddress);
//            }


            break;
    }

    stimulationController.currentWaveState++;
    if (stimulationController.currentWaveState > longFlat)
    {
        stimulationController.burstCounter--;
        stimulationController.currentWaveState = upBump;
    }

}
