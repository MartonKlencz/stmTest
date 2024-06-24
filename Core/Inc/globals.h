//
// Created by klenc on 01/02/2024.
//

#ifndef L476_DAC_GLOBALS_H
#define L476_DAC_GLOBALS_H

#define RX_SIZE 8

#define STIMCHANNELORDERSIZE 30

#define TIMING_PADDING 4 //this value defines the 2047 value padding of the data arrays (when using very fast clocks, the DMA interrupt causes headaches)
#define NS (128 + TIMING_PADDING) //64 for data and 64 for control (these are interlaced)
#define TIMING_PROGRAM_LENGTH 100
#define DEFAULT_TIMING 3125

enum WaveState {upBump, shortFlat, downBump, delay2, longFlat};

typedef struct
{
	bool enableTransit;
	uint16_t data[1];
} SPIHandler;

typedef struct
{
    // A wave duration default value: 250 us
    // (1/80e6) * 64 * 312.5 = 0.00025s = 250 us
    // (1/(CLK) * NS * ARR = duration) -> ARR = duration * CLK/NS
    // where CLK = 80e6, NS = 64
    // going below ARR = 64 may cause problems, however it is possible //TODO - outdated

    //these are set into TIM2 during the DMA interrupt calls (see stm32l4xx_it.c)

    uint16_t A_ARR[TIMING_PROGRAM_LENGTH];
    uint16_t B_ARR[TIMING_PROGRAM_LENGTH];
    uint16_t C_ARR[TIMING_PROGRAM_LENGTH];
    uint16_t D_ARR[TIMING_PROGRAM_LENGTH];

} TimerSettingHandle;


typedef struct
{
    //enables stimulation
    bool enableStimulation;

    //if set to true, the whole sequence is repeated continuously
    bool continuousStimulation;

    //number of waves in a burst
    uint16_t maxWaveNumber;

    //this is in units of timeBetweenBursts * D_ARR * CLK / NS [us] !!I'm not sure that this is true, seems wrong to me!!
    uint16_t timeBetweenBursts;

    //holds the amount of repeats for an index in the stimulation sequence
    uint8_t channelRepeatNumber[TIMING_PROGRAM_LENGTH];

    //holds the number of repeats for the current index in the stimulation sequence, reset if larger than channelRepeatNumber[channelIndex]
    uint16_t currentChannelRepeatIndex;

    //TODO - find out what this is
    //probably the time that defines how often checks should be for the switching (should not switch during a wave)
    uint16_t channelSwitchTime;

    //the current active channel (in theory, needs testing //TODO)
    uint8_t channelIndex;

    //this array holds the order in which the channels should open and close
    //more specifically the 16 bit value that is sent to the MAX14802 (now its a different mux)
    uint16_t channelOrder[TIMING_PROGRAM_LENGTH];

    //this holds the amount of channels that were set by the user, should not be set above STIMCHANNELORDERSIZE
    uint8_t validIndeces;

    //this holds the individual intensity values for each channel in the order
    uint16_t channelOrderIntensity[TIMING_PROGRAM_LENGTH];

    //holds the current amount of waves in the current index
    uint16_t burstCounter;

    //holds the current pause index when burstCounter is zero
    uint16_t burstPauseIndex;

    enum WaveState currentWaveState;

    uint8_t shortFlatNS;

    bool skipDelay;

    bool panelSwitchRequest;

    uint8_t panelSwitchAddress;

    float intensityMultiplier;

} StimulationControllerHandle;




#endif //L476_DAC_GLOBALS_H
