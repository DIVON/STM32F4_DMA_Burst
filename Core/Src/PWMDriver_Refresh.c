/*
 * file: PWMDriver_Refresh.c
 *
 * Implementation for PWMDriver_Refresh
 *
 * $Author: $
 * $Date: $
 * $Revision: $
 *
 * Magnetic Active Suspension System.
 */

/* INCLUDES */

#include "PWMDriver.h"

/* END OF INCLUDES */


/* MACROS */



/* END OF MACROS */


/* TYPE DEFINITIONS */


/* END OF TYPE DEFINITIONS */


/* VARIABLES */

/* Second buffer shall be second due to First buffer is in use right after starting of PWM and Scheduler timers */
static enPwmBufferIndex currentBufferIndex = pbiSECOND;
static volatile cdtPwmBuffer     pwmBuffers[pbiBUFFERS_COUNT];

/* END OF VARIABLES */


/* LOCAL FUNCTION DECLARATIONS */

/* END OF LOCAL FUNCTION DECLARATIONS */


/* LOCAL FUNCTION DEFINITIONS */



/* END OF LOCAL FUNCTION DEFINITIONS */


/* GLOBAL FUNCTION DEFINITIONS */

void WriteSpaceVectorToPwmBuffer(enPwmBufferIndex bufferIndex, uint8 step, enSpaceVectorDirection vector, uint32 setTime)
{
    pwmBuffers[bufferIndex].step[step].setTime = setTime;

    switch (vector)
    {
        case SV_ZP:
        {
            pwmBuffers[bufferIndex].step[step].phaseUstate = PWM_PULL_UP;
            pwmBuffers[bufferIndex].step[step].phaseVstate = PWM_PULL_UP;
            pwmBuffers[bufferIndex].step[step].phaseWstate = PWM_PULL_UP;
            break;
        }
        case SV_ZM:
        {
            pwmBuffers[bufferIndex].step[step].phaseUstate = PWM_PULL_DOWN;
            pwmBuffers[bufferIndex].step[step].phaseVstate = PWM_PULL_DOWN;
            pwmBuffers[bufferIndex].step[step].phaseWstate = PWM_PULL_DOWN;
            break;
        }
        case SV_UP:
        {
            pwmBuffers[bufferIndex].step[step].phaseUstate = PWM_PULL_UP;
            pwmBuffers[bufferIndex].step[step].phaseVstate = PWM_PULL_DOWN;
            pwmBuffers[bufferIndex].step[step].phaseWstate = PWM_PULL_DOWN;
            break;
        }
        case SV_UM:
        {
            pwmBuffers[bufferIndex].step[step].phaseUstate = PWM_PULL_DOWN;
            pwmBuffers[bufferIndex].step[step].phaseVstate = PWM_PULL_UP;
            pwmBuffers[bufferIndex].step[step].phaseWstate = PWM_PULL_UP;
            break;
        }
        case SV_VP:
        {
            pwmBuffers[bufferIndex].step[step].phaseUstate = PWM_PULL_DOWN;
            pwmBuffers[bufferIndex].step[step].phaseVstate = PWM_PULL_UP;
            pwmBuffers[bufferIndex].step[step].phaseWstate = PWM_PULL_DOWN;
            break;
        }
        case SV_VM:
        {
            pwmBuffers[bufferIndex].step[step].phaseUstate = PWM_PULL_UP;
            pwmBuffers[bufferIndex].step[step].phaseVstate = PWM_PULL_DOWN;
            pwmBuffers[bufferIndex].step[step].phaseWstate = PWM_PULL_UP;
            break;
        }
        case SV_WP:
        {
            pwmBuffers[bufferIndex].step[step].phaseUstate = PWM_PULL_DOWN;
            pwmBuffers[bufferIndex].step[step].phaseVstate = PWM_PULL_DOWN;
            pwmBuffers[bufferIndex].step[step].phaseWstate = PWM_PULL_UP;
            break;
        }
        case SV_WM:
        {
            pwmBuffers[bufferIndex].step[step].phaseUstate = PWM_PULL_UP;
            pwmBuffers[bufferIndex].step[step].phaseVstate = PWM_PULL_UP;
            pwmBuffers[bufferIndex].step[step].phaseVstate = PWM_PULL_DOWN;
            break;
        }
    }
}

//static void SwapBuffer(void)
//{
//    currentBufferIndex = (currentBufferIndex == pbiFIRST) ? pbiSECOND : pbiFIRST;
//}

/* 
 *  This RTE functions could be used: 
 *  Rte_Pim_PwmEnableStatus
 *  Rte_Pim_RequiredVector
 *  Rte_Read_DutyCycles_dutyCycles
 *  Rte_Write_Status_Status
 */
void PWMDriver_ruRefresh(void)
{
//	uint32 pwmCycleDuration[arrSpaceVector_ELEMENTS_COUNT];
//	arrSpaceVector requiredSpaceVectors;
//    uint16 setTime = 0u;
//    uint16 sumTime = 0u;
    
//
//    Rte_Read_RequiredSpaceVectors_SpaceVectors(&requiredSpaceVectors);
//
//    /* Each step of PWM consist of 6 different PWM vectors which fill full pwm cycle. Precalculate timer int values from float */
//    for (uint32 i = 0; i < arrDutyCycles_ELEMENTS_COUNT; i++)
//    {
//    	pwmCycleDuration[i] = sumTime + (uint32)(requiredSpaceVectors.Values[i].Duration * PWM_TIMER_ONE_PERIOD_LENGTH);
//    	sumTime += pwmCycleDuration[i];
//    }
//
//    /* Fill full period of scheduler with PWM cycles */
//    for (uint32 pwmPeriod = 0; pwmPeriod < PWM_PERIODS_PER_SCHEDULER_PERIODS; pwmPeriod++)
//    {
//    	/* Each of sub periods of PWM shall be written to the DMA buffer */
//        for (uint32 step = 0u; step < arrDutyCycles_ELEMENTS_COUNT; step++)
//        {
//        	/* Each next vector shall be started after previous vector is finished. First vector always starts from zero */
//            WriteSpaceVectorToPwmBuffer(currentBufferIndex, step, requiredSpaceVectors.Values[step].Direction, setTime);
//
//            /* Calculate start of the next vector */
//            setTime += pwmCycleDuration[step];
//        }
//    }
//
//    /* Two buffers shall be used because one of buffers is always used during calculation. */
//    SwapBuffer();
}

/* END OF GLOBAL FUNCTION DEFINITIONS */

/* End of file */
