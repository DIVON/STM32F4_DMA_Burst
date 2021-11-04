/*
 * file: PWMDriver_Init.c
 *
 * Implementation for PWMDriver_Init
 *
 * $Author: $
 * $Date: $
 * $Revision: $
 *
 * Magnetic Active Suspension System.
 */

/* INCLUDES */

#include "PWMDriver.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

/* END OF INCLUDES */


/* MACROS */

/* END OF MACROS */


/* TYPE DEFINITIONS */

/* END OF TYPE DEFINITIONS */


/* VARIABLES */

/* END OF VARIABLES */


/* LOCAL FUNCTION DECLARATIONS */

/* END OF LOCAL FUNCTION DECLARATIONS */


/* LOCAL FUNCTION DEFINITIONS */

void ConfigureDmaStream(DMA_Stream_TypeDef * dmaStream, uint16 * srcAddress0, uint16 * srcAddress1, volatile uint32_t * dstAddress, const uint16 dataLength)
{
    /* Configure the source, destination address and the data length */

    /* Configure Number Data To Transmit */
    dmaStream->NDTR = dataLength;


    /* Memory to peripheral */

    /* Configure DMA Stream source address */
    dmaStream->PAR = (uint32)dstAddress;

    /* Configure DMA Stream destination address */
    dmaStream->M0AR = (uint32)srcAddress0;
    dmaStream->M1AR = (uint32)srcAddress1;

    dmaStream->CR &= ~DMA_SxCR_HTIE;

    dmaStream->CR |=  DMA_SxCR_CIRC |       /* Enable circular mode */
                      DMA_SxCR_DBM  |       /* Enable double buffer mode */
                      DMA_SxCR_EN   |       /* Enable the Peripheral */
                      DMA_SxCR_TCIE |       /* Enable transfer complete interrupt flag */
                      DMA_SxCR_TEIE;        /* Enable transfer error interrupt flag*/
}

static void ConfigureTimer()
{
    /* Channel 1-3 are used as complementary outputs,
     * Channel 4 is used for updating CCR1-4 values via triggering DMA */

    PWM_TIMER->CR2 &= ~TIM_CR2_CCDS;    /* CCx DMA requests sent when CCx event occurs */
    PWM_TIMER->CR2 |= TIM_TRGO_OC4REF;

    /* Enable DMA Triggering by CC4 */
    PWM_TIMER->DIER |= TIM_DIER_CC4DE | TIM_DIER_CC4IE;  /* Capture/Compare 4 DA request enable */
    PWM_TIMER->CCER |= TIM_CCER_CC4E;

    /* Configure the DMA Burst Mode */
    PWM_TIMER->DCR = 0u;
    PWM_TIMER->DCR = TIM_DMABASE_CCR1 | TIM_DMABURSTLENGTH_4TRANSFERS;

    /* Pull all output pins to the ground */
    PWM_TIMER->CCR1 = PWM_PULL_DOWN;
    PWM_TIMER->CCR2 = PWM_PULL_DOWN;
    PWM_TIMER->CCR3 = PWM_PULL_DOWN;

    /* Enable timer interrupt by CCR4 */
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
}
/* END OF LOCAL FUNCTION DEFINITIONS */


/* GLOBAL FUNCTION DEFINITIONS */

/* 
 *  This RTE functions could be used: 
 *  Rte_Pim_PwmEnableStatus
 *  Rte_Pim_RequiredVector
 *  Rte_Read_RequiredSpaceVectors_SpaceVectors
 *  Rte_Write_Status_Status
 */

#define CCRx_VALUES_COUNT 80u
volatile uint32_t CCRxValues1[24]= {
		PWM_PULL_UP,  	PWM_PULL_DOWN, 	PWM_PULL_DOWN, 	500,	//UP
		PWM_PULL_UP,  	PWM_PULL_UP,   	PWM_PULL_DOWN, 	1500,	//WM
		PWM_PULL_DOWN,  PWM_PULL_UP, 	PWM_PULL_DOWN, 	2500,	//VP
		PWM_PULL_DOWN, 	PWM_PULL_UP, 	PWM_PULL_UP, 	3500,	//UM
		PWM_PULL_DOWN,  PWM_PULL_DOWN, 	PWM_PULL_UP, 	4500,	//WP
		PWM_PULL_UP,    PWM_PULL_DOWN, 	PWM_PULL_UP, 	0u		//VM
};


volatile uint32_t CCRxValues2[24]= {
		PWM_PULL_UP,    PWM_PULL_DOWN, 	PWM_PULL_UP, 	100u,	//VM
		PWM_PULL_DOWN,  PWM_PULL_DOWN, 	PWM_PULL_UP, 	200,	//WP
		PWM_PULL_DOWN, 	PWM_PULL_UP, 	PWM_PULL_UP, 	300,	//UM
		PWM_PULL_DOWN,  PWM_PULL_UP, 	PWM_PULL_DOWN, 	400,	//VP
		PWM_PULL_UP,  	PWM_PULL_UP,   	PWM_PULL_DOWN, 	500,	//WM
		PWM_PULL_UP,  	PWM_PULL_DOWN, 	PWM_PULL_DOWN, 	0		//UP
};

//volatile uint32_t CCRxValues2[CCRx_VALUES_COUNT]= {
//		   1,  21, 31, 1500,
//		   2,  22, 32, 2500,
//		   3,  23, 33, 3500,
//		   4,  24, 34, 4500,
//		   5,  25, 35, 5500,
//		   6,  26, 36, 6500,
//		   7,  27, 37, 7500,
//		   8,  28, 38, 8500,
//		   9,  29, 39, 9500,
//		   10, 30, 40, 10500,
//		   11, 31, 41, 11500,
//		   12, 32, 42, 12500,
//		   13, 33, 43, 13500,
//		   14, 34, 44, 14500,
//		   15, 35, 45, 15500,
//		   16, 36, 46, 16500,
//		   17, 37, 47, 17500,
//		   18, 38, 48, 18500,
//		   19, 39, 49, 19500,
//		   20, 30, 50, 20000      };

void PWMDriver_ruInit(void)
{
    /* First buffer */
    WriteSpaceVectorToPwmBuffer(pbiFIRST, 0u, SV_ZM, 0u);
    WriteSpaceVectorToPwmBuffer(pbiFIRST, 1u, SV_ZM, 3000u);

    WriteSpaceVectorToPwmBuffer(pbiFIRST, 2u, SV_ZM, 6000u);
    WriteSpaceVectorToPwmBuffer(pbiFIRST, 3u, SV_ZM, 9000u);

    WriteSpaceVectorToPwmBuffer(pbiFIRST, 4u, SV_ZM, 14000u);
    WriteSpaceVectorToPwmBuffer(pbiFIRST, 5u, SV_ZM, 17000u);

    /* Second buffer */
    WriteSpaceVectorToPwmBuffer(pbiSECOND, 0u, SV_ZM, 0u);
    WriteSpaceVectorToPwmBuffer(pbiSECOND, 1u, SV_ZM, 1500u);

    WriteSpaceVectorToPwmBuffer(pbiSECOND, 2u, SV_ZM, 6000u);
    WriteSpaceVectorToPwmBuffer(pbiSECOND, 3u, SV_ZM, 9000u);

    WriteSpaceVectorToPwmBuffer(pbiSECOND, 4u, SV_ZM, 12000u);
    WriteSpaceVectorToPwmBuffer(pbiSECOND, 5u, SV_ZM, 16000u);

//    ConfigureDmaStream(
//            PWM_DMA_STREAM,
//            &pwmBuffers[0].step[0].phaseUstate,
//            &pwmBuffers[1].step[0].phaseUstate,
//            &PWM_TIMER->DMAR,
//            PWM_MAX_STEPS_COUNT * 4u);

    ConfigureTimer();
}

/* END OF GLOBAL FUNCTION DEFINITIONS */

/* End of file */
