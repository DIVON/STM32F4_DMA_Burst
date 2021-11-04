/*
 * file: PWMDriver.h
 *
 * Implementation for header of PWMDriver
 *
 * $Author: ivan.davydenko $
 * $Date: 2020-04-24 11:20:19 +0300 (Пт, 24 апр 2020) $
 * $Revision: 434 $
 *
 * Magnetic Active Suspension System.
 */

/**** INCLUDES ***********************************************************************************/

#include "Rte_PWMDriver.h"
#include "stm32f4xx_hal.h"

/**** END OF INCLUDES ****************************************************************************/


/**** MACROS *************************************************************************************/

#define PWM_DMA_STREAM  DMA2_Stream4
#define PWM_TIMER       TIM1

#define PWM_MAX_STEPS_COUNT  6u

#define PWM_PULL_UP     0xFFFFu
#define PWM_PULL_DOWN   0x0u

/**** END OF MACROS ******************************************************************************/


/**** TYPE DEFINITIONS ***************************************************************************/

typedef enum
{
    BRIDGE_NO_REQUEST,
    BRIDGE_OFF,
    BRIDGE_LEG_CONTROL,
    BRIDGE_PWM_ON
} BridgeControlRequest;


typedef enum
{
    pbiFIRST,
    pbiSECOND,
	pbiBUFFERS_COUNT
} enPwmBufferIndex;

typedef struct
{
    uint16 phaseUstate;
    uint16 phaseVstate;
    uint16 phaseWstate;
    uint16 setTime;
} PwmBufferStep;

typedef struct
{
    PwmBufferStep step[PWM_MAX_STEPS_COUNT];
} cdtPwmBuffer;


/**** END OF TYPE DEFINITIONS ********************************************************************/


/**** EXTERNAL VARIABLES *************************************************************************/

/**** END OF EXTERNAL VARIABLES ******************************************************************/


/**** GLOBAL FUNCTION DECLARATIONS ***************************************************************/

void WriteSpaceVectorToPwmBuffer(enPwmBufferIndex bufferIndex, uint8 step, enSpaceVectorDirection vector, uint32 setTime);

/**** END OF GLOBAL FUNCTION DECLARATIONS ********************************************************/

/* End of file */
