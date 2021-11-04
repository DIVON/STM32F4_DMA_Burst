/*
 * file: Rte_PWMDriver.h
 *
 * Implementation for PWMDriver header file
 *
 * $Author: $
 * $Date: $
 * $Revision: $
 *
 * Magnetic Active Suspension System.
 */
#ifndef RTE_PWMDRIVER_H
#define RTE_PWMDRIVER_H


/* INCLUDES */
#include "Rte_DataTypes.h"

/* END OF INCLUDES */


/* MACROS */
/* Runnables frequences */
#define Rte_Period_PWMDriver_ruInit               (0UL)
#define Rte_Period_PWMDriver_ruRefresh            (0UL)
#define Rte_Pim_PwmEnableStatus                   Rte_Pim_PWMDriver_PwmEnableStatus
#define Rte_Pim_RequiredVector                    Rte_Pim_PWMDriver_RequiredVector
#define Rte_Call_BridgeControl_EnablePwm          Rte_Call_PWMDriver_BridgeControl_EnablePwm
#define Rte_Call_BridgeControl_SetVector          Rte_Call_PWMDriver_BridgeControl_SetVector
#define Rte_Call_BridgeControl_Stop               Rte_Call_PWMDriver_BridgeControl_Stop
#define Rte_Read_RequiredSpaceVectors_SpaceVectors Rte_Read_PWMDriver_RequiredSpaceVectors_SpaceVectors
#define Rte_Write_Status_Status                   Rte_Write_PWMDriver_Status_Status

/* END OF MACROS */


/* RTE FUNCTION DECLARATIONS */

Std_ReturnType Rte_Read_RequiredSpaceVectors_SpaceVectors(arrSpaceVector * const SpaceVectors);
Std_ReturnType Rte_Write_Status_Status(const enPwmDriverStatus * const Status);
enPwmDriverStatus * const Rte_Pim_PwmEnableStatus();
enBridgeRelayVectorIndex * const Rte_Pim_RequiredVector();
/* END OF RTE FUNCTION DECLARATIONS */


#endif /* RTE_PWMDRIVER_H */
