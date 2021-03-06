/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "fsl_mmdvsq_hal.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : MMDVSQ_HAL_DivUR
 * Description   : perform the MMDVSQ unsigned divide operation
 * This function will return the remainder to result register, it doesn't check  
 * whether divide by zero.  
 *
 *END**************************************************************************/
uint32_t MMDVSQ_HAL_DivUR(uint32_t baseAddr, uint32_t dividend, uint32_t divisor)
{
    MMDVSQ_HAL_SetDivideFastStart(baseAddr, true);     /* set fast start mode */ 
    MMDVSQ_HAL_SetUnsignedCalculation(baseAddr, true);  /* set [USGN] bit  */ 
    MMDVSQ_HAL_SetRemainderCalculation(baseAddr, true); /* set [REM] bit  */
    MMDVSQ_HAL_SetDividend(baseAddr, dividend);    /* set dividend value */ 
    MMDVSQ_HAL_SetDivisor(baseAddr, divisor);      /* set divisor value and start divide fast mode operation*/     
    return MMDVSQ_HAL_GetResult(baseAddr);        /* retrun divide result */           
}                                        
                   
/*FUNCTION**********************************************************************
 *
 * Function Name : MMDVSQ_HAL_DivUQ
 * Description   : perform the MMDVSQ unsigned divide operation
 * This function will return the quotient to result register, it doesn't check  
 * whether divide by zero.  
 *
 *END**************************************************************************/
uint32_t MMDVSQ_HAL_DivUQ(uint32_t baseAddr, uint32_t dividend, uint32_t divisor)
{
    MMDVSQ_HAL_SetDivideFastStart(baseAddr, true);     /* set fast start mode */ 
    MMDVSQ_HAL_SetUnsignedCalculation(baseAddr, true);  /* set [USGN] bit  */ 
    MMDVSQ_HAL_SetRemainderCalculation(baseAddr, false);/* set [REM] bit  */
    MMDVSQ_HAL_SetDividend(baseAddr, dividend);    /* set dividend value */ 
    MMDVSQ_HAL_SetDivisor(baseAddr, divisor);      /* set divisor value and start divide fast mode operation*/     
    return MMDVSQ_HAL_GetResult(baseAddr);        /* retrun divide result */           
} 

/*FUNCTION**********************************************************************
 *
 * Function Name : MMDVSQ_HAL_DivSR
 * Description   : perform the MMDVSQ signed divide operation
 * This function will return the remainder to result register, it doesn't check  
 * whether divide by zero.  
 *
 *END**************************************************************************/
uint32_t MMDVSQ_HAL_DivSR(uint32_t baseAddr, uint32_t dividend, uint32_t divisor)
{
    MMDVSQ_HAL_SetDivideFastStart(baseAddr, true);     /* set fast start mode */ 
    MMDVSQ_HAL_SetUnsignedCalculation(baseAddr, false); /* set [USGN] bit  */ 
    MMDVSQ_HAL_SetRemainderCalculation(baseAddr, true); /* set [REM] bit  */
    MMDVSQ_HAL_SetDividend(baseAddr, dividend);    /* set dividend value */ 
    MMDVSQ_HAL_SetDivisor(baseAddr, divisor);      /* set divisor value and start divide fast mode operation*/     
    return MMDVSQ_HAL_GetResult(baseAddr);        /* retrun divide result */           
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MMDVSQ_HAL_DivSQ
 * Description   : perform the MMDVSQ signed divide operation
 * This function will return the quotient to result register, it doesn't check  
 * whether divide by zero.  
 *
 *END**************************************************************************/
uint32_t MMDVSQ_HAL_DivSQ(uint32_t baseAddr, uint32_t dividend, uint32_t divisor)
{
    MMDVSQ_HAL_SetDivideFastStart(baseAddr, true);     /* set fast start mode */ 
    MMDVSQ_HAL_SetUnsignedCalculation(baseAddr, false); /* set [USGN] bit  */ 
    MMDVSQ_HAL_SetRemainderCalculation(baseAddr, false); /* set [REM] bit  */
    MMDVSQ_HAL_SetDividend(baseAddr, dividend);    /* set dividend value */ 
    MMDVSQ_HAL_SetDivisor(baseAddr, divisor);      /* set divisor value and start divide fast mode operation*/     
    return MMDVSQ_HAL_GetResult(baseAddr);        /* retrun divide result */           
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MMDVSQ_HAL_Sqrt
 * Description   : perform the MMDVSQ square root operation
 * This function will return the square root result. 
 *
 *END**************************************************************************/                   
uint16_t MMDVSQ_HAL_Sqrt(uint32_t baseAddr, uint32_t radicand)
{
    MMDVSQ_HAL_SetRadicand(baseAddr, radicand);        
    return MMDVSQ_HAL_GetResult(baseAddr);  /* return square root calcluation result */
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

