/*
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
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

#if !defined(__BOARD_H__)
#define __BOARD_H__

#include "pin_mux.h"
#include "rdc.h"
#include "rdc_defs_imx7d.h"
#include "ccm_imx7d.h"
#include "clock_freq.h"

/* The board name */
#define BOARD_NAME                            "IMX7D_PICO_M4"
#define DWARF_BOARD                           (0)
#define BOARD_DOMAIN_ID                       (1)

/* ADC information for this board */
#define BOARD_ADC_RDC_PDAP                    rdcPdapAdc1
#define BOARD_ADC_CCM_CCGR                    ccmCcgrGateAdc
#define BOARD_ADC_BASEADDR                    ADC1
#define BOARD_ADC_IRQ_NUM                     ADC1_IRQn
#define BOARD_ADC_HANDLER                     ADC1_Handler
#define BOARD_ADC_INPUT_CHANNEL               (3)

/* WDOG information for this board */
#define BOARD_WDOG_RDC_PDAP                   rdcPdapWdog3
#define BOARD_WDOG_CCM_ROOT                   ccmRootWdog
#define BOARD_WDOG_CCM_CCGR                   ccmCcgrGateWdog3
#define BOARD_WDOG_BASEADDR                   WDOG3
#define BOARD_WDOG_IRQ_NUM                    WDOG3_IRQn
#define BOARD_WDOG_HANDLER                    WDOG3_Handler

/* SEMA4 information for this board */
#define BOARD_SEMA4_RDC_PDAP                  rdcPdapSemaphoreHs
#define BOARD_SEMA4_CCM_CCGR                  ccmCcgrGateSemaHs
#define BOARD_SEMA4_BASEADDR                  SEMA4
#define BOARD_SEMA4_IRQ_NUM                   SEMA4_HS_M4_IRQn
#define BOARD_SEMA4_HANDLER                   SEMA4_HS_M4_Handler

/* GPT instance A information for this board */
#define BOARD_GPTA_RDC_PDAP                   rdcPdapGpt3
#define BOARD_GPTA_CCM_ROOT                   ccmRootGpt3
#define BOARD_GPTA_CCM_CCGR                   ccmCcgrGateGpt3
#define BOARD_GPTA_BASEADDR                   GPT3
#define BOARD_GPTA_IRQ_NUM                    GPT3_IRQn
#define BOARD_GPTA_HANDLER                    GPT3_Handler
/* GPT instance B information for this board */
#define BOARD_GPTB_RDC_PDAP                   rdcPdapGpt4
#define BOARD_GPTB_CCM_ROOT                   ccmRootGpt4
#define BOARD_GPTB_CCM_CCGR                   ccmCcgrGateGpt4
#define BOARD_GPTB_BASEADDR                   GPT4
#define BOARD_GPTB_IRQ_NUM                    GPT4_IRQn
#define BOARD_GPTB_HANDLER                    GPT4_Handler

/* GPIO information for this board */
#define BOARD_GPIO_LED_CCM_CCGR               ccmCcgrGateGpio2
#define BOARD_GPIO_LED_RDC_PDAP               rdcPdapGpio2
#define BOARD_GPIO_LED_CONFIG                 (&gpioLed)

#if ( DWARF_BOARD == 1 )
#define BOARD_GPIO_SENSOR_CCM_CCGR            ccmCcgrGateGpio2
#define BOARD_GPIO_SENSOR_RDC_PDAP            rdcPdapGpio2
#define BOARD_GPIO_SENSOR_CONFIG              (&gpioSensorInt)
#define BOARD_GPIO_SENSOR_IRQ_NUM             GPIO2_INT15_0_IRQn
#define BOARD_GPIO_SENSOR_HANDLER             GPIO2_INT15_0_Handler

#define BOARD_GPIO_GYR_CONFIG                 (&gpioGyrInt)

#define BOARD_GPIO_PRESSURE_CONFIG            (&gpioPressureInt)
#endif	// #if ( DWARF_BOARD == 1 )

/* Debug UART information for this board */
#define BOARD_DEBUG_UART_RDC_PDAP             rdcPdapUart6
#define BOARD_DEBUG_UART_CCM_ROOT             ccmRootUart6
#define BOARD_DEBUG_UART_CCM_CCGR             ccmCcgrGateUart6
#define BOARD_DEBUG_UART_BASEADDR             UART6
#define BOARD_DEBUG_UART_IRQ_NUM              UART6_IRQn
#define BOARD_DEBUG_UART_HANDLER              UART6_Handler
/*#define BOARD_DEBUG_UART_RDC_PDAP             rdcPdapUart6
#define BOARD_DEBUG_UART_CCM_ROOT             ccmRootUart6
#define BOARD_DEBUG_UART_CCM_CCGR             ccmCcgrGateUart6
#define BOARD_DEBUG_UART_BASEADDR             UART6
#define BOARD_DEBUG_UART_IRQ_NUM              UART6_IRQn
#define BOARD_DEBUG_UART_HANDLER              UART6_Handler*/

/* MU information for this board */
#define BOARD_MU_HANDLER                      MU_M4_Handler
#define BOARD_MU_IRQ_NUM                      MU_M4_IRQn
#define BOARD_MU_BASE_ADDR                    MUB
#define BOARD_MU_CCM_CCGR                     ccmCcgrGateMu
#define BOARD_MU_RDC_PDAP                     rdcPdapMuB

#if ( DWARF_BOARD == 1 )
/* I2C information for this board */
#define BOARD_I2C_RDC_PDAP                    rdcPdapI2c1
#define BOARD_I2C_CCM_ROOT                    ccmRootI2c1
#define BOARD_I2C_CCM_CCGR                    ccmCcgrGateI2c1
#define BOARD_I2C_BASEADDR                    I2C1
#define BOARD_I2C_IRQ_NUM                     I2C1_IRQn
#define BOARD_I2C_HANDLER                     I2C1_Handler
#define BOARD_I2C_FXAS21002_ADDR              (0x21)
#define BOARD_I2C_FXOS8700_ADDR               (0x1E)
#endif	// #if ( DWARF_BOARD == 1 )

/* GPC information for this board*/
#define BOARD_GPC_BASEADDR                    GPC

/* SIM_WAKEUP CG information*/
#define BOARD_SIM_WAKEUP_CCGR                 ccmCcgrGateSimWakeup

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

void hardware_init(void);
void dbg_uart_init(void);
/* Function to initialize clock base on board configuration. */
void BOARD_ClockInit(void);
void BOARD_RdcInit(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* __BOARD_H__ */
