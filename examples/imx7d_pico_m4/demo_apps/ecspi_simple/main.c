/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "board.h"
#include "uart_imx.h"
#include "debug_console_imx.h"
#include "ecspi_xfer.h"
#include "clock_freq.h"

/* define ECSPI module parameters configuration. */
#define ECSPI_BURSTLENGTH               (39)
#define ECSPI_STARTMODE                 (false)

/******************************************************************************
*
* Function Name: vMemoryTask
* Comments: this task is used to operate memory, such as read status, write data 
*           in memory and read data from memory.
*
******************************************************************************/
void vMemoryTask(void *pvParameters)
{
    PRINTF("\n--------------------------- ECSPI Flash Demo ----------------------------\n\n\r");
    PRINTF("This demo application demonstrates usage of ECSPI driver based on FreeRTOS.\n\r");
    PRINTF("It transfers data over SPI bus.\n\n\r");

	uint8_t tx_buff[3];
	uint8_t rx_buff[3];
	uint8_t cpt = 0;

	tx_buff[0] = 0x55;
	tx_buff[1] = 0xAA;

	while(1) {
		tx_buff[2] = cpt++;
		if (!ECSPI_XFER_TransferBlocking((uint8_t*) tx_buff, (uint8_t*) rx_buff, 3))
			PRINTF("transfer error\n");
	}
		 
}

/******************************************************************************
*
* Function Name: main
* Comments: main function, MCU configured as master mode, read from and write to
*           flash memory with FreeRTOS.
*
******************************************************************************/
int main(void)
{
    /* Hardware initialiize, include RDC, CLOCK, IOMUX, ENABLE MODULE */
    hardware_init();

    /* Ecspi module initialize, include configure parameters */
    ecspi_init_config_t ecspiInitConfig = {
        .clockRate = get_ecspi_clock_freq(BOARD_ECSPI_BASEADDR),
        .baudRate = 500000,
        .mode = ecspiMasterMode,
        .burstLength = ECSPI_BURSTLENGTH,
        .channelSelect = BOARD_ECSPI_CHANNEL,
        .clockPhase = ecspiClockPhaseFirstEdge,
        .clockPolarity = ecspiClockPolarityActiveHigh,
        .ecspiAutoStart = ECSPI_STARTMODE
    };
    ECSPI_XFER_Config(&ecspiInitConfig);

    /* Create two tasks with different priority */
    xTaskCreate(vMemoryTask, "memory", configMINIMAL_STACK_SIZE,
	            NULL, tskIDLE_PRIORITY+1, NULL);

    /* Start task schedule */
    vTaskStartScheduler();

    for(;;);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
