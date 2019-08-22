/*
 * Copyright 2016-2019 NXP
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
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
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
 
/**
 * @file    LPC54114J256_Project_motor drivers.c
 * @brief   Application entry point.
 */
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_usart.h"
#include "fsl_sctimer.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/


#define USART USART2
#define USART_CLK_SRC kCLOCK_Flexcomm2
#define USART_CLK_FREQ CLOCK_GetFreq(kCLOCK_Flexcomm2)
#define SCTIMER_CLK_FREQ CLOCK_GetFreq(kCLOCK_BusClk)
#define FIRST_SCTIMER_OUT kSCTIMER_Out_4
#define SECOND_SCTIMER_OUT kSCTIMER_Out_5
#define THIRD_SCTIMER_OUT kSCTIMER_Out_7
#define FOURTH_SCTIMER_OUT kSCTIMER_Out_2
 sctimer_config_t sctimerInfo;
 sctimer_pwm_signal_param_t pwmParam;
 uint32_t event1,event2,event3,event4,speed=70;
 uint32_t sctimerClock;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

uint8_t txbuff[] = "Usart polling example\r\nBoard will send back received characters\r\n";
//uint8_t rxbuff[20] = {0};

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{
    uint8_t ch;
    usart_config_t config;



   sctimer_config_t sctimerInfo;
   sctimer_pwm_signal_param_t pwmParam;


   uint32_t event1,event2,event3,event4,speed=79;
   uint32_t sctimerClock;

    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk( BOARD_DEBUG_UART_CLK_ATTACH_CORE1);

    /* reset FLEXCOMM for USART */
    RESET_PeripheralReset( kFC0_RST_SHIFT_RSTn);
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);
    BOARD_InitPins();
    BOARD_BootClockFROHF48M();
    BOARD_InitDebugConsole();
    /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kUSART_ParityDisabled;
     * config.stopBitCount = kUSART_OneStopBit;
     * config.loopback = false;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    USART_GetDefaultConfig(&config);
    config.baudRate_Bps = 115200;
    config.enableTx = true;
    config.enableRx = true;
    USART_Init(USART, &config, USART_CLK_FREQ);

    sctimerClock = SCTIMER_CLK_FREQ;
    SCTIMER_GetDefaultConfig(&sctimerInfo);
    /* Initialize SCTimer module */
    SCTIMER_Init(SCT0, &sctimerInfo);



    pwmParam.output = FIRST_SCTIMER_OUT;
    pwmParam.level = kSCTIMER_HighTrue;
    pwmParam.dutyCyclePercent = 1;
    if (SCTIMER_SetupPwm(SCT0, &pwmParam, kSCTIMER_EdgeAlignedPwm , 2400U, sctimerClock, &event1) == kStatus_Fail)
    {
           return -1;
    }

    pwmParam.output = SECOND_SCTIMER_OUT;
    pwmParam.level = kSCTIMER_HighTrue;
        pwmParam.dutyCyclePercent =2 ;
        if (SCTIMER_SetupPwm(SCT0, &pwmParam, kSCTIMER_EdgeAlignedPwm , 2400U, sctimerClock, &event2) == kStatus_Fail)
        {
               return -1;
        }
        pwmParam.output = THIRD_SCTIMER_OUT;
        pwmParam.level = kSCTIMER_HighTrue;
        pwmParam.dutyCyclePercent = 1;
        if (SCTIMER_SetupPwm(SCT0, &pwmParam, kSCTIMER_EdgeAlignedPwm , 2400U, sctimerClock, &event3) == kStatus_Fail)
        {
              return -1;
        }

        pwmParam.output = FOURTH_SCTIMER_OUT;
        pwmParam.level = kSCTIMER_HighTrue;
        pwmParam.dutyCyclePercent =2 ;
        if (SCTIMER_SetupPwm(SCT0, &pwmParam, kSCTIMER_EdgeAlignedPwm , 2400U, sctimerClock, &event4) == kStatus_Fail)
        {
                 return -1;
        }

    //USART_WriteBlocking(USART, txbuff, sizeof(txbuff) - 1);

    while (1)
    {
        USART_ReadBlocking(USART, &ch, 1);
//g

        if(ch == 'f')
                {
                printf("Rotate on axis\n");
                  SCTIMER_UpdatePwmDutycycle(SCT0,FIRST_SCTIMER_OUT, 60, event1);
                  SCTIMER_UpdatePwmDutycycle(SCT0,SECOND_SCTIMER_OUT,1, event2);
                  SCTIMER_UpdatePwmDutycycle(SCT0,THIRD_SCTIMER_OUT, 1, event3);
                  SCTIMER_UpdatePwmDutycycle(SCT0,FOURTH_SCTIMER_OUT,60, event4);

                }

        if(ch == 'b')
        {
        printf("right\n");
          SCTIMER_UpdatePwmDutycycle(SCT0,FIRST_SCTIMER_OUT, speed, event1);
          SCTIMER_UpdatePwmDutycycle(SCT0,SECOND_SCTIMER_OUT,1, event2);
          SCTIMER_UpdatePwmDutycycle(SCT0,THIRD_SCTIMER_OUT, 1, event3);
          SCTIMER_UpdatePwmDutycycle(SCT0,FOURTH_SCTIMER_OUT,1, event4);

        }
       if(ch == 'a')
        {
      SCTIMER_UpdatePwmDutycycle(SCT0,FIRST_SCTIMER_OUT, 1, event1);
      SCTIMER_UpdatePwmDutycycle(SCT0,SECOND_SCTIMER_OUT, 1, event2);
      SCTIMER_UpdatePwmDutycycle(SCT0,THIRD_SCTIMER_OUT, speed, event3);
      SCTIMER_UpdatePwmDutycycle(SCT0,FOURTH_SCTIMER_OUT, 1, event4);

      printf("left\n");
        }
        if(ch == 'c')
        {
        SCTIMER_UpdatePwmDutycycle(SCT0,FIRST_SCTIMER_OUT,speed, event1);
        SCTIMER_UpdatePwmDutycycle(SCT0,SECOND_SCTIMER_OUT,1, event2);
             SCTIMER_UpdatePwmDutycycle(SCT0,THIRD_SCTIMER_OUT,speed, event3);
        SCTIMER_UpdatePwmDutycycle(SCT0,FOURTH_SCTIMER_OUT,1, event4);

        printf("go straight\n");
        }
        if(ch == 'd')
        {
        printf("stop");
        SCTIMER_UpdatePwmDutycycle(SCT0,FIRST_SCTIMER_OUT, 1, event1);
        SCTIMER_UpdatePwmDutycycle(SCT0,SECOND_SCTIMER_OUT, 1, event2);
        SCTIMER_UpdatePwmDutycycle(SCT0,THIRD_SCTIMER_OUT, 1, event3);
        SCTIMER_UpdatePwmDutycycle(SCT0,FOURTH_SCTIMER_OUT, 1, event4);

        }
       // if(ch =='e')
        //{
        //printf("reverse");
        //rintf("right\n");
        //SCTIMER_UpdatePwmDutycycle(SCT0,FIRST_SCTIMER_OUT, 1, event1);
        //SCTIMER_UpdatePwmDutycycle(SCT0,SECOND_SCTIMER_OUT,speed, event2);
        //SCTIMER_UpdatePwmDutycycle(SCT0,THIRD_SCTIMER_OUT, 1, event3);
        //SCTIMER_UpdatePwmDutycycle(SCT0,FOURTH_SCTIMER_OUT,speed, event4);

        //}


        if(ch == 'e')
                {
                printf("stop");
                SCTIMER_UpdatePwmDutycycle(SCT0,FIRST_SCTIMER_OUT, 1, event1);
                SCTIMER_UpdatePwmDutycycle(SCT0,SECOND_SCTIMER_OUT, 2, event2);
                SCTIMER_UpdatePwmDutycycle(SCT0,THIRD_SCTIMER_OUT, 3, event3);
                SCTIMER_UpdatePwmDutycycle(SCT0,FOURTH_SCTIMER_OUT, 4, event4);

                }
        //USART_WriteBlocking(DEMO_USART, &ch, 1);
    }
}


