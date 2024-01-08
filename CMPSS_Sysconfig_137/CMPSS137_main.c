//#############################################################################
//
// FILE:   empty_driverlib_main.c
//
//! \addtogroup driver_example_list
//! <h1>Empty Project Example</h1> 
//!
//! This example is an empty project setup for Driverlib development.
//!
//
//#############################################################################
//
//
// $Copyright:
// Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "c2000ware_libraries.h"


float32_t High_V = 2.0f;
float32_t Low_V = 1.0f;

float32_t High_I    = 8.0f;
float32_t Low_I     = 6.0f;

float32_t CurrentmodeH[3];
float32_t CurrentmodeL[3];
uint16_t CurrentTime[3];

uint16_t start = 0;

uint16_t High_DAC = 3000U;
uint16_t Low_DAC = 1000U;

uint16_t currentScaling = 0;
uint16_t constantmode = 1;
//
// Main
//
void main(void)
{

    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pull-ups.
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // PinMux and Peripheral Initialization
    //
    Board_init();

    //
    // C2000Ware Library initialization
    //
    C2000Ware_libraries_init();

    //
    // Enable Global Interrupt (INTM) and real time interrupt (DBGM)
    //
    EINT;
    ERTM;



    CurrentmodeH[0] = 10.0f;
    CurrentmodeH[0] = 9.0f;
    CurrentmodeH[0] = 8.0f;

    CurrentmodeL[0] = 9.0f;
    CurrentmodeL[0] = 7.0f;
    CurrentmodeL[0] = 7.0f;

    CurrentTime[0] = 10;
    CurrentTime[0] = 20;
    CurrentTime[0] = 30;



    while(1)
    {

        if(constantmode)
        {
            DEVICE_DELAY_US(100);
            uint16_t High_DAC_old = High_DAC;
            uint16_t Low_DAC_old = Low_DAC;

            if(currentScaling)
            {
                High_V  = High_I / 50.0f * 3.0f;
                Low_V   = Low_I  / 50.0f * 3.0f;
            }

            High_DAC    = (uint16_t)(High_V * 4096.0f / 3.3f) - 1U;
            Low_DAC     = (uint16_t)(Low_V * 4096.0f / 3.3f) - 1U;

            if(High_DAC_old != High_DAC)
            {
                //
                // Sets the value of the internal DAC of the high comparator.
                //
                CMPSS_setDACValueHigh(Hys_BASE,High_DAC);
            }
            if(Low_DAC_old != Low_DAC)
            {
                //
                // Sets the value of the internal DAC of the low comparator.
                //
                CMPSS_setDACValueLow(Hys_BASE,Low_DAC);
    //            CMPSS_setDACValueHigh(myCMPSS0_BASE,Low_DAC);
            }
        }
        else
        {
            static uint16_t count = 0;
            static uint16_t started = 0;
            uint16_t phase = 0;
            DEVICE_DELAY_US(10);
            if(start)
            {
                count = 0;
                started = 1;
                start = 0;
            }
            if(started)
            {
                count++;
                if(count <= CurrentTime[0])
                {
                    phase = 0;
                }
                else if(count > CurrentTime[1] && count <= CurrentTime[2])
                {
                    phase = 1;
                }
                else if(count > CurrentTime[2])
                {
                    phase = 2;
                    started = 0;
                }

                uint16_t High_DAC_old = High_DAC;
                uint16_t Low_DAC_old = Low_DAC;

                High_I = CurrentmodeH[phase];
                Low_I = CurrentmodeL[phase];


                High_V  = High_I / 50.0f * 3.0f;
                Low_V   = Low_I  / 50.0f * 3.0f;


                High_DAC    = (uint16_t)(High_V * 4096.0f / 3.3f) - 1U;
                Low_DAC     = (uint16_t)(Low_V * 4096.0f / 3.3f) - 1U;

                if(High_DAC_old != High_DAC)
                {
                    CMPSS_setDACValueHigh(Hys_BASE,High_DAC);
                }
                if(Low_DAC_old != Low_DAC)
                {
                    CMPSS_setDACValueLow(Hys_BASE,Low_DAC);
                }


//                    float32_t Currentmode[3];
//                    float32_t CurrentTime[3];



            }
        }
    }
}

extern __interrupt void INT_myCPUTIMER0_ISR(void)
{
    GPIO_togglePin(4);
    CPUTimer_clearOverflowFlag(myCPUTIMER0_BASE);
    Interrupt_clearACKGroup(INT_myCPUTIMER0_INTERRUPT_ACK_GROUP);
}

//
// End of File
//
