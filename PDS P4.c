/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
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

#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_adc16.h"
#include "fsl_dac.h"

#include "clock_config.h"
#include "pin_mux.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "peripherals.h"
#include "pin_mux.h"
#include "MK64F12.h"
#include "fsl_port.h"
#include "fsl_gpio.h"

#include "Botones.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_ADC16_BASEADDR ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define DEMO_ADC16_USER_CHANNEL 12U /* PTB2, ADC0_SE12 */
#define DEMO_DAC_BASEADDR DAC0

#define DEMO_ADC16_IRQn ADC0_IRQn
#define DEMO_ADC16_IRQ_HANDLER_FUNC ADC0_IRQHandler
#define DAC_1_0_VOLTS 1241U

#define VREF_BRD 3.300
#define SE_12BIT 4096.0

/*! This definition is as a general definitions to bits in regiter or pins in the microcontroller.*/
typedef enum {BIT0,  /*!< Bit 0 */
              BIT1,  /*!< Bit 1 */
              BIT2,  /*!< Bit 2 */
              BIT3,  /*!< Bit 3 */
              BIT4,  /*!< Bit 4 */
              BIT5,  /*!< Bit 5 */
              BIT6,  /*!< Bit 6 */
              BIT7,  /*!< Bit 7 */
              BIT8,  /*!< Bit 8 */
              BIT9,  /*!< Bit 9 */
              BIT10, /*!< Bit 10 */
              BIT11, /*!< Bit 11 */
              BIT12, /*!< Bit 12 */
              BIT13, /*!< Bit 13 */
              BIT14, /*!< Bit 14 */
              BIT15, /*!< Bit 15 */
              BIT16, /*!< Bit 16 */
              BIT17, /*!< Bit 17 */
              BIT18, /*!< Bit 18 */
              BIT19, /*!< Bit 19 */
              BIT20, /*!< Bit 20 */
              BIT21, /*!< Bit 21 */
              BIT22, /*!< Bit 22 */
              BIT23,/*!< Bit 23 */
              BIT24, /*!< Bit 24 */
              BIT25, /*!< Bit 25 */
              BIT26, /*!< Bit 26 */
              BIT27, /*!< Bit 27 */
              BIT28, /*!< Bit 28 */
              BIT29, /*!< Bit 29 */
              BIT30, /*!< Bit 30 */
              BIT31  /*!< Bit 31 */
            } BitsType;

typedef union
        {
            uint8_t allBits;
            struct{
                uint8_t bit0 :1;
                uint8_t bit1 :1;
                uint8_t bit2 :1;
                uint8_t bit3 :1;
            }bitField;
        } Binario;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* Initialize ADC16 & DAC */
static void DAC_ADC_Init(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool g_Adc16ConversionDoneFlag = false;
volatile uint32_t g_Adc16ConversionValue = 1;
adc16_channel_config_t g_adc16ChannelConfigStruct;

/*******************************************************************************
 * Code
 ******************************************************************************/
static void DAC_ADC_Init(void)
{
    adc16_config_t adc16ConfigStruct;
    dac_config_t dacConfigStruct;

    /* Configure the DAC. */
    /*
     * dacConfigStruct.referenceVoltageSource = kDAC_ReferenceVoltageSourceVref2;
     * dacConfigStruct.enableLowPowerMode = false;
     */
    DAC_GetDefaultConfig(&dacConfigStruct);
    DAC_Init(DEMO_DAC_BASEADDR, &dacConfigStruct);
    DAC_Enable(DEMO_DAC_BASEADDR, true); /* Enable output. */

    /* Configure the ADC16. */
    /*
     * adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
     * adc16ConfigStruct.clockSource = kADC16_ClockSourceAsynchronousClock;
     * adc16ConfigStruct.enableAsynchronousClock = true;
     * adc16ConfigStruct.clockDivider = kADC16_ClockDivider8;
     * adc16ConfigStruct.resolution = kADC16_ResolutionSE12Bit;
     * adc16ConfigStruct.longSampleMode = kADC16_LongSampleDisabled;
     * adc16ConfigStruct.enableHighSpeed = false;
     * adc16ConfigStruct.enableLowPower = false;
     * adc16ConfigStruct.enableContinuousConversion = false;
     */
    ADC16_GetDefaultConfig(&adc16ConfigStruct);
#if defined(BOARD_ADC_USE_ALT_VREF)
    adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
#endif
    adc16ConfigStruct.enableHighSpeed = true;
    ADC16_Init(DEMO_ADC16_BASEADDR, &adc16ConfigStruct);

    /* Make sure the software trigger is used. */
    ADC16_EnableHardwareTrigger(DEMO_ADC16_BASEADDR, false);
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    if (kStatus_Success == ADC16_DoAutoCalibration(DEMO_ADC16_BASEADDR))
    {
        PRINTF("\r\nADC16_DoAutoCalibration() Done.");
    }
    else
    {
        PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
    }
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

    /* Prepare ADC channel setting */
    g_adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
    g_adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = true;

#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    g_adc16ChannelConfigStruct.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */
}

void DEMO_ADC16_IRQ_HANDLER_FUNC(void)
{
    g_Adc16ConversionDoneFlag = true;
    /* Read conversion result to clear the conversion completed flag. */
    g_Adc16ConversionValue = ADC16_GetChannelConversionValue(DEMO_ADC16_BASEADDR, DEMO_ADC16_CHANNEL_GROUP);
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void inicializacionBotones ()
{
    /*Se inicializa cada uno de los pines a utilizar por los botones*/
    CLOCK_EnableClock (kCLOCK_PortC);

    port_pin_config_t config_button =
    { kPORT_PullDown, kPORT_FastSlewRate, kPORT_PassiveFilterDisable,
            kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAsGpio,
            kPORT_UnlockRegister };

    gpio_pin_config_t button_config_gpio =
    { kGPIO_DigitalInput, 1 };

    PORT_SetPinConfig (PORTC, BIT5, &config_button);
    PORT_SetPinConfig (PORTC, BIT7, &config_button);
    PORT_SetPinConfig (PORTC, BIT0, &config_button);
    PORT_SetPinConfig (PORTC, BIT9, &config_button);
    GPIO_PinInit (GPIOC, BIT5, &button_config_gpio);
    GPIO_PinInit (GPIOC, BIT7, &button_config_gpio);
    GPIO_PinInit (GPIOC, BIT0, &button_config_gpio);
    GPIO_PinInit (GPIOC, BIT9, &button_config_gpio);

    /* Init input switch GPIO. */
    PORT_SetPinInterruptConfig (PORTC, BIT5, kPORT_InterruptRisingEdge);
    PORT_SetPinInterruptConfig (PORTC, BIT7, kPORT_InterruptRisingEdge);
    PORT_SetPinInterruptConfig (PORTC, BIT0, kPORT_InterruptRisingEdge);
    PORT_SetPinInterruptConfig (PORTC, BIT9, kPORT_InterruptRisingEdge);
    /*Se habilitan sus interrupciones*/
    NVIC_EnableIRQ (PORTC_IRQn);
    NVIC_SetPriority (PORTC_IRQn, 3);
}


#define SAMPLE_SIZE 20000

static SemaphoreHandle_t sem;

uint16_t adc_read[SAMPLE_SIZE] = {0};
uint16_t dac_out[SAMPLE_SIZE] = {0};
uint16_t dac_out_2[SAMPLE_SIZE] = {0};

volatile static float alpha = 0.5;
volatile static uint16_t time_delay = 10000;

static volatile Butons valorBoton;

void PORTC_IRQHandler (void)
{
    Binario numero = { 0 };

    /*Se leen los pines y se guardan en la estructura*/
    numero.bitField.bit0 = GPIO_PinRead (GPIOC, BIT5); //B0
    numero.bitField.bit1 = GPIO_PinRead (GPIOC, BIT7); //B1
    numero.bitField.bit2 = GPIO_PinRead (GPIOC, BIT0); //B2
    numero.bitField.bit3 = GPIO_PinRead (GPIOC, BIT9); //B2

    uint32_t x = numero.allBits;

    /*Dependiendo del botón, la funcion regresará 0,1,2,3,4 or 5*/
    if (BOTON_B0_MASK & x)
    {
        valorBoton = BUTTON_0;
        time_delay += 2000;
        if(SAMPLE_SIZE <= time_delay)
            time_delay = SAMPLE_SIZE;
    }
    else if (BOTON_B1_MASK & x)
    {
        time_delay -= 2000;
        if((0 > time_delay) || (SAMPLE_SIZE <= time_delay))
            time_delay = 0;
        valorBoton = BUTTON_1;
    }
    else if (BOTON_B2_MASK & x)
    {
        alpha += 0.100000;
        if(1.1 < alpha)
            alpha = 1;
        valorBoton = BUTTON_2;
    }
    else if (BOTON_B3_MASK & x)
    {
        alpha -= 0.1;
        if((1.1 <= alpha) || (0 > alpha))
            alpha = 0;
        valorBoton = BUTTON_3;
    }
    else
    {
        valorBoton = NO_BUTTON;
    }

    PORT_ClearPinsInterruptFlags (PORTC, 1 << (BIT5)); //clear irq
    PORT_ClearPinsInterruptFlags (PORTC, 1 << (BIT7)); //clear irq
    PORT_ClearPinsInterruptFlags (PORTC, 1 << (BIT0)); //clear irq
    PORT_ClearPinsInterruptFlags (PORTC, 1 << (BIT9)); //clear irq

    uint32_t bits_irq = PORT_GetPinsInterruptFlags (PORTC);

    return;
}

void adc_task()
{
    float voltRead;
    while (1)
        {
            for(uint16_t i = 0; SAMPLE_SIZE > i; i++)
            {
                g_Adc16ConversionDoneFlag = false;
                ADC16_SetChannelConfig(DEMO_ADC16_BASEADDR, DEMO_ADC16_CHANNEL_GROUP, &g_adc16ChannelConfigStruct);

                while (!g_Adc16ConversionDoneFlag);

                /* Convert ADC value to a voltage based on 3.3V VREFH on board */
                voltRead = (float)(g_Adc16ConversionValue * (VREF_BRD / SE_12BIT));
                adc_read[i] = (uint16_t) (DAC_1_0_VOLTS*voltRead);
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            for(uint16_t i = 0; SAMPLE_SIZE > i; i++)
            {
                dac_out_2[i] = dac_out[i];
                dac_out[i] = adc_read[i];
            }
            xSemaphoreGive(sem);
        }
}

void dac_task()
{
    while (1)
    {
        xSemaphoreTake(sem, portMAX_DELAY);
        for(uint16_t i = 0; (SAMPLE_SIZE) > i; i++)
        {
            uint32_t  val = (uint32_t) dac_out[i];
            if(time_delay >= i)
            {
                val += alpha*dac_out_2[i];
            }
            else
            {
                uint8_t uy = 0;
                uy++;
            }
            DAC_SetBufferValue(DEMO_DAC_BASEADDR, 0U,(uint32_t ) (val/2));
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

void init_task()
{
    vTaskDelete(NULL);
}

/*!
 * @brief Main function
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_InitBootClocks ();
    /* Init FSL debug console. */
    BOARD_InitDebugConsole ();
    EnableIRQ(DEMO_ADC16_IRQn);
    DAC_ADC_Init();
    inicializacionBotones ();

    sem = xSemaphoreCreateBinary();
    xSemaphoreTake(sem, pdMS_TO_TICKS(1));

    xTaskCreate (adc_task, "ADC", 110, NULL, configMAX_PRIORITIES - 1,
               NULL);
    xTaskCreate (dac_task, "DAC", 110, NULL, configMAX_PRIORITIES - 1,
               NULL);

   vTaskStartScheduler ();

    while(1);
}
