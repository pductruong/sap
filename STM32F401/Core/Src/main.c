#include "stm32f4xx.h"

#define dryValue 500
#define wetValue 270
#define ALPHA 0.2f  // EMA smoothing factor

void delay_ms(uint32_t ms);
void adc_init(void);
uint16_t adc_read(void);
void usart_init(void);
void usart_send_char(char c);
void usart_send_string(char *str);

float filteredValue = 0.0f;

int main(void) {
    uint16_t rawValue;
    int moisturePercent;

    // Initialize peripherals
    adc_init();
    usart_init();

    while (1) {
        rawValue = adc_read();

        // Apply EMA filter
        if (filteredValue == 0.0f) {
            filteredValue = rawValue;
        } else {
            filteredValue = ALPHA * rawValue + (1.0f - ALPHA) * filteredValue;
        }

        // Map to percentage
        int sensorValue = (int)filteredValue;
        if (sensorValue >= dryValue)
            moisturePercent = 0;
        else if (sensorValue <= wetValue)
            moisturePercent = 100;
        else
            moisturePercent = 100 - ((sensorValue - wetValue) * 100UL / (dryValue - wetValue));

        // Send over USART
        char buffer[10];
        itoa(moisturePercent, buffer, 10);
        usart_send_string(buffer);
        usart_send_string("\r\n");

        delay_ms(1000);
    }
}

// Initialize ADC on PA0 (Channel 0)
void adc_init(void) {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    ADC_InitTypeDef ADC_InitStruct;
    ADC_StructInit(&ADC_InitStruct);
    ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStruct.ADC_ScanConvMode = DISABLE;
    ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStruct.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStruct);

    ADC_ChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_3Cycle);

    ADC_Cmd(ADC1, ENABLE);
    ADC_StartConversion(ADC1);
}

// Read ADC value
uint16_t adc_read(void) {
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    return ADC_GetConversionValue(ADC1);
}

// Initialize USART1 at 9600 baud
void usart_init(void) {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);

    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = 9600;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStruct);

    USART_Cmd(USART1, ENABLE);
}

// Send character via USART
void usart_send_char(char c) {
    while (!USART_GetFlagStatus(USART1, USART_FLAG_TXE));
    USART_SendData8(USART1, c);
}

// Send string via USART
void usart_send_string(char *str) {
    while (*str) {
        usart_send_char(*str++);
    }
}

// Simple millisecond delay (for 84MHz system clock)
void delay_ms(uint32_t ms) {
    for (; ms > 0; ms--)
        for (volatile uint32_t i = 0; i < 18000; i++);  // Tune this number based on actual clock
}
