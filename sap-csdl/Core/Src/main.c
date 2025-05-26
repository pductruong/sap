#include "stm32f4xx.h"
#include <stdio.h>
#define MEDIAN_WINDOW 5

uint8_t temp_buffer[MEDIAN_WINDOW] = {0};
uint8_t temp_index = 0;
uint8_t temperature_median = 0;

float temperature_filtered = 0.0f;
#define EMA_ALPHA 0.1f

// Biến toàn cục lưu dữ liệu DHT11
uint8_t DHT11_Data[5];

// ---------- Delay ----------
void TIM2_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 16 - 1;        // 16 MHz / 16 = 1 MHz (1 µs/tick)
    TIM2->ARR = 0xFFFF;
    TIM2->CNT = 0;
    TIM2->CR1 = 0;
    TIM2->CR1 |= TIM_CR1_CEN;
}

void delay_us(uint32_t us) {
    if (us == 0 || us > 0xFFFF) return;
    TIM2->CNT = 0;
    while (TIM2->CNT < us);
}

// ---------- UART ----------
void USART2_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER &= ~(3 << (2 * 2));
    GPIOA->MODER |= (2 << (2 * 2));
    GPIOA->AFR[0] &= ~(0xF << (2 * 4));
    GPIOA->AFR[0] |= (7 << (2 * 4));

    USART2->BRR = (104 << 4) | 3;
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
}

void USART2_WriteChar(char c) {
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = c;
}

void USART2_WriteString(char* str) {
    while (*str) USART2_WriteChar(*str++);
}

// ---------- GPIO ----------
void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(3 << (1 * 2));
    GPIOA->PUPDR &= ~(3 << (1 * 2));
}

void DHT11_SetPinOutput(void) {
    GPIOA->MODER &= ~(3 << (1 * 2));
    GPIOA->MODER |= (1 << (1 * 2));
}

void DHT11_SetPinInput(void) {
    GPIOA->MODER &= ~(3 << (1 * 2));
}

// ---------- DHT11 ----------
uint8_t DHT11_Read(void) {
    uint8_t i, j;

    DHT11_SetPinOutput();
    GPIOA->ODR &= ~(1 << 1);
    delay_us(18000);
    GPIOA->ODR |= (1 << 1);
    delay_us(30);
    DHT11_SetPinInput();

    uint32_t timeout = 20000;
    while (GPIOA->IDR & (1 << 1)) {
        if (--timeout == 0) {
            USART2_WriteString("Loi: Khong thay xung thap phan hoi\r\n");
            return 1;
        }
    }
    timeout = 20000;
    while (!(GPIOA->IDR & (1 << 1))) {
        if (--timeout == 0) {
            USART2_WriteString("Loi: Xung thap phan hoi qua lau\r\n");
            return 2;
        }
    }
    timeout = 20000;
    while (GPIOA->IDR & (1 << 1)) {
        if (--timeout == 0) {
            USART2_WriteString("Loi: Xung cao phan hoi qua lau\r\n");
            return 3;
        }
    }

    for (j = 0; j < 5; j++) {
        DHT11_Data[j] = 0;
        for (i = 0; i < 8; i++) {
            while (!(GPIOA->IDR & (1 << 1)));
            delay_us(30);
            if (GPIOA->IDR & (1 << 1)) {
                DHT11_Data[j] |= (1 << (7 - i));
            }
            while (GPIOA->IDR & (1 << 1));
        }
    }

    uint8_t sum = DHT11_Data[0] + DHT11_Data[1] + DHT11_Data[2] + DHT11_Data[3];
    if (sum != DHT11_Data[4]) return 4;

    return 0;
}

void update_median_filter(uint8_t new_value) {
    temp_buffer[temp_index] = new_value;
    temp_index = (temp_index + 1) % MEDIAN_WINDOW;

    uint8_t sorted[MEDIAN_WINDOW];
    for (int i = 0; i < MEDIAN_WINDOW; i++) {
        sorted[i] = temp_buffer[i];
    }

    for (int i = 0; i < MEDIAN_WINDOW - 1; i++) {
        for (int j = i + 1; j < MEDIAN_WINDOW; j++) {
            if (sorted[i] > sorted[j]) {
                uint8_t temp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = temp;
            }
        }
    }

    temperature_median = sorted[MEDIAN_WINDOW / 2];
}

// ---------- Main ----------
int main(void) {
    GPIO_Init();
    TIM2_Init();
    USART2_Init();

    char buffer[128];

    while (1) {
        uint8_t status = DHT11_Read();

        if (status == 0) {
            update_median_filter(DHT11_Data[2]);
            sprintf(buffer, "Nhiet do: %d°C | Trung vi: %d°C | Do am: %d%%\r\n",
                    DHT11_Data[2], temperature_median, DHT11_Data[0]);
        } else if (status == 1 || status == 2 || status == 3) {
            sprintf(buffer, "Loi: Xem chi tiet tren\r\n");
        } else {
            sprintf(buffer, "Loi checksum! Data: %d,%d,%d,%d,%d\r\n",
                    DHT11_Data[0], DHT11_Data[1], DHT11_Data[2], DHT11_Data[3], DHT11_Data[4]);
        }

        USART2_WriteString(buffer);
        for (volatile int i = 0; i < 1000000; i++);
    }
}
