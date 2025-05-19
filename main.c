#include "stm32f103xb.h"
#include <stdio.h>

#define RAW_DRY      950
#define RAW_WET      200
#define FILTER_SIZE   8

static uint16_t rainBuf[FILTER_SIZE];
static uint8_t  bufIdx = 0;

//------------------------------------------------------------------------------
// Delay ms dùng SysTick
static void delay_ms(uint32_t ms) {
    // SysTick clock = HCLK
    SysTick->LOAD = SystemCoreClock/1000 * ms - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    // đợi COUNTFLAG
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)==0);
    SysTick->CTRL = 0;
}

//------------------------------------------------------------------------------
// UART1 @115200, TX=PA9, RX=PA10
static void uart1_init(void) {
    // 1) Bật clock GPIOA và USART1
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN  | RCC_APB2ENR_USART1EN;

    // 2) PA9 = AF Push‑Pull, max 50MHz
    GPIOA->CRH &= ~(0xF << (1*4+0));   // clear CNF9/MODE9
    GPIOA->CRH |=  (0xB << (1*4+0));   // MODE9=11, CNF9=10 (AF_PP)

    // 3) PA10 = Input Floating
    GPIOA->CRH &= ~(0xF << (2*4+0));   // MODE10=00
    GPIOA->CRH |=  (0x4 << (2*4+0));   // CNF10=01 (floating)

    // 4) Baudrate = PCLK2 / 115200 = 72MHz/115200 ≈ 625
    USART1->BRR = (SystemCoreClock + 115200/2) / 115200;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE;  // enable TX/RX
    USART1->CR1 |= USART_CR1_UE;                // enable USART
}

static void uart1_putc(char c) {
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = c;
}

static void uart1_puts(const char *s) {
    while (*s) uart1_putc(*s++);
}

int _write(int fd, char *ptr, int len) {
    // retarget printf → USART1
    for (int i = 0; i < len; i++) uart1_putc(ptr[i]);
    return len;
}

//------------------------------------------------------------------------------
// ADC1 trên PA0 (IN0)
static void adc_init(void) {
    // 1) Bật clock ADC1 và GPIOA
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_IOPAEN;
    // 2) PA0 analog: MODE=00, CNF=00
    GPIOA->CRL &= ~(0xF << (0*4));

    // 3) ADC prescaler PCLK2/6 → 12 MHz
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;

    // 4) Bật và hiệu chuẩn ADC
    ADC1->CR2 |= ADC_CR2_ADON;      // bật ADC
    delay_ms(1);
    ADC1->CR2 |= ADC_CR2_RSTCAL;    // reset calibration
    while (ADC1->CR2 & ADC_CR2_RSTCAL);
    ADC1->CR2 |= ADC_CR2_CAL;       // start calibration
    while (ADC1->CR2 & ADC_CR2_CAL);
}

static uint16_t adc_read(void) {
    // Chọn channel 0, sample time 239.5 cycles
    ADC1->SQR3   = 0;
    ADC1->SMPR2 |= ADC_SMPR2_SMP0;  // SMP0 = 111
    ADC1->CR2  |= ADC_CR2_SWSTART;  // start conversion
    while (!(ADC1->SR & ADC_SR_EOC));
    return ADC1->DR;
}

//------------------------------------------------------------------------------
// GPIO: PA1 = DO input, PC13 = LED output (onboard Blue Pill)
static void gpio_init(void) {
    // 1) Bật clock GPIOA, GPIOC
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN;
    // 2) PA1 floating input
    GPIOA->CRL &= ~(0xF << (1*4));
    GPIOA->CRL |=  (0x4 << (1*4));   // CNF=01, MODE=00
    // 3) PC13 push‑pull output 2MHz
    GPIOC->CRH &= ~(0xF << (5*4));
    GPIOC->CRH |=  (0x2 << (5*4));   // MODE13=10, CNF13=00
}

//------------------------------------------------------------------------------
// Moving Average filter
static float rain_filter(uint16_t sample) {
    rainBuf[bufIdx++] = sample;
    if (bufIdx >= FILTER_SIZE) bufIdx = 0;
    uint32_t sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) sum += rainBuf[i];
    return (float)sum / FILTER_SIZE;
}

// Calibration sang %
static float rain_percent(uint16_t v) {
    if (v > RAW_DRY) v = RAW_DRY;
    if (v < RAW_WET) v = RAW_WET;
    return (float)(RAW_DRY - v) * 100.0f / (RAW_DRY - RAW_WET);
}

//------------------------------------------------------------------------------
// Main
int main(void) {
    // Hệ thống clocks đã được SystemInit() gọi trước khi main()
    gpio_init();
    uart1_init();
    adc_init();

    // init filter buffer
    for (int i = 0; i < FILTER_SIZE; i++) rainBuf[i] = RAW_DRY;

    char line[80];
    while (1) {
        uint16_t raw  = adc_read();
        uint8_t  digi = (GPIOA->IDR & (1<<1)) ? 1 : 0;

        float filt = rain_filter(raw);
        float pct  = rain_percent((uint16_t)filt);

        // LED: PC13 HIGH = OFF, LOW = ON (onboard LED active‑low)
        if (digi == 0) GPIOC->BSRR = (1u << 29); // reset bit13
        else          GPIOC->BSRR = (1u << 13); // set bit13

        int l = snprintf(line, sizeof(line),
            "Raw=%u  Filt=%.1f  Pct=%.1f%%  DO=%u\r\n",
            raw, filt, pct, digi);
        // gửi qua UART
        for (int i = 0; i < l; i++) uart1_putc(line[i]);

        delay_ms(500);
    }
}
