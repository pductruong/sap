#include "stm32f4xx.h"

// Cấu hình các tham số của sensor/filter/calibration
#define CALIB_SAMPLES    100
#define FILTER_SIZE      16
#define THRESHOLD_RAW    1000

static uint32_t offset_dark = 0;
static uint16_t filter_buf[FILTER_SIZE];
static uint32_t filter_sum = 0;
static uint8_t  filter_idx = 0;

// Prototype
uint16_t ADC1_Read(void);

// 1) Calibration: đo offset tối khi khởi động
void ADC_Calibrate(void) {
    uint32_t sum = 0;
    for (uint32_t i = 0; i < CALIB_SAMPLES; i++) {
        sum += ADC1_Read();
    }
    offset_dark = sum / CALIB_SAMPLES;
}

// 2) Moving average filter để mượt dữ liệu
uint16_t Filter_MovingAverage(uint16_t raw) {
    filter_sum -= filter_buf[filter_idx];
    uint16_t adj = (raw > offset_dark) ? (raw - offset_dark) : 0;
    filter_buf[filter_idx] = adj;
    filter_sum += adj;
    filter_idx = (filter_idx + 1) % FILTER_SIZE;
    return (uint16_t)(filter_sum / FILTER_SIZE);
}

// 3) Cấu hình GPIOA PA0 làm ADC, GPIOC PC13 làm LED
void ADC_GPIO_Config(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER   |=  GPIO_MODER_MODER0;  // PA0 analog
    GPIOA->PUPDR   &= ~GPIO_PUPDR_PUPDR0;  // no pull-up/down
}

void LED_GPIO_Config(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER   &= ~GPIO_MODER_MODER13;
    GPIOC->MODER   |=  GPIO_MODER_MODER13_0; // PC13 output
}

// 4) Cấu hình ADC1 để đọc kênh 0
void ADC1_Config(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC->CCR      |= ADC_CCR_ADCPRE_0;      // PCLK2/4 = ~21MHz
    ADC1->CR1      = 0;
    ADC1->CR2      = ADC_CR2_ADON;         // Bật ADC
    ADC1->SMPR2   |= ADC_SMPR2_SMP0_2      // Sample time 144 cyc
                    | ADC_SMPR2_SMP0_1;
    ADC1->SQR1    &= ~ADC_SQR1_L;          // 1 conversion
    ADC1->SQR3     = 0;                    // Kênh 0 = PA0
}

// 5) Đọc một mẫu ADC
uint16_t ADC1_Read(void) {
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return (uint16_t)(ADC1->DR & 0x0FFF);
}

// 6) Hàm chính
int main(void) {
    // Khởi tạo giao tiếp sensor và LED
    ADC_GPIO_Config();
    LED_GPIO_Config();
    ADC1_Config();

    // Hiệu chuẩn offset (dark level)
    ADC_Calibrate();

    // Reset filter
    for (uint8_t i = 0; i < FILTER_SIZE; i++) filter_buf[i] = 0;
    filter_sum = filter_idx = 0;

    while (1) {
        uint16_t raw    = ADC1_Read();
        uint16_t smooth = Filter_MovingAverage(raw);

        // Nếu ánh sáng thấp (smooth < ngưỡng), bật LED, ngược lại tắt
        if (smooth < THRESHOLD_RAW) {
            GPIOC->BSRR = GPIO_BSRR_BS13;  // SET PC13
        } else {
            GPIOC->BSRR = GPIO_BSRR_BR13;  // RESET PC13
        }
    }
}
