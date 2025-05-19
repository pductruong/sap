#include "stm32f401xc.h"

// ================= Delay Microseconds ==================
void delay_us(uint32_t us) {
    // Bật clock cho TIM2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Reset và cấu hình TIM2
    TIM2->CR1 = 0;
    TIM2->PSC = 84 - 1; // 1 MHz nếu chạy 84 MHz
    TIM2->ARR = 0xFFFF;
    TIM2->EGR = TIM_EGR_UG; // Update register
    TIM2->SR = 0;
    TIM2->CR1 |= TIM_CR1_CEN;

    TIM2->CNT = 0;
    while (TIM2->CNT < us);
}

// ================= GPIO Init ==================
void gpio_init() {
    // Bật clock cho GPIOA, GPIOB
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // PA0 output (LED)
    GPIOA->MODER &= ~(3 << (0 * 2));
    GPIOA->MODER |= (1 << (0 * 2)); // Output mode
    GPIOA->OTYPER &= ~(1 << 0);     // Push-pull
    GPIOA->OSPEEDR |= (3 << (0 * 2)); // High speed

    // PB12 input (DHT11)
    GPIOB->MODER &= ~(3 << (12 * 2)); // Input mode
}

// ================= DHT11 Pin Config ==================
void set_pin_output() {
    GPIOB->MODER &= ~(3 << (12 * 2)); // Clear
    GPIOB->MODER |= (1 << (12 * 2));  // Set as output
}

void set_pin_input() {
    GPIOB->MODER &= ~(3 << (12 * 2)); // Set as input
}

void dht_start() {
    set_pin_output();
    GPIOB->ODR &= ~(1 << 12); // Pull low
    delay_us(18000);          // 18 ms
    GPIOB->ODR |= (1 << 12);  // Pull high
    delay_us(20);
    set_pin_input();
}

uint8_t dht_read_bit() {
    while (!(GPIOB->IDR & (1 << 12))); // Wait for high
    delay_us(30);
    uint8_t bit = (GPIOB->IDR & (1 << 12)) ? 1 : 0;
    while (GPIOB->IDR & (1 << 12)); // Wait for low
    return bit;
}

uint8_t dht_read_byte() {
    uint8_t i, result = 0;
    for (i = 0; i < 8; i++) {
        result <<= 1;
        result |= dht_read_bit();
    }
    return result;
}

void dht_read(uint8_t *temp, uint8_t *humi) {
    dht_start();
    delay_us(80); // Wait for response

    // Skip response signals
    while (GPIOB->IDR & (1 << 12));
    while (!(GPIOB->IDR & (1 << 12)));

    uint8_t humi_int = dht_read_byte();
    dht_read_byte(); // humi decimal
    uint8_t temp_int = dht_read_byte();
    dht_read_byte(); // temp decimal
    dht_read_byte(); // checksum

    *temp = temp_int;
    *humi = humi_int;
}

// ================= Main ==================
int main(void) {
    gpio_init();

    uint8_t temp = 0, humi = 0;

    while (1) {
        dht_read(&temp, &humi);

        if (temp > 30) {
            GPIOA->ODR |= (1 << 0); // Turn on LED
        } else {
            GPIOA->ODR &= ~(1 << 0); // Turn off LED
        }

        for (volatile int i = 0; i < 1000000; i++); // Delay loop
    }
}
