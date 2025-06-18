#include "stm32f4xx.h"
#include <math.h>
#include <stdio.h>
#include "liquidcrystal_i2c.h"

// ==================== Global Variables ====================
volatile uint8_t warning = 0;
volatile uint8_t status = 1;
volatile float max_ppm = 0;
volatile uint32_t systick_counter = 0;
uint32_t blink_interval = 1000; // milliseconds
volatile int temp = 0;


// ==================== Function Prototypes ====================
void uart2_init(void);
void uart2_write_char(char c);
void uart2_write_string(char *str);
void uart1_init(void);
void uart1_write_char(char c);
void uart1_write_string(char *str);

void delay(volatile uint32_t t);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void Delay_Init(void);

void adc_init(void);
uint16_t adc_read(void);

void gpio_init(void);
void SystemClock_Config(void);
void update_blink_interval(float ppm);

void exti4_init(void);
void exti5_init(void);

void I2C1_Init(void);

void EXTI4_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void SysTick_Handler(void);



// ==================== Main Function ====================
int main(void) {
    char buffer[128];
    char buffer1[128];
    float V_adc, V_input, RL, Rs, ppm;
    uint16_t raw;

    uart2_init();
    uart1_init();
    adc_init();
    gpio_init();
    exti5_init();
    exti4_init();
    SystemClock_Config();
    I2C1_Init();
    Delay_Init();

    HD44780_Init(2);
    HD44780_Clear();

    // setup default blink interval
    SysTick_Config(SystemCoreClock / 1000); // 1ms interrupt
    blink_interval = 1000;  // default 1Hz blink


    RL = 10000.0f;
    const float R0 = 21538.0f;
    const float a = -2.54f;
    const float b = 1.91f;


    while (1) {


            raw = adc_read();
            V_adc = (3.3f * raw) / 4095.0f;
            V_input = V_adc * 1.5f;

            if (V_input > 0.01f) {
                Rs = RL * ((5.0f - V_input) / V_input);
            } else {
                Rs = 100000.0f;
            }

            ppm = powf(10.0f, a * log10f(Rs / R0) + b);
            temp = ppm;
        	HD44780_Clear();
        	sprintf(buffer, "ppm: %.1f   ", ppm);
        	HD44780_SetCursor(0, 0);
        	HD44780_PrintStr(buffer);
            if( ppm < 10) {

            	sprintf(buffer1, "Gas: 1  Sys: %d",status);
            	HD44780_SetCursor(0, 1);
            	HD44780_PrintStr(buffer1);

            }
            else if (ppm < 500) {


            	sprintf(buffer1, "Gas: 2  Sys: %d",status);
            	HD44780_SetCursor(0, 1);
            	HD44780_PrintStr(buffer1);

            }
            else {

            	sprintf(buffer1, "Gas: 3  Sys: %d",status);
            	HD44780_SetCursor(0, 1);
            	HD44780_PrintStr(buffer1);
            }
            if (status == 1) {
                GPIOC->BSRR = GPIO_BSRR_BR10;
            delay(1000000);

            if (ppm > max_ppm) {
                max_ppm = ppm;
            }

            update_blink_interval(max_ppm);

            if(max_ppm > 500 && status == 1) {
                uart1_write_string("nong do gas o nguong bao dong( >500ppm)\r\n");
            }

            if (ppm >= 500 ) {
                warning = 1;
            }
            if(ppm >= 200 && ppm <=500  && status == 1) {
            	uart1_write_string("nong do gas o nguong cao( >200ppm)\r\n");
            }

            if (warning == 1) {
                GPIOC->BSRR = GPIO_BSRR_BS12| GPIO_BSRR_BS4;
                GPIOC->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6;

            } else {
            	GPIOC->BSRR = GPIO_BSRR_BR4;
                if (ppm < 10) {
                    GPIOC->BSRR = GPIO_BSRR_BS5;
                    GPIOC->BSRR = GPIO_BSRR_BR6 | GPIO_BSRR_BR8 | GPIO_BSRR_BR12;
                } else if (ppm < 200) {
                    GPIOC->BSRR = GPIO_BSRR_BS6;
                    GPIOC->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR8 | GPIO_BSRR_BR12;
                } else if (ppm < 500) {
                    GPIOC->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6 | GPIO_BSRR_BR12;
                }
            }

        } else {
            sprintf(buffer, "System stop          \r\n");
            GPIOC->BSRR = GPIO_BSRR_BS10;
            GPIOC->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6 | GPIO_BSRR_BR8 | GPIO_BSRR_BR12;
//            uart2_write_string(buffer);
            HD44780_SetCursor(0, 0);
            HD44780_PrintStr(buffer);
            HD44780_SetCursor(0, 1);
            sprintf(buffer1, "Sys : %d         \r\n",status);
            HD44780_PrintStr(buffer1);
            delay(1000000);
        }
    }
}

// ==================== Peripheral Functions ====================
void uart2_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER &= ~(3 << (2 * 2));
    GPIOA->MODER |=  (2 << (2 * 2));
    GPIOA->AFR[0] |= (7 << (4 * 2));

    USART2->BRR = 16000000 / 9600;
    USART2->CR1 |= USART_CR1_TE | USART_CR1_UE;
}

void uart2_write_char(char c) {
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = c;
}

void uart2_write_string(char *str) {
    while (*str) uart2_write_char(*str++);
}

void uart1_init(void) {
    // Bật clock cho GPIOA và USART1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // PA9 -> USART1_TX (AF7)
    GPIOA->MODER &= ~(3 << (9 * 2));
    GPIOA->MODER |=  (2 << (9 * 2)); // Alternate function

    GPIOA->AFR[1] &= ~(0xF << ((9 - 8) * 4));
    GPIOA->AFR[1] |=  (7 << ((9 - 8) * 4)); // AF7 = USART1

    // Baudrate 9600 (giả sử HSI = 16 MHz)
    USART1->BRR = SystemCoreClock / 9600;

    // Bật truyền và USART
    USART1->CR1 |= USART_CR1_TE;
    USART1->CR1 |= USART_CR1_UE;
}


void uart1_write_char(char c) {
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = c;
}

void uart1_write_string(char *str) {
    while (*str) uart1_write_char(*str++);
}


void delay(volatile uint32_t t) {
    while (t--);
}

void Delay_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us) {
    uint32_t cycles = (SystemCoreClock / 1000000L) * us;
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}

void delay_ms(uint32_t ms) {
    while (ms--) delay_us(1000);
}

void adc_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER |= (3 << (0 * 2));
    ADC1->CR2 = 0;
    ADC1->SQR3 = 0;
    ADC1->SMPR2 |= (7 << 0);
    ADC1->CR2 |= ADC_CR2_ADON;
}

uint16_t adc_read(void) {
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return ADC1->DR;
}

void gpio_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOBEN|RCC_AHB1ENR_GPIOAEN;

    GPIOC->MODER |= (1 << (5 * 2)) | (1 << (6 * 2)) | (1 << (8 * 2)) | (1 << (10 * 2)) | (1 << (12 * 2)) | 1 << (4 * 2);

    GPIOB->MODER &= ~((3 << (4 * 2)) | (3 << (5 * 2)));
    GPIOB->PUPDR &= ~((3 << (4 * 2)) | (3 << (5 * 2)));
    GPIOB->PUPDR |=  (1 << (4 * 2)) | (1 << (5 * 2));

    GPIOB->MODER &= ~((3 << (8 * 2)) | (3 << (9 * 2)));
    GPIOB->MODER |=  (2 << (8 * 2)) | (2 << (9 * 2));
    GPIOB->OTYPER |= (1 << 8) | (1 << 9);
    GPIOB->PUPDR &= ~((3 << (8 * 2)) | (3 << (9 * 2)));
    GPIOB->PUPDR |=  (1 << (8 * 2)) | (1 << (9 * 2));
    GPIOB->AFR[1] &= ~((0xF << (0 * 4)) | (0xF << (1 * 4)));
    GPIOB->AFR[1] |=  (4 << (0 * 4)) | (4 << (1 * 4));

}

void SystemClock_Config(void) {
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_HSI;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);
}

void SysTick_Handler(void) {
    systick_counter++;

    if (systick_counter >= blink_interval) {
        systick_counter = 0;
        if ((temp >= 200 && status != 0) | (max_ppm > 500 && status != 0) ) {
            GPIOC->ODR ^= (1 << 8);  // Toggle LED PC8
        } else {
            GPIOC->BSRR = GPIO_BSRR_BR8;  // Turn off LED
        }
    }
}




void update_blink_interval(float ppm) {
    if (ppm < 500) {
        blink_interval = 1000;
    } else if (ppm < 700) {
        blink_interval = 500;
    } else if (ppm < 1000) {
        blink_interval = 333;
    } else if (ppm < 2000) {
        blink_interval = 250;
    } else if (ppm < 5000) {
        blink_interval = 143;
    } else if (ppm < 10000) {
        blink_interval = 100;
    } else {
        blink_interval = 50;
    }
}




void exti4_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    SYSCFG->EXTICR[1] &= ~(0xF << 0);
    SYSCFG->EXTICR[1] |=  (0x1 << 0);

    EXTI->IMR  |= (1 << 4);
    EXTI->FTSR |= (1 << 4);

    NVIC_EnableIRQ(EXTI4_IRQn);
    NVIC_SetPriority(EXTI4_IRQn, 2);
}

void exti5_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    SYSCFG->EXTICR[1] &= ~(0xF << 4);
    SYSCFG->EXTICR[1] |=  (0x1 << 4);

    EXTI->IMR  |= (1 << 5);
    EXTI->FTSR |= (1 << 5);

    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_SetPriority(EXTI9_5_IRQn, 1);
}

void EXTI4_IRQHandler(void) {
    if (EXTI->PR & (1 << 4)) {
        if(status == 1) {
        	EXTI->PR |= (1 << 4);
        	        warning = 0;
        	        max_ppm = 0;
        }
    }
}

void EXTI9_5_IRQHandler(void) {
    if (EXTI->PR & (1 << 5)) {
        EXTI->PR |= (1 << 5);
        status = !status;
    }
}

void I2C1_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    I2C1->CR1 &= ~I2C_CR1_PE;
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;
    I2C1->CR2 = 16;
    I2C1->CCR = 80;
    I2C1->TRISE = 17;
    I2C1->CR1 |= I2C_CR1_PE;
}



