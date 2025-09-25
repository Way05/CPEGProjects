/*
* TEAM40: W. Tan and M. Becker
* CPEG222 Project 2A, 9/26/25
* NucleoF466RE CMSIS PMOD Seven Segment Counter
*/

#include "stm32f4xx.h"
#include <stdbool.h>
#define PIN1 0
#define PIN2 1
#define PIN3 2
#define PIN4 3
#define PIN5 4
#define PIN6 1
#define PIN7 0
#define FREQ 16000000UL / 4
#define CAT_FREQ 500
#define COMC_PORT GPIOB
#define COMC_PIN 0

bool tensDigit = false;
int counter = 0;

const unsigned char digitSegments[] = {
    0b0111111, // 0
    0b0000110, // 1
    0b1011011, // 2
    0b1001111, // 3
    0b1100110, // 4
    0b1101101, // 5
    0b1111101, // 6
    0b0000111, // 7
    0b1111111, // 8
    0b1101111, // 9
    0b0000000 // blank/off
};

void SysTick_Handler(void) {
    counter++;
    if (counter > 99) {
        counter = 0;
    }
}

// void EXTI15_10_IRQHandler(void) {}

void TIM2_IRQHandler(void) {
    //first clear segment
    GPIOC->ODR &= (((digitSegments[10] >> 0) & 1) << PIN1); // (1) bitshift and compare the corresponding PMOD segment to get individual bit,
    GPIOC->ODR &= (((digitSegments[10] >> 1) & 1) << PIN2); // (2) then shift into pin
    GPIOC->ODR &= (((digitSegments[10] >> 2) & 1) << PIN3);
    GPIOC->ODR &= (((digitSegments[10] >> 3) & 1) << PIN4);
    GPIOA->ODR &= (((digitSegments[10] >> 4) & 1) << PIN5);
    GPIOA->ODR &= (((digitSegments[10] >> 5) & 1) << PIN6);
    GPIOA->ODR &= (((digitSegments[10] >> 6) & 1) << PIN7);
    if (TIM2->SR & TIM_SR_UIF) { // Check if the update interrupt flag is set
        GPIOB->ODR &= digitSegments[10];
        if (tensDigit) { // If tensDigit is true, update the first digit
            COMC_PORT->ODR |= (1 << COMC_PIN); // Turn on common pin for first digit
            int firstDigit = counter / 10; // Get the first digit
            //if zero display blank
            if (firstDigit == 0) {
                GPIOC->ODR &= (((digitSegments[10] >> 0) & 1) << PIN1);
                GPIOC->ODR &= (((digitSegments[10] >> 1) & 1) << PIN2);
                GPIOC->ODR &= (((digitSegments[10] >> 2) & 1) << PIN3);
                GPIOC->ODR &= (((digitSegments[10] >> 3) & 1) << PIN4);
                GPIOA->ODR &= (((digitSegments[10] >> 4) & 1) << PIN5);
                GPIOA->ODR &= (((digitSegments[10] >> 5) & 1) << PIN6);
                GPIOA->ODR &= (((digitSegments[10] >> 6) & 1) << PIN7);
            }
            else {
                //else display the right number
                GPIOC->ODR |= (((digitSegments[firstDigit] >> 0) & 1) << PIN1);
                GPIOC->ODR |= (((digitSegments[firstDigit] >> 1) & 1) << PIN2);
                GPIOC->ODR |= (((digitSegments[firstDigit] >> 2) & 1) << PIN3);
                GPIOC->ODR |= (((digitSegments[firstDigit] >> 3) & 1) << PIN4);
                GPIOA->ODR |= (((digitSegments[firstDigit] >> 4) & 1) << PIN5);
                GPIOA->ODR |= (((digitSegments[firstDigit] >> 5) & 1) << PIN6);
                GPIOA->ODR |= (((digitSegments[firstDigit] >> 6) & 1) << PIN7);
            }
            tensDigit = !tensDigit; // Toggle digitSelect for next interrupt
            TIM2->SR &= ~TIM_SR_UIF; // Clear the update interrupt flag
        }
        else {
            COMC_PORT->ODR &= ~(1 << COMC_PIN); // Turn on common pin for second digit
            int firstDigit = counter % 10; // Get the second digit
            //display second number
            GPIOC->ODR |= (((digitSegments[firstDigit] >> 0) & 1) << PIN1);
            GPIOC->ODR |= (((digitSegments[firstDigit] >> 1) & 1) << PIN2);
            GPIOC->ODR |= (((digitSegments[firstDigit] >> 2) & 1) << PIN3);
            GPIOC->ODR |= (((digitSegments[firstDigit] >> 3) & 1) << PIN4);
            GPIOA->ODR |= (((digitSegments[firstDigit] >> 4) & 1) << PIN5);
            GPIOA->ODR |= (((digitSegments[firstDigit] >> 5) & 1) << PIN6);
            GPIOA->ODR |= (((digitSegments[firstDigit] >> 6) & 1) << PIN7);
            tensDigit = !tensDigit; // Toggle digitSelect for next interrupt
            TIM2->SR &= ~TIM_SR_UIF; // Clear the update interrupt flag
        }
    }
}

int main() {
    //enabling clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    //enable SYSCFG clock for EXTI handler
    // RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    //clearing and setting mode bits for PMOD pins for output
    GPIOC->MODER &= ~(0x3 << (PIN1 * 2));
    GPIOC->MODER |= (0x1 << (PIN1 * 2));
    GPIOC->MODER &= ~(0x3 << (PIN2 * 2));
    GPIOC->MODER |= (0x1 << (PIN2 * 2));
    GPIOC->MODER &= ~(0x3 << (PIN3 * 2));
    GPIOC->MODER |= (0x1 << (PIN3 * 2));
    GPIOC->MODER &= ~(0x3 << (PIN4 * 2));
    GPIOC->MODER |= (0x1 << (PIN4 * 2));
    GPIOA->MODER &= ~(0x3 << (PIN5 * 2));
    GPIOA->MODER |= (0x1 << (PIN5 * 2));
    GPIOA->MODER &= ~(0x3 << (PIN6 * 2));
    GPIOA->MODER |= (0x1 << (PIN6 * 2));
    GPIOA->MODER &= ~(0x3 << (PIN7 * 2));
    GPIOA->MODER |= (0x1 << (PIN7 * 2));

    //clearing and setting mode bit for CAT pin on PMOD
    COMC_PORT->MODER &= ~(0x3 << (COMC_PIN * 2));
    COMC_PORT->MODER |= (0x1 << (COMC_PIN * 2));

    //systick timer configuration
    SysTick->LOAD = FREQ - 1; // this is where 1s is calculated and set on the timer
    SysTick->VAL = 0; // this clears the timer to 0 so it can start counting from 0
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
    NVIC_SetPriority(SysTick_IRQn, 1); //setting priority

    //TIM2 Timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 clock
    TIM2->PSC = 15; // Prescaler: (16MHz/(15+1) = 1MHz, 1usec period)
    TIM2->ARR = CAT_FREQ - 1; // Auto-reload when CNT = XX: (period = XX usec)
    TIM2->DIER |= TIM_DIER_UIE; // Enable update interrupt
    TIM2->SR &= ~TIM_SR_UIF; // Clear any pending interrupt
    NVIC_EnableIRQ(TIM2_IRQn); // Enable TIM2 interrupt in NVIC
    NVIC_SetPriority(TIM2_IRQn, 1); // Set priority for TIM2
    TIM2->CR1 = TIM_CR1_CEN; // Enable TIM2

    //sets up interrupts for the button
    // EXTI->IMR |= (1 << Btn); // unmasks EXTI so it can be used
    // EXTI->FTSR |= (1 << Btn); // button triggers on falling edge
    // SYSCFG->EXTICR[3] &= ~(0xF << (1 * 4)); // clears EXTI bits
    // SYSCFG->EXTICR[3] |= (2 << (1 * 4)); // maps ExTI to PC13 button
    // NVIC_SetPriority(EXTI15_10_IRQn, 0); // sets priority of the button interrupt to most important
    // NVIC_EnableIRQ(EXTI15_10_IRQn); // enables EXTI line interrupt in NVIC

    return 0;
}