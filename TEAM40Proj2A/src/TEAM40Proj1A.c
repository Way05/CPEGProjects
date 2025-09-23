/*
* TEAM40: W. Tan and M. Becker
* CPEG222 Project 1B, 9/9/25
* NucleoF466RE CMSIS Sequence PMOD LEDs with software delays
*/

#include "stm32f4xx.h"
#include <stdbool.h>
#define PIN1 0
#define PIN2 1
#define PIN3 2
#define PIN4 3
#define LED_PORT GPIOC
#define Btn 13
#define FREQ 16000000UL

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

}

void EXTI15_10_IRQHandler(void) {

}
int main() {
    //enabling C clock for our PCx pins for the pmod, also the button
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    //enable SYSCFG clock for EXTI handler
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    //systick timer configuration
    SysTick->LOAD = FREQ / 4 - 1; // this is where 250 ms is calculated and set on the timer
    SysTick->VAL = 0; // this clears the timer to 0 so it can start counting from 0
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
    NVIC_SetPriority(SysTick_IRQn, 1); //setting priority

    //sets up interrupts for the button
    EXTI->IMR |= (1 << Btn); // unmasks EXTI so it can be used
    EXTI->FTSR |= (1 << Btn); // button triggers on falling edge
    SYSCFG->EXTICR[3] &= ~(0xF << (1 * 4)); // clears EXTI bits
    SYSCFG->EXTICR[3] |= (2 << (1 * 4)); // maps ExTI to PC13 button
    NVIC_SetPriority(EXTI15_10_IRQn, 0); // sets priority of the button interrupt to most important
    NVIC_EnableIRQ(EXTI15_10_IRQn); // enables EXTI line interrupt in NVIC

    //runs infinitely so the handlers can run
    //no code needs to be here because the handlers run internally on the board
    while (1) {}

    return 0;
}