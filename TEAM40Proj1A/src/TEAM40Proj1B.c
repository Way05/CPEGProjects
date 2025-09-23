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

int delayNum = 500000;
int increment = PIN1;
bool PAUSE = false;
bool SHIFT_LEFT = false;

//called every 250ms
void SysTick_Handler(void) {
    //if statement updates led status
    if(!PAUSE) {
        //turn off all leds before updating to next one
        LED_PORT->ODR &= (0 << PIN1);
        LED_PORT->ODR &= (0 << PIN2);
        LED_PORT->ODR &= (0 << PIN3);
        LED_PORT->ODR &= (0 << PIN4);

        LED_PORT->ODR ^= (1 << increment % 4);
        if (!SHIFT_LEFT){
            increment++;
        } else if (SHIFT_LEFT){
            increment--;
            increment += 4;
        }
    }
}

void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR & (1 << Btn)) { // checks if button is interrupting
        EXTI->PR |= (1 << Btn); // clear interrupt so it can check again
        //if statement below toggles pause and shift direction
        if (!PAUSE) {
            PAUSE = true;
        } else {
            PAUSE = false;
            SHIFT_LEFT = !SHIFT_LEFT;
            //this fixes the led moving an extra spot before reversing
            if(SHIFT_LEFT) {
                increment--;
            } else {
                increment++;
            }
        }
    }
}
int main() {
    //enabling C clock for our PCx pins for the pmod, also the button
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    //enable SYSCFG clock for EXTI handler
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    //clearing and setting mode bits for PMOD pins for output
    LED_PORT->MODER &= ~(0x3 << (PIN1 * 2));
    LED_PORT->MODER |= (0x1 << (PIN1 * 2));
    LED_PORT->MODER &= ~(0x3 << (PIN2 * 2));
    LED_PORT->MODER |= (0x1 << (PIN2 * 2));
    LED_PORT->MODER &= ~(0x3 << (PIN3 * 2));
    LED_PORT->MODER |= (0x1 << (PIN3 * 2));
    LED_PORT->MODER &= ~(0x3 << (PIN4 * 2));
    LED_PORT->MODER |= (0x1 << (PIN4 * 2));

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
    while(1) {}

    return 0;
}