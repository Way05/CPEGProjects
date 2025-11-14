// ****************************************************************
// * TEAM40: Tan. Wei-en and Becker, Mia
// * CPEG222 Project 4B
// * NucleoF466RE CMSIS STM32F4xx
// * Simple Line Following
// * Using both motors + IR for navigation
// ****************************************************************

#include "stm32f4xx.h"
#include <stdio.h>
#include <stdbool.h>
#include "SSD_Array.h"

#define FREQUENCY 16000000UL // 16 MHz

#define LEFT_SERVO 8
#define RIGHT_SERVO 9
#define SERVO_PORT GPIOC

#define IR_PORT GPIOC

#define BTN_PIN 13

#define UART_TX_PIN 2
#define UART_RX_PIN 3
#define UART_PORT GPIOA
#define BAUDRATE 115200

volatile uint8_t digitSelect = 0;

volatile bool stop = 1;
volatile bool hash_stop = 0;
volatile int sensor = 0;
int bar_count = 0;

void PWM_Output_PC8_Init(void)
{
    // Enable GPIOC and TIM8 clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;

    // Set PC8 to alternate function (AF3 for TIM8_CH1)
    GPIOC->MODER &= ~(0x3 << (LEFT_SERVO * 2));
    GPIOC->MODER |= (0x2 << (LEFT_SERVO * 2)); // Alternate function
    GPIOC->AFR[1] &= ~(0xF << ((LEFT_SERVO - 8) * 4));
    GPIOC->AFR[1] |= (0x3 << ((LEFT_SERVO - 8) * 4)); // AF3 = TIM8

    // Configure output type and speed for PC8: push-pull, high speed
    GPIOC->OTYPER &= ~(1 << LEFT_SERVO); // push-pull
    GPIOC->OSPEEDR &= ~(0x3 << (LEFT_SERVO * 2));
    GPIOC->OSPEEDR |= (0x2 << (LEFT_SERVO * 2)); // high speed

    // Configure TIM8 for PWM output on CH1 (PC8)
    TIM8->PSC = 15;    // 16 MHz / 16 = 1 MHz timer clock (1us resolution)
    TIM8->ARR = 19999; // Period for 50 Hz
    TIM8->CCR3 = 1500; // Duty cycle (1.475 ms pulse width)
    TIM8->CCMR2 &= ~(TIM_CCMR2_OC3M);
    TIM8->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos); // PWM mode 1
    TIM8->CCMR2 |= TIM_CCMR2_OC3PE;           // Preload enable
    TIM8->CCER |= TIM_CCER_CC3E;              // Enable CH1 output (PC8)

    // For advanced-control timers (TIM8) the main output must be enabled
    // using the BDTR MOE bit or outputs will stay inactive even if CC1E is set.
    TIM8->BDTR |= TIM_BDTR_MOE;
    TIM8->CR1 |= TIM_CR1_ARPE; // Auto-reload preload enable
    TIM8->EGR = TIM_EGR_UG;    // Generate update event
    TIM8->CR1 |= TIM_CR1_CEN;  // Enable timer
}

void PWM_Output_PC9_Init(void)
{
    GPIOC->MODER &= ~(0x3 << (RIGHT_SERVO * 2));
    GPIOC->MODER |= (0x2 << (RIGHT_SERVO * 2)); // Alternate function
    GPIOC->AFR[1] &= ~(0xF << ((RIGHT_SERVO - 8) * 4));
    GPIOC->AFR[1] |= (0x3 << ((RIGHT_SERVO - 8) * 4)); // AF3 = TIM8

    // Configure output type and speed for PC8: push-pull, high speed
    GPIOC->OTYPER &= ~(1 << RIGHT_SERVO); // push-pull
    GPIOC->OSPEEDR &= ~(0x3 << (RIGHT_SERVO * 2));
    GPIOC->OSPEEDR |= (0x2 << (RIGHT_SERVO * 2)); // high speed

    // Configure TIM8 for PWM output on CH1 (PC8)
    TIM8->PSC = 15;    // 16 MHz / 16 = 1 MHz timer clock (1us resolution)
    TIM8->ARR = 19999; // Period for 50 Hz
    TIM8->CCR4 = 1500; // Duty cycle (1.475 ms pulse width)
    TIM8->CCMR2 &= ~(TIM_CCMR2_OC4M);
    TIM8->CCMR2 |= (6 << TIM_CCMR2_OC4M_Pos); // PWM mode 1
    TIM8->CCMR2 |= TIM_CCMR2_OC4PE;           // Preload enable
    TIM8->CCER |= TIM_CCER_CC4E;              // Enable CH1 output (PC8)
}

void servo_angle_set(int pwm1, int pwm2)
{
    TIM8->CCR3 = pwm1;
    TIM8->CCR4 = pwm2;
}

void TIM2_IRQHandler(void)
{ // TIM2 interrupt handler for SSD refresh
    if (TIM2->SR & TIM_SR_UIF)
    {                                        // Check if the update interrupt flag is set
        SSD_update(digitSelect, sensor, 0);  // Update the SSD with the current distance showing hundredths
        digitSelect = (digitSelect + 1) % 4; // Cycle through digitSelect values 0 to 3
        TIM2->SR &= ~TIM_SR_UIF;             // Clear the update interrupt flag
    }
}

int left_servo_width = 1500;
int right_servo_width = 1500;
int previous_state = 0;
int speed_left = 1550;
int speed_right = 1450;
void SysTick_Handler(void)
{
    int IRSensorReading = IR_PORT->IDR & 0x0F;

    if (stop == 0)
    {
        // 1 is not on line, 0 is on line
        switch (IRSensorReading)
        {
        // no line 1111
        case 15:
            left_servo_width = 1500;
            right_servo_width = 1500;
            break;
        // hard left 0001
        case 1:
            left_servo_width = 1600;
            right_servo_width = 1580;
            break;
        // hard left 0111
        case 7:
            left_servo_width = 1600;
            right_servo_width = 1580;
            break;
        // hard right 1000
        case 8:
            left_servo_width = 1400;
            right_servo_width = 1380;

            break;
        // hard right 1110
        case 14:
            left_servo_width = 1400;
            right_servo_width = 1380;

            break;
        // turn right 1100
        case 12:
            left_servo_width = speed_left - 40;
            right_servo_width = speed_right - 10;
            break;
        // turn left 0011
        case 3:
            left_servo_width = speed_left + 10;
            right_servo_width = speed_right + 40;
            break;
        // centered 0110
        case 9:
            left_servo_width = speed_left - 3;
            right_servo_width = speed_right;
            break;
        // stop bar 0000
        case 0:
            if (hash_stop == 0 && stop == 0)
            {
                left_servo_width = speed_left + 40;
                right_servo_width = speed_right - 40;
                hash_stop = 1;
            }
            else if (hash_stop == 1 && previous_state != 0)
            {
                left_servo_width = 1500;
                right_servo_width = 1500;
                stop = 1;
            }
            break;
        }
    }

    previous_state = IRSensorReading;

    // decimal to binary for display
    // inverses bits for 1 line and 0 none
    sensor = 0;
    sensor += (~(IRSensorReading >> 0) & 1);
    sensor += (~(IRSensorReading >> 1) & 1) * 10;
    sensor += (~(IRSensorReading >> 2) & 1) * 100;
    sensor += (~(IRSensorReading >> 3) & 1) * 1000;

    if (stop == 0)
    {
        servo_angle_set(left_servo_width, right_servo_width);
    }
    else
    {
        servo_angle_set(1500, 1500);
    }
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & (1 << BTN_PIN))
    {                               // checks if button is interrupting
        EXTI->PR |= (1 << BTN_PIN); // clear interrupt so it can check again
        stop = 0;
        hash_stop = 0;
    }
}

int main(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    SSD_init();                    // Initialize SSD
    SysTick_Config(FREQUENCY / 4); // Configure for 1ms intervals (1kHz)

    // button setup
    EXTI->IMR |= (1 << BTN_PIN);            // unmasks EXTI so it can be used
    EXTI->FTSR |= (1 << BTN_PIN);           // button triggers on falling edge
    SYSCFG->EXTICR[3] &= ~(0xF << (1 * 4)); // clears EXTI bits
    SYSCFG->EXTICR[3] |= (2 << (1 * 4));    // maps ExTI to PC13 button
    NVIC_SetPriority(EXTI15_10_IRQn, 0);    // sets priority of the button interrupt to most important
    NVIC_EnableIRQ(EXTI15_10_IRQn);         // enables EXTI line interrupt in NVIC

    // Configure TIM2 for 0.5ms interrupt (assuming 16MHz HSI clock)
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 clock to refresh SSD
    TIM2->PSC = 15;                     // Prescaler: (16MHz/16 = 1MHz, 1usec period)
    TIM2->ARR = 499;                    // Auto-reload: 500us (0.5ms period)
    TIM2->DIER |= TIM_DIER_UIE;         // Enable update interrupt
    TIM2->SR &= ~TIM_SR_UIF;            // Clear any pending interrupt
    NVIC_EnableIRQ(TIM2_IRQn);          // Enable TIM2 interrupt in NVIC
    NVIC_SetPriority(TIM2_IRQn, 2);     // Set priority for TIM2
    TIM2->CR1 = TIM_CR1_CEN;            // Enable TIM2

    PWM_Output_PC8_Init();
    PWM_Output_PC9_Init();

    stop = 1;
    hash_stop = 0;

    while (1)
    {
    }
}