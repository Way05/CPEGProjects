// ****************************************************************
// * TEAM 0: T. LUM and R. MARTIN
// * CPEG222 StopWatch, 8/16/25
// * NucleoF466RE CMSIS STM32F4xx example
// * Display time in tenths of a second on the four digit 7-segment
// * display using SysTick interrupts for counting 100 ms increments
// * Use TIM2 to generate SSD refresh interrupts at 0.5ms intervals
// ****************************************************************
#include "stm32f4xx.h"
#include "SSD_Array.h"
#include <stdbool.h>

#define FREQUENCY 16000000 // HSI clock frequency
#define TRIG_PORT GPIOC
#define TRIG_PIN 4
#define ECHO_PORT GPIOB
#define ECHO_PIN 0

volatile int distance = 0;
volatile int digitSelect = 0;
volatile int currentEdge = 0;

void TIM2_IRQHandler(void)
{
  if (TIM2->SR & TIM_SR_UIF)
  {                                       // Check if the update interrupt flag is set
    SSD_update(digitSelect, distance, 3); // Update the SSD with the current value of milliSec
    digitSelect = (digitSelect + 1) % 4;  // Cycle through digitSelect values 0 to 3
    TIM2->SR &= ~TIM_SR_UIF;              // Clear the update interrupt flag
  }
}

void SysTick_Handler(void)
{
  // This interrupt sends a 10us trigger pulse to the HC-SR04 every 0.5 seconds
  TRIG_PORT->ODR |= (1 << TRIG_PIN); // Set the trigger pin high
  currentEdge = TIM5->CNT;           // Get the current timer count
  while ((TIM5->CNT - currentEdge) < 10)
    ;                                 // Wait for 10us
  TRIG_PORT->ODR &= ~(1 << TRIG_PIN); // Set the trigger pin low
}

int main(void)
{
  // 1. Enable clock for GPIO ports A, B, and C
  SSD_init(); // Initialize SSD GPIO pins

  // 2. Enable TIM2 clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  RCC->APB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->APB1ENR |= RCC_AHB1ENR_GPIOCEN;
  RCC->APB1ENR |= RCC_APB2ENR_SYSCFGEN;

  TRIG_PORT->MODER &= ~(0x3 << (TRIG_PIN * 2));
  TRIG_PORT->MODER |= (0x1 << (TRIG_PIN * 2));
  ECHO_PORT->MODER &= ~(0x3 << (ECHO_PIN * 2));
  ECHO_PORT->MODER |= (0x1 << (ECHO_PIN * 2));

  //   SysTick->LOAD = FREQUENCY/10 -1; 			// Load value for 0.1 second at 16 MHz
  //   SysTick->VAL = 0;         				// Clear current value
  //   SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
  //   NVIC_SetPriority(SysTick_IRQn, 1); 	// Set the SysTick priority (optional)
  SysTick_Config(FREQUENCY / 10); // Configure SysTick for 100 millisecond interrupts

  // 3. Configure TIM2 for 0.5ms interrupt (assuming 16MHz HSI clock)
  TIM2->PSC = 15;                 // Prescaler: (16MHz/16 = 1MHz)
  TIM2->ARR = 499;                // Auto-reload: 1MHz/2000 = 500Hz (0.5ms period)
  TIM2->DIER |= TIM_DIER_UIE;     // Enable update interrupt
  TIM2->SR &= ~TIM_SR_UIF;        // Clear any pending interrupt
  NVIC_EnableIRQ(TIM2_IRQn);      // Enable TIM2 interrupt in NVIC
  NVIC_SetPriority(TIM2_IRQn, 1); // Set priority for TIM2
  TIM2->CR1 = (1 << 0);           // Enable TIM2

  // Configure TIM5 as a free-running timer with each tick equal to 1 msec
  RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; // Enable TIM5 clock
  TIM5->PSC = 15999;                  // Prescaler: (16MHz/(1599+1) = 1kHz, 1msec period)
  TIM5->ARR = 0xFFFFFFFF;             // Auto-reload: Max value for free running (32-bits)
  TIM5->EGR = TIM_EGR_UG;             // Update event generation register
  TIM5->CR1 = TIM_CR1_CEN;            // Enable TIM5

  // 6. Main loop to update the SSD with numbers from 0 to 99
  while (1)
  {
    // A SysTick interrupt will update milliSec every second
    // A TIM2 interrupt will update a SSD every 0.5 ms
  }
}