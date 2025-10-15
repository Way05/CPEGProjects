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

volatile int milliSec = 0;
volatile int digitSelect = 0;

void TIM2_IRQHandler(void){
  if(TIM2->SR & TIM_SR_UIF){ // Check if the update interrupt flag is set
    SSD_update(digitSelect, milliSec, 3); // Update the SSD with the current value of milliSec
    digitSelect = (digitSelect + 1) % 4; // Cycle through digitSelect values 0 to 3
    TIM2->SR &= ~TIM_SR_UIF; // Clear the update interrupt flag
  }
}

void SysTick_Handler(void) {
  milliSec++; // Increment the counter if SysTick interrupt occurs
  if (milliSec > 9999) {
      milliSec = 0; // Reset to 0 after reaching 9999
  }
}

int main(void) {
  // 1. Enable clock for GPIO ports A, B, and C
  SSD_init(); // Initialize SSD GPIO pins

  // 2. Enable TIM2 clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

//   SysTick->LOAD = FREQUENCY/10 -1; 			// Load value for 0.1 second at 16 MHz
//   SysTick->VAL = 0;         				// Clear current value
//   SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
//   NVIC_SetPriority(SysTick_IRQn, 1); 	// Set the SysTick priority (optional)
  SysTick_Config(FREQUENCY/10); // Configure SysTick for 100 millisecond interrupts

  // 3. Configure TIM2 for 0.5ms interrupt (assuming 16MHz HSI clock)
  TIM2->PSC = 15; // Prescaler: (16MHz/16 = 1MHz)
  TIM2->ARR = 499; // Auto-reload: 1MHz/2000 = 500Hz (0.5ms period)
  TIM2->DIER |= TIM_DIER_UIE; // Enable update interrupt
  TIM2->SR &= ~TIM_SR_UIF; // Clear any pending interrupt
  NVIC_EnableIRQ(TIM2_IRQn); // Enable TIM2 interrupt in NVIC
  NVIC_SetPriority(TIM2_IRQn, 1); // Set priority for TIM2
  TIM2->CR1 = (1 << 0); // Enable TIM2

  // 5. Set display to 0 initially
  milliSec = 0; // Start with 0

  // 6. Main loop to update the SSD with numbers from 0 to 99
  while (1) {
      // A SysTick interrupt will update milliSec every second
      // A TIM2 interrupt will update a SSD every 0.5 ms
  }
}