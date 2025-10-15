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
#define TRIG_PORT GPIOA
#define TRIG_PIN 4
#define ECHO_PORT GPIOB
#define ECHO_PIN 0

#define UART_TX_PIN 2 // PA2
#define UART_RX_PIN 3 // PA3
#define UART_PORT GPIOA
#define BAUDRATE 115200

#define Btn 13

void uart_sendChar(char c)
{
  while (!(USART2->SR & USART_SR_TXE))
    ; // Wait until transmit data register is empty
  USART2->DR = c;
}

void uart_sendString(const char *str)
{
  while (*str)
  {
    uart_sendChar(*str++);
  }
}

bool isCM = true;
volatile float distance = 0;
volatile int digitSelect = 0;
volatile int currentEdge = 0;
volatile int before = 0;
volatile int after = 0;

void TIM2_IRQHandler(void)
{
  if (TIM2->SR & TIM_SR_UIF)
  {                                       // Check if the update interrupt flag is set
    SSD_update(digitSelect, distance, 2); // Update the SSD with the current value of milliSec
    digitSelect = (digitSelect + 1) % 4;  // Cycle through digitSelect values 0 to 3
    TIM2->SR &= ~TIM_SR_UIF;              // Clear the update interrupt flag
  }
}

void SysTick_Handler(void)
{
  // Send a 10us trigger pulse to the HC-SR04 every 0.5 seconds
  TRIG_PORT->ODR |= (1 << TRIG_PIN); // Set the trigger pin high
  currentEdge = TIM5->CNT;           // Get the current timer count
  while ((TIM5->CNT - currentEdge) < 10)
    ;                                 // Wait for 10us
  TRIG_PORT->ODR &= ~(1 << TRIG_PIN); // Set the trigger pin low

  // Wait for echo pin to go high (start of echo pulse)
  while (!(ECHO_PORT->IDR & (1 << ECHO_PIN)))
    ;
  before = TIM5->CNT; // Record timer value at rising edge

  // Wait for echo pin to go low (end of echo pulse)
  while (ECHO_PORT->IDR & (1 << ECHO_PIN))
    ;
  after = TIM5->CNT; // Record timer value at falling edge

  int pulse_width = after - before;

  if (isCM)
  {
    // Calculate distance in cm
    distance = pulse_width / 58.3;
  }
  else
  {
    // Calculate distance in in
    distance = pulse_width / 148.1;
  }

  if (distance > 99.99)
  {
    distance = 99.99;
  }

  distance *= 100;

  char str[32];
  if (isCM)
  {
    sprintf(str, "Dist: %.2f cm\r\n", distance / 100);
  }
  else
  {
    sprintf(str, "Dist: %.2f in\n", distance / 100);
  }
  uart_sendString(str);
}

void EXTI15_10_IRQHandler(void)
{
  if (EXTI->PR & (1 << Btn))
  {                         // checks if button is interrupting
    EXTI->PR |= (1 << Btn); // clear interrupt so it can check again
    isCM = !isCM;
  }
}

int main(void)
{
  SSD_init(); // Initialize SSD GPIO pins

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  TRIG_PORT->MODER &= ~(0x3 << (TRIG_PIN * 2));
  TRIG_PORT->MODER |= (0x1 << (TRIG_PIN * 2));
  ECHO_PORT->MODER &= ~(0x3 << (ECHO_PIN * 2));
  ECHO_PORT->MODER |= (0x1 << (TRIG_PIN * 2));

  //   SysTick->LOAD = FREQUENCY/10 -1; 			// Load value for 0.1 second at 16 MHz
  //   SysTick->VAL = 0;         				// Clear current value
  //   SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
  //   NVIC_SetPriority(SysTick_IRQn, 1); 	// Set the SysTick priority (optional)
  SysTick_Config(FREQUENCY / 2); // Configure SysTick for 500 millisecond interrupts

  EXTI->IMR |= (1 << Btn);                // unmasks EXTI so it can be used
  EXTI->FTSR |= (1 << Btn);               // button triggers on falling edge
  SYSCFG->EXTICR[3] &= ~(0xF << (1 * 4)); // clears EXTI bits
  SYSCFG->EXTICR[3] |= (2 << (1 * 4));    // maps ExTI to PC13 button
  NVIC_SetPriority(EXTI15_10_IRQn, 0);    // sets priority of the button interrupt to most important
  NVIC_EnableIRQ(EXTI15_10_IRQn);         // enables EXTI line interrupt in NVIC

  // 3. Configure TIM2 for 0.5ms interrupt (assuming 16MHz HSI clock)
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  TIM2->PSC = 15;                 // Prescaler: (16MHz/16 = 1MHz)
  TIM2->ARR = 499;                // Auto-reload: 1MHz/2000 = 500Hz (0.5ms period)
  TIM2->DIER |= TIM_DIER_UIE;     // Enable update interrupt
  TIM2->SR &= ~TIM_SR_UIF;        // Clear any pending interrupt
  NVIC_EnableIRQ(TIM2_IRQn);      // Enable TIM2 interrupt in NVIC
  NVIC_SetPriority(TIM2_IRQn, 1); // Set priority for TIM2
  TIM2->CR1 = (1 << 0);           // Enable TIM2

  // Configure TIM5 as a free-running timer with each tick equal to 1 us
  RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; // Enable TIM5 clock
  TIM5->PSC = 15;                     // Prescaler: (16MHz/(15+1) = 1MHz) -> 1us ticks
  TIM5->ARR = 0xFFFFFFFF;             // Auto-reload: Max value for free running (32-bits)
  TIM5->EGR = TIM_EGR_UG;             // Update event generation register
  TIM5->CR1 = TIM_CR1_CEN;            // Enable TIM5

  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  // Set PA2 (TX) and PA3 (RX) to alternate function
  GPIOA->MODER &= ~((3 << (UART_TX_PIN * 2)) | (3 << (UART_RX_PIN * 2))); // Clear mode bits
  GPIOA->MODER |= (2 << (UART_TX_PIN * 2)) | (2 << (UART_RX_PIN * 2));    // AF mode
  GPIOA->AFR[0] |= (7 << (UART_TX_PIN * 4)) | (7 << (UART_RX_PIN * 4));   // AF7 for USART2
  // Configure USART2: 9600 baud, 8N1, enable TX and RX
  USART2->BRR = FREQUENCY / BAUDRATE;                       // Assuming 16 MHz clock
  USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // Enable TX, RX, USART
  uart_sendString("CPEG222 Project 3 Part 1\r\n");

  // 6. Main loop to update the SSD with numbers from 0 to 99
  while (1)
  {
    // A SysTick interrupt will update milliSec every second
    // A TIM2 interrupt will update a SSD every 0.5 ms
  }
}