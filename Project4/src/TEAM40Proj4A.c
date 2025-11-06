// ****************************************************************
// * TEAM40: Tan. Wei-en and Becker, Mia
// * CPEG222 Project 4A
// * NucleoF466RE CMSIS STM32F4xx
// * Single wheel potentiometer RPM control
// * Print data to uart
// ****************************************************************

#include "stm32f4xx.h"
#include "UART2.h"
#include <stdio.h>
#include <stdbool.h>
#include "SSD_Array.h"

#define FREQUENCY 16000000UL // 16 MHz
#define ENCODER_PIN (7)      // Assuming servo motor encoder is connected to GPIOC pin 7
#define ENCODER_PORT (GPIOC)
#define SERVO3_PIN (6) // Assuming servo motor 3 control pin is connected to GPIOC pin 6
#define SERVO3_PORT (GPIOC)

#define BTN_PIN 13

#define UART_TX_PIN 2
#define UART_RX_PIN 3
#define UART_PORT GPIOA
#define BAUDRATE 115200

// port and pin for potentiometer
#define ANALOG_PIN 1
#define ANALOG_PORT GPIOA
#define ADC_CHANNEL 1  // ADC Channel for PA1
#define ADC_SAMPLES 16 // Number of samples for averaging

int offsetDeg = 235;        // this will depend on your setup
int min_pulse_width = 32;   // minimum encoder pulse width in microseconds
int max_pulse_width = 1076; // maximum encoder pulse width in microseconds
int angle = 0;
int cw_pulse_width = 1400;  // 1450 for slower movement;
int ccw_pulse_width = 1600; // 1550 for slower movement;
volatile int current_angle = 0;
volatile int last_angle = 0;
volatile uint32_t last_rising = 0;
// volatile uint32_t before_falling = 0; //initial  falling edge
volatile uint32_t last_falling = 0;
volatile uint32_t pulse_width = 0;
volatile uint8_t waiting_for_falling = 0;
volatile uint8_t digitSelect = 0;

volatile float voltage = 0;
volatile bool pause = false;
volatile bool cw = true;
volatile int total = 0;
volatile float rpm = 0;
volatile int total_angle = 0;

void PWM_Output_PC6_Init(void)
{
    // Enable GPIOC and TIM8 clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;

    // Set PC6 to alternate function (AF3 for TIM8_CH1)
    GPIOC->MODER &= ~(0x3 << (SERVO3_PIN * 2));
    GPIOC->MODER |= (0x2 << (SERVO3_PIN * 2)); // Alternate function
    GPIOC->AFR[0] &= ~(0xF << (SERVO3_PIN * 4));
    GPIOC->AFR[0] |= (0x3 << (SERVO3_PIN * 4)); // AF3 = TIM8

    // Configure output type and speed for PC6: push-pull, high speed
    GPIOC->OTYPER &= ~(1 << SERVO3_PIN); // push-pull
    GPIOC->OSPEEDR &= ~(0x3 << (SERVO3_PIN * 2));
    GPIOC->OSPEEDR |= (0x2 << (SERVO3_PIN * 2)); // high speed

    // Configure TIM8 for PWM output on CH1 (PC6)
    TIM8->PSC = 15;    // 16 MHz / 16 = 1 MHz timer clock (1us resolution)
    TIM8->ARR = 19999; // Period for 50 Hz
    TIM8->CCR1 = 1500; // Duty cycle (1.475 ms pulse width)
    TIM8->CCMR1 &= ~(TIM_CCMR1_OC1M);
    TIM8->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos); // PWM mode 1
    TIM8->CCMR1 |= TIM_CCMR1_OC1PE;           // Preload enable
    TIM8->CCER |= TIM_CCER_CC1E;              // Enable CH1 output (PC6)

    // For advanced-control timers (TIM8) the main output must be enabled
    // using the BDTR MOE bit or outputs will stay inactive even if CC1E is set.
    TIM8->BDTR |= TIM_BDTR_MOE;
    TIM8->CR1 |= TIM_CR1_ARPE; // Auto-reload preload enable
    TIM8->EGR = TIM_EGR_UG;    // Generate update event
    TIM8->CR1 |= TIM_CR1_CEN;  // Enable timer
}

void PWM_Input_PC7_Init(void)
{
    // Enable GPIOC and TIM3 clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    // Set PC7 to alternate function (AF2 for TIM3_CH2)
    GPIOC->MODER &= ~(0x3 << (7 * 2));
    GPIOC->MODER |= (0x2 << (7 * 2)); // Alternate function
    GPIOC->AFR[0] &= ~(0xF << (7 * 4));
    GPIOC->AFR[0] |= (0x2 << (7 * 4)); // AF2 = TIM3

    // Configure TIM3 for simple input capture on CH2 (PC7)
    TIM3->PSC = 15; // 16 MHz / 16 = 1 MHz timer clock (1us resolution)
    TIM3->ARR = 0xFFFF;
    TIM3->CCMR1 &= ~(0x3 << 8);                      // CC2S bits for CH2
    TIM3->CCMR1 |= (0x01 << 8);                      // CC2: IC2 mapped to TI2
    TIM3->CCER &= ~(TIM_CCER_CC2P | TIM_CCER_CC2NP); // Rising edge
    TIM3->CCER |= TIM_CCER_CC2E;                     // Enable capture on CH2
    TIM3->DIER |= TIM_DIER_CC2IE;                    // Enable capture/compare 2 interrupt
    TIM3->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM3_IRQn); // Enable TIM3 interrupt in NVIC
}

volatile int count = 0;
void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_CC2IF)
    {                              // Check if CC2IF is set
        TIM3->SR &= ~TIM_SR_CC2IF; // Clear interrupt flag
        if (!waiting_for_falling)
        {
            last_rising = TIM3->CCR2;
            // Switch to capture falling edge
            TIM3->CCER |= TIM_CCER_CC2P; // Set to falling edge
            waiting_for_falling = 1;
        }
        else
        {
            last_falling = TIM3->CCR2;
            if (last_falling >= last_rising)
            {
                if (last_falling - last_rising < 1100)
                {
                    pulse_width = last_falling - last_rising;

                    //---
                    count++;
                    if (count > 1000000 / 4)
                    {
                        count = 0;
                        total_angle = 0;
                    }
                    // if (pulse_width >= min_pulse_width && pulse_width <= max_pulse_width &&
                    //     !pause && voltage > 0.1)
                    // {
                    // Save previous angle
                    int prev_angle = current_angle;

                    // Calculate new angle position
                    current_angle = (pulse_width - min_pulse_width) * 360 / (max_pulse_width - min_pulse_width);

                    // Calculate angle change with wraparound handling
                    int angle_change;
                    if (cw)
                    {
                        angle_change = current_angle - prev_angle;
                    }
                    else
                    {
                        angle_change = prev_angle - current_angle;
                    }

                    // Handle wraparound cases
                    if (angle_change > 180)
                    {
                        angle_change -= 360; // Going backwards across 0/360 boundary
                    }
                    else if (angle_change < -180)
                    {
                        angle_change += 360; // Going forwards across 0/360 boundary
                    }
                    total_angle += angle_change;
                }
                //---
            }
            else
                pulse_width = (0xFFFF - last_rising) + last_falling + 1;
            // Switch back to capture rising edge
            TIM3->CCER &= ~TIM_CCER_CC2P; // Set to rising edge
            waiting_for_falling = 0;
        }
    }
}

void servo_angle_set(int pwm)
{
    TIM8->CCR1 = pwm;
}

void TIM2_IRQHandler(void)
{ // TIM2 interrupt handler for SSD refresh
    if (TIM2->SR & TIM_SR_UIF)
    {                                         // Check if the update interrupt flag is set
        SSD_update(digitSelect, rpm * 10, 3); // Update the SSD with the current distance showing hundredths
        digitSelect = (digitSelect + 1) % 4;  // Cycle through digitSelect values 0 to 3
        TIM2->SR &= ~TIM_SR_UIF;              // Clear the update interrupt flag
    }
}

void SysTick_Handler(void)
{

    uint32_t total = 0;
    for (int i = 0; i < ADC_SAMPLES; i++)
    {
        ADC1->CR2 |= ADC_CR2_SWSTART; // Start conversion
        while (!(ADC1->SR & ADC_SR_EOC))
            ;              // Wait for conversion to complete
        total += ADC1->DR; // Read data
    }
    uint16_t average = total / ADC_SAMPLES;
    voltage = (average / 4095.0) * 3.3; // Convert to voltage

    volatile int servo_width;

    if (pause)
    {
        servo_width = 1500; // Center/stop position
    }
    else
    {
        if (cw)
        {
            // Map voltage (0-3.3V) to CW range (1500-1300)
            servo_width = 1500 - (voltage * 200.0 / 3.3);
        }
        else
        {
            // Map voltage (0-3.3V) to CCW range (1500-1700)
            servo_width = 1500 + (voltage * 200.0 / 3.3);
        }
    }

    rpm = (total_angle * 60) / 360;

    // Debug output
    uart2_send_string("last: ");
    uart2_send_int32(last_angle);
    uart2_send_string(" current: ");
    uart2_send_int32(current_angle);
    uart2_send_string(" total: ");
    uart2_send_int32(total_angle);
    uart2_send_string("\n");

    // Reset counters
    total_angle = 0;

    uart2_send_string("ADC: ");
    uart2_send_int32(voltage);
    uart2_send_string("\t  dir: ");
    if (cw)
    {
        uart2_send_string(" cw");
    }
    else
    {
        uart2_send_string(" ccw");
    }
    uart2_send_string("\t  servo pulsewidth(us): ");
    uart2_send_int32(servo_width);
    uart2_send_string("\t  rpm: ");
    uart2_send_int32(rpm);
    uart2_send_string("\r\n");

    servo_angle_set(servo_width);
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & (1 << BTN_PIN))
    {                               // checks if button is interrupting
        EXTI->PR |= (1 << BTN_PIN); // clear interrupt so it can check again
        pause = !pause;
        if (!pause)
        {
            cw = !cw;
        }
    }
}

int main(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    UART2_Init();
    SSD_init();                // Initialize SSD
    SysTick_Config(FREQUENCY); // Configure for 1ms intervals (1kHz)

    // Set PA1 (ADC) to analog mode
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    ANALOG_PORT->MODER &= ~(0x3 << (ANALOG_PIN * 2));
    ANALOG_PORT->MODER |= (0x3 << (ANALOG_PIN * 2));
    // Initialize ADC, Default resolution is 12 bits
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;                // Enable ADC1 clock
    ADC1->SQR3 = ADC_CHANNEL;                          // Select channel
    ADC1->SMPR2 = ADC_SMPR2_SMP1_0 | ADC_SMPR2_SMP1_1; // Sample time 56 cycles
    ADC1->CR2 = ADC_CR2_ADON;                          // Enable ADC

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

    PWM_Output_PC6_Init();
    PWM_Input_PC7_Init();
    uart2_send_string("CPEG222 Continuous Servo Demo Program!\r\n");
    uart2_send_string("Setting angle to 0 degrees.\r\n");

    while (1)
    {
    }
}