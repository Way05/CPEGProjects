#include <SSD_Array.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "SparkFun_BMI270_Arduino_Library.h"

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
// Cannot use 6 as output for ESP. Pins 6-11 are connected to SPI flash. Use 16 instead.
#define LED_PIN PA0

#define SEALEVELPRESSURE_HPA (1013.25)
#define BME280_ADDRESS (0x76) // Primary I2C Address

#define LED_COUNT 4
// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

int patternCurrent = 0; // Current Pattern Number
int SSD_Display = 0;
int SSD_Decimal = 0;
uint16_t pixelNumber = LED_COUNT; // Total Number of Pixels
// Forward declarations for functions used before their definitions

int digitSelect = 0;

void printValues();
void selectPattern();
bool begin(uint8_t addr = BME280_ADDRESS, TwoWire *theWire = &Wire);
Adafruit_BME280 bme; // I2C
unsigned long delayTime;

HardwareTimer timer(TIM1);

void OnTimer1Interrupt()
{
  SSD_update(digitSelect, SSD_Display, SSD_Decimal);
  digitSelect = (digitSelect + 1) % 4; // Cycle through digitSelect values 0 to 3
}

int lastDebounce;
int debounceDelay = 50;
void buttonISR()
{
  long curr = millis();
  if (curr - lastDebounce > debounceDelay)
  {
    int buttonState = digitalRead(PC13);
    if (buttonState == LOW)
    {

      selectPattern();
      patternCurrent = (patternCurrent + 1) % 4;
      lastDebounce = millis();
    }
  }
}

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ; // time to get serial running
  Serial.println(F("BME280 test"));
  unsigned status;
  // default settings
  status = bme.begin(BME280_ADDRESS);
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status)
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x");
    Serial.println(bme.sensorID(), 16);
    Serial.print(" ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print(" ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print(" ID of 0x60 represents a BME 280.\n");
    Serial.print(" ID of 0x61 represents a BME 680.\n");
    while (1)
      delay(10);
  }
  Serial.println("-- Default Test --");
  delayTime = 1000;
  Serial.println();
  Serial.println("CPEG222 Project 5 - BME280 Values");

  // END of Trinket-specific code.
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all  ASAP
  strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

  SSD_init();

  pinMode(PC13, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PC13), buttonISR, CHANGE);

  // Configure timer
  timer.setPrescaleFactor(15); // Set prescaler to 2564 => timer frequency = 168MHz/2564 = 65522 Hz (from prediv'd by 1 clocksource of 168 MHz)
  timer.setOverflow(500);      // Set overflow to 32761 => timer frequency = 65522 Hz / 32761 = 2 Hz
  timer.attachInterrupt(OnTimer1Interrupt);
  timer.refresh(); // Make register changes take effect
  timer.resume();  // Start
}

void selectPattern()
{
  // Temperature C, red
  if (patternCurrent == 0)
  {
    strip.fill(strip.Color(255, 0, 0), 0, LED_COUNT);
    SSD_Decimal = 2;
    SSD_Display = bme.readTemperature() * 100;
  }
  // Temperature F, purple
  else if (patternCurrent == 1)
  {
    strip.fill(strip.Color(255, 0, 255), 0, LED_COUNT);
    SSD_Decimal = 2;
    SSD_Display = (bme.readTemperature() * 9 / 5 + 32) * 100;
  }
  // Humidity, blue
  else if (patternCurrent == 2)
  {
    strip.fill(strip.Color(0, 0, 255), 0, LED_COUNT);
    SSD_Decimal = 2;
    SSD_Display = bme.readHumidity() * 100;
  }
  else if (patternCurrent == 3)
  // Pressure, green
  {
    strip.fill(strip.Color(0, 255, 0), 0, LED_COUNT);
    SSD_Decimal = 1;
    SSD_Display = bme.readPressure() / 100.0F;
  }
  strip.show();
}

void loop()
{
  printValues();
  delay(delayTime);
}

void printValues()
{
  Serial.print("Temp(C) = ");
  Serial.print(bme.readTemperature());
  Serial.print(", Temp(F) = ");
  Serial.print(bme.readTemperature() * 9 / 5 + 32);
  Serial.print(", RelHum(%) = ");
  Serial.print(bme.readHumidity());
  Serial.print(", Press(atm) = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println();
}