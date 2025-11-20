#include <SSD_Array.h>
#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "SparkFun_BMI270_Arduino_Library.h"
#include <Adafruit_NeoPixel.h>

#define LED_PIN PA0
#define BTN_PIN PC13
#define SEALEVELPRESSURE_HPA (1013.25)
#define BME280_ADDRESS (0x76) // Primary I2C Address
#define LED_COUNT 4
// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_BME280 bme; // I2C

int patternCurrent = 0; // Current Pattern Number
int SSD_Display = 0;
int SSD_Decimal = 0;
uint16_t pixelNumber = LED_COUNT; // Total Number of Pixels
int digitSelect = 0;
int lastDebounce;
int debounceDelay = 50;
unsigned long delayTime = 1000;
long uartPrevDelay;

void printValues();
void selectPattern();
bool begin(uint8_t addr = BME280_ADDRESS, TwoWire *theWire = &Wire);
HardwareTimer timer(TIM1);

void OnTimer1Interrupt()
{
  SSD_update(digitSelect, SSD_Display, SSD_Decimal);
  digitSelect = (digitSelect + 1) % 4;
}

void buttonISR()
{
  long curr = millis();
  if (curr - lastDebounce > debounceDelay)
  {
    int buttonState = digitalRead(BTN_PIN);
    if (buttonState == LOW)
    {
      patternCurrent = (patternCurrent + 1) % 4;
      selectPattern();
      lastDebounce = millis();
    }
  }
}

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ; // time to get serial running
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
  Serial.println("CPEG222 Project 5 - BME280 Values");

  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all  ASAP
  strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

  SSD_init();

  pinMode(BTN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_PIN), buttonISR, CHANGE);

  // Configure timer
  timer.setPrescaleFactor(15); // Set prescaler to 15
  timer.setOverflow(500);      // Set overflow to 500
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
  long currUartDelay = millis();
  if (currUartDelay - uartPrevDelay > 5000)
  {
    printValues();
    uartPrevDelay = millis();
  }
  selectPattern();
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