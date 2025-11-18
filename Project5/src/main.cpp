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
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
// NEO_KHZ800 800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
// NEO_KHZ400 400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
// NEO_GRB Pixels are wired for GRB bitstream (most NeoPixel products)
// NEO_RGB Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
// NEO_RGBW Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
unsigned long pixelPrevious = 0;   // Previous Pixel Millis
unsigned long patternPrevious = 0; // Previous Pattern Millis
int patternCurrent = 0;            // Current Pattern Number
int patternInterval = 5000;        // Pattern Interval (ms)
bool patternComplete = false;
int pixelInterval = 50;           // Pixel Interval (ms)
int pixelQueue = 0;               // Pattern Pixel Queue
int pixelCycle = 0;               // Pattern Pixel Cycle
uint16_t pixelNumber = LED_COUNT; // Total Number of Pixels
// Forward declarations for functions used before their definitions

void printValues();
bool begin(uint8_t addr = BME280_ADDRESS, TwoWire *theWire = &Wire);
Adafruit_BME280 bme; // I2C
unsigned long delayTime;

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
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)
}

void loop()
{
  printValues();
  delay(delayTime);

  strip.rainbow();
  strip.show();
}

void printValues()
{
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" °C");
  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");
  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");
  Serial.println();
}