#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Wire.h"
#include <i2cdetect.h>
#include "Opt3001.h"
#include <FastLED.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
Opt3001 opt3001;

#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

struct
{
  uint8_t button_left_pins;
  uint8_t button_right_pins;
  uint8_t uart0_tx_pins;
  uint8_t uart0_rx_pins;

  uint8_t i2c_scl_pins;
  uint8_t i2c_sda_pins;
  uint8_t led_pins;
  uint8_t battery_adc_pins;

  uint8_t lora_io0_pins;
  uint8_t lora_io1_pins;
  uint8_t lora_io2_pins;
  uint8_t lora_io3_pins;
  uint8_t lora_io4_pins;

  uint8_t lora_en_pins;
  uint8_t lora_reset_pins;
  uint8_t lora_cs_pins;
  uint8_t lora_sck_pins;
  uint8_t lora_miso_pins;
  uint8_t lora_mosi_pins;
} board_pins = {
    0, 36, 43, 44,
    48, 47, 35, 15,
    12, 13, 14, 10, 11,
    17, 18, 21, 16, 33, 34};

void setup()
{
  Serial.begin(115200);

  Serial.println(F("Hello Link32!!!"));

  Wire.begin(board_pins.i2c_sda_pins, board_pins.i2c_scl_pins);

  i2cdetect(); // default range from 0x03 to 0x77

  // default settings
  bme.begin(BME280_ADDRESS_ALTERNATE);

  opt3001.begin();

  FastLED.addLeds<WS2812, 35, RGB>(leds, NUM_LEDS);
  FastLED.setBrightness(50);
}

void print_bme280_values()
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

void print_opt3001_values()
{
  uint32_t readings = opt3001.readResult();
  Serial.print("LUX Readings = ");
  Serial.println(readings, DEC);
}

void change_led_color()
{
  static uint8_t led_color;
  switch (led_color)
  {
  case 0:
    leds[0] = CRGB::Red;
    break;
  case 1:
    leds[0] = CRGB::Green;
    break;
  case 2:
    leds[0] = CRGB::Blue;
    break;
  default:
    leds[0] = CRGB::Black;
    break;
  }
  FastLED.show();
  led_color++;
  led_color %= 4;
}

void loop()
{

  print_bme280_values();
  print_opt3001_values();
  change_led_color();
  delay(500);
}

void enter_deep_sleep()
{
  uint8_t *pin_p = &board_pins.button_left_pins;
  for (int i = 0; i < sizeof(board_pins); i++)
  {
    uint8_t pin = *pin_p;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);

    gpio_hold_en((gpio_num_t)pin);
    pin_p++;
  }

  gpio_hold_dis((gpio_num_t)board_pins.button_left_pins);
  pinMode(board_pins.button_left_pins, INPUT_PULLUP);
  gpio_hold_en((gpio_num_t)board_pins.button_left_pins);

  gpio_hold_dis((gpio_num_t)board_pins.i2c_scl_pins);
  pinMode(board_pins.i2c_scl_pins, OUTPUT);
  digitalWrite(board_pins.i2c_scl_pins, HIGH);
  gpio_hold_en((gpio_num_t)board_pins.i2c_scl_pins);

  gpio_hold_dis((gpio_num_t)board_pins.i2c_sda_pins);
  pinMode(board_pins.i2c_sda_pins, OUTPUT);
  digitalWrite(board_pins.i2c_sda_pins, HIGH);
  gpio_hold_en((gpio_num_t)board_pins.i2c_sda_pins);

  gpio_hold_dis((gpio_num_t)board_pins.battery_adc_pins);
  pinMode(board_pins.battery_adc_pins, INPUT);

  esp_sleep_enable_ext0_wakeup((gpio_num_t)board_pins.button_left_pins, 0);
  esp_sleep_enable_gpio_wakeup();
  // deep sleep BAT 3.9V current 886uA
  esp_deep_sleep_start();
}