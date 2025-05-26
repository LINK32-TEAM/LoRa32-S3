#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Wire.h"
#include <i2cdetect.h>
#include <ClosedCube_OPT3001.h>
#include <FastLED.h>
#include <U8g2lib.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

ClosedCube_OPT3001 opt3001;
#define OPT3001_ADDRESS 0x45

#define NUM_LEDS 1
constexpr uint8_t LED_PIN = 35; // 重新添加这个常量用于 FastLED
CRGB leds[NUM_LEDS];

typedef struct
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
} board_pins_t;

const board_pins_t board_pins = {
    .button_left_pins = 0,
    .button_right_pins = 36,
    .uart0_tx_pins = 43,
    .uart0_rx_pins = 44,
    .i2c_scl_pins = 48,
    .i2c_sda_pins = 47,
    .led_pins = LED_PIN,
    .battery_adc_pins = 15,
    .lora_io0_pins = 12,
    .lora_io1_pins = 13,
    .lora_io2_pins = 14,
    .lora_io3_pins = 10,
    .lora_io4_pins = 11,
    .lora_en_pins = 17,
    .lora_reset_pins = 18,
    .lora_cs_pins = 21,
    .lora_sck_pins = 16,
    .lora_miso_pins = 33,
    .lora_mosi_pins = 34};

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
uint8_t is_oled_init = 0;

void setup()
{
  Serial.begin(115200);
  // USBSerial.begin(115200);
  Serial.setTxTimeoutMs(0);

  Serial.println(F("Hello Link32!!!"));

  // 初始化I2C并检查结果
  Wire.begin(board_pins.i2c_sda_pins, board_pins.i2c_scl_pins);
  delay(100); // 给I2C总线一些初始化时间

  Serial.println("Scanning I2C bus...");
  i2cdetect(); // default range from 0x03 to 0x77
  is_oled_init = u8g2.begin();

  if (is_oled_init)
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 10, "Hello Link32!");
    u8g2.sendBuffer();
    delay(1000);
  }
  // 尝试初始化BME280并检查结果
  if (!bme.begin(BME280_ADDRESS_ALTERNATE))
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    delay(1000);
  }
  else
  {
    Serial.println("BME280 sensor initialized successfully");
  }

  // 尝试初始化OPT3001并检查结果
  if (0 != opt3001.begin(OPT3001_ADDRESS))
  {
    Serial.println("Could not find a valid OPT3001 sensor, check wiring!");
    delay(1000);
  }
  else
  {
    Serial.println("OPT3001 sensor initialized successfully");
  }

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(50);
}

void print_bme280_values()
{
  // 添加传感器连接检查
  float temp = bme.readTemperature();
  if (isnan(temp))
  {
    Serial.println("Failed to read from BME280 sensor!");
    return;
  }

  Serial.print("Temperature = ");
  Serial.print(temp);
  Serial.println(" °C");

  float pressure = bme.readPressure();
  if (isnan(pressure))
  {
    Serial.println("Failed to read pressure!");
    return;
  }

  Serial.print("Pressure = ");
  Serial.print(pressure / 100.0F);
  Serial.println(" hPa");

  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  Serial.print("Approx. Altitude = ");
  Serial.print(altitude);
  Serial.println(" m");

  float humidity = bme.readHumidity();
  if (isnan(humidity))
  {
    Serial.println("Failed to read humidity!");
    return;
  }

  Serial.print("Humidity = ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.println();
}

void print_opt3001_values()
{
  // 添加错误检查
  struct OPT3001 readings = opt3001.readResult();
  if (readings.error != 0)
  {
    Serial.println("Failed to read from OPT3001 sensor!");
    return;
  }

  Serial.print("LUX Readings = ");
  Serial.println(readings.lux);
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

void display_oled()
{
  if (!is_oled_init)
  {
    return;
  }

  char buf[32]; // 用于格式化字符串

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr); // 使用8像素高的字体

  // 显示温度
  float temp = bme.readTemperature();
  if (!isnan(temp))
  {
    snprintf(buf, sizeof(buf), "Temp: %.1f C", temp);
    u8g2.drawStr(0, 10, buf);
  }

  // 显示湿度
  float humidity = bme.readHumidity();
  if (!isnan(humidity))
  {
    snprintf(buf, sizeof(buf), "Humi: %.1f%%", humidity);
    u8g2.drawStr(0, 24, buf);
  }

  // 显示气压
  float pressure = bme.readPressure() / 100.0F;
  if (!isnan(pressure))
  {
    snprintf(buf, sizeof(buf), "Pres: %.1f hPa", pressure);
    u8g2.drawStr(0, 38, buf);
  }

  // 显示光照
  OPT3001 result = opt3001.readResult();
  if (result.error == 0)
  {
    snprintf(buf, sizeof(buf), "Light: %.1f lux", result.lux);
    u8g2.drawStr(0, 52, buf);
  }

  u8g2.sendBuffer();
}

void loop()
{
  change_led_color();
  print_bme280_values();
  print_opt3001_values();
  display_oled();
  delay(500);
}

void enter_deep_sleep()
{
  // 创建一个非const的数组来存储所有引脚
  uint8_t pins[] = {
      board_pins.button_left_pins,
      board_pins.button_right_pins,
      board_pins.uart0_tx_pins,
      board_pins.uart0_rx_pins,
      board_pins.i2c_scl_pins,
      board_pins.i2c_sda_pins,
      board_pins.led_pins,
      board_pins.battery_adc_pins,
      board_pins.lora_io0_pins,
      board_pins.lora_io1_pins,
      board_pins.lora_io2_pins,
      board_pins.lora_io3_pins,
      board_pins.lora_io4_pins,
      board_pins.lora_en_pins,
      board_pins.lora_reset_pins,
      board_pins.lora_cs_pins,
      board_pins.lora_sck_pins,
      board_pins.lora_miso_pins,
      board_pins.lora_mosi_pins};

  // 遍历数组而不是直接操作结构体
  for (int i = 0; i < sizeof(pins) / sizeof(pins[0]); i++)
  {
    uint8_t pin = pins[i];
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    gpio_hold_en((gpio_num_t)pin);
  }

  // 剩余的特殊处理保持不变
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