// Не забудьте положить файл tla2528.h в каталог со скетчем!

#include <Wire.h>  //библиотека для I2C интерфейса

#include "tla2528.h"

#include <Adafruit_PWMServoDriver.h>                          // библиотека для моторной платы
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x79);  // адрес платы

#include <VL53L0X.h>  // библиотека для датчика MGS-D20
VL53L0X lox1;
VL53L0X lox2;

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_APDS9960.h"
Adafruit_MPU6050 mpu;
Adafruit_APDS9960 apds9960;

#define I2C_HUB_ADDR 0x70  // настройки I2C для платы MGB-I2C63EN
#define EN_MASK 0x08
#define DEF_CHANNEL 0x00
#define MAX_CHANNEL 0x08

#define GYRO 0x07
#define DIST1 0x05
#define DIST2 0x03
#define BUZ 0x04
/*
  I2C порт 0x07 - выводы GP16 (SDA), GP17 (SCL)
  I2C порт 0x06 - выводы GP4 (SDA), GP13 (SCL)
  I2C порт 0x05 - выводы GP14 (SDA), GP15 (SCL)
  I2C порт 0x04 - выводы GP5 (SDA), GP23 (SCL)
  I2C порт 0x03 - выводы GP18 (SDA), GP19 (SCL)
*/

struct RGB {
  int red;
  int green;
  int blue;
};
const RGB rgb1 = { 0, 1, 2 };     // 3
const RGB rgb2 = { 5, 4, 3 };     // 8
const RGB rgb3 = { 6, 7, 12 };    // 9
const RGB rgb4 = { 13, 14, 15 };  // 10
int brightness = 25;              /// яркость 0-100

#include <Adafruit_MCP4725.h>  // библиотека для MGB-BUZ1
Adafruit_MCP4725 buzzer;
int vol1 = 1000;
int vol2 = 100;  // разница значений = громкость
int ton;

void setup() {
  Serial.begin(115200);
  // Инициализация драйвера
  Wire.begin();
  pwm.begin();
  // Частота (Гц)
  pwm.setPWMFreq(100);
  // Все порты выключены
  for (int channel = 0; channel < 16; channel++) {
    pwm.setPWM(channel, 0, 0);
  }

  setBusChannel(GYRO);
  if (!mpu.begin(0x69)) {  // (0x68) (также попробуйте просканировать адрес: https://github.com/MAKblC/Codes/tree/master/I2C%20scanner)
    Serial.println("Failed to find MPU6050 chip");
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  if (!apds9960.begin()) {
    Serial.println("Failed to initialize device!");
  }
  Serial.println("CLM60");
  // Инициализация режимов работы датчика
  apds9960.enableColor(true);
  apds9960.enableProximity(true);

  setBusChannel(BUZ);
  buzzer.begin(0x60);           // Без перемычки адрес будет 0x61
  buzzer.setVoltage(0, false);  // выключение звука

  setBusChannel(DIST1);
  lox1.init();
  lox1.setTimeout(500);
  lox1.setMeasurementTimingBudget(20000);
  setBusChannel(DIST2);
  lox2.init();
  lox2.setTimeout(500);
  lox2.setMeasurementTimingBudget(20000);
}

void loop() {
  // включаем моторы по часовой стрелке
  motor_setpower(1, 30, false);
  motor_setpower(2, 30, false);
  delay(2000);
  motor_setpower(1, 30, true);
  motor_setpower(2, 30, true);
  delay(20);
  motor_setpower(1, 0, false);
  motor_setpower(2, 0, false);
  delay(2000);
  // включить светодиоды
  rgb_set(rgb1, 255, 0, 255);  // розовый
  rgb_set(rgb2, 0, 255, 0);    // зеленый
  rgb_set(rgb3, 125, 0, 203);  // фиолетовый
  rgb_set(rgb4, 0, 0, 255);    // синий
  delay(3000);
  // выключить светодиоды
  rgb_set(rgb1, 0, 0, 0);
  rgb_set(rgb2, 0, 0, 0);
  rgb_set(rgb3, 0, 0, 0);
  rgb_set(rgb4, 0, 0, 0);  
  delay(1000);
  // включить звук
  setBusChannel(BUZ);
  buzzer.setVoltage(0, false);  // выключение звука
  note(3, 450);
  note(5, 150);
  note(6, 450);
  // MGS-A6
  setBusChannel(GYRO);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");
  // MGS-D20
  setBusChannel(DIST1);
  float dist1 = lox2.readRangeSingleMillimeters();
  Serial.println("Расстояние 1 = " + String(dist1, 0) + " mm  ");
  setBusChannel(DIST2);
  float dist2 = lox2.readRangeSingleMillimeters();
  Serial.println("Расстояние 2 = " + String(dist2, 0) + " mm  ");
  // MGS-CLM60 
  uint16_t red_data = 0;
  uint16_t green_data = 0;
  uint16_t blue_data = 0;
  uint16_t clear_data = 0;
  uint16_t prox_data = 0;
  while (!apds9960.colorDataReady()) {
    delay(5);
  }
  apds9960.getColorData(&red_data, &green_data, &blue_data, &clear_data);
  // Определение близости препятствия
  prox_data = apds9960.readProximity();
  // Вывод измеренных значений в терминал
  Serial.println("Красный   = " + String(red_data));
  Serial.println("Зеленый = " + String(green_data));
  Serial.println("Синий  = " + String(blue_data));
  Serial.println("Освещенность = " + String(clear_data));
  Serial.println("Приближение  = " + String(prox_data));

  // MGS-CLMLN8
  Serial.println("Датчик линии:");
  getLines();
}

bool setBusChannel(uint8_t i2c_channel)  // смена I2C порта
{
  if (i2c_channel >= MAX_CHANNEL) {
    return false;
  } else {
    Wire.beginTransmission(I2C_HUB_ADDR);
    Wire.write(i2c_channel | EN_MASK);  // для микросхемы PCA9547
                                        // Wire.write(0x01 << i2c_channel); // Для микросхемы PW548A
    Wire.endTransmission();
    return true;
  }
}


void rgb_set(RGB led, byte red, byte green, byte blue) {
  pwm.setPWM(led.red, 0, fabs(red) * 16.05 * brightness / 100);
  pwm.setPWM(led.green, 0, fabs(green) * 16.05 * brightness / 100);
  pwm.setPWM(led.blue, 0, fabs(blue) * 16.05 * brightness / 100);
}

void motor_setpower(int motor, float pwr, bool invert) {
  // Проверка, инвертирован ли мотор
  if (invert) {
    pwr = -pwr;
  }
  // Проверка диапазонов
  if (pwr < -100) {
    pwr = -100;
  }
  if (pwr > 100) {
    pwr = 100;
  }
  int pwmvalue = fabs(pwr) * 40.95;
  if (pwr < 0) {
    if (motor == 1) {
      pwm.setPWM(10, 0, 4096);
      pwm.setPWM(11, 0, pwmvalue);
    }
    if (motor == 2) {
      pwm.setPWM(8, 0, 4096);
      pwm.setPWM(9, 0, pwmvalue);
    }
  } else {
    if (motor == 1) {
      pwm.setPWM(11, 0, 4096);
      pwm.setPWM(10, 0, pwmvalue);
    }
    if (motor == 2) {
      pwm.setPWM(9, 0, 4096);
      pwm.setPWM(8, 0, pwmvalue);
    }
  }
}

void note(int type, int duration) {  // нота (нота, длительность)
  switch (type) {
    case 1: ton = 1000; break;
    case 2: ton = 860; break;
    case 3: ton = 800; break;
    case 4: ton = 700; break;
    case 5: ton = 600; break;
    case 6: ton = 525; break;
    case 7: ton = 450; break;
    case 8: ton = 380; break;
    case 9: ton = 315; break;
    case 10: ton = 250; break;
    case 11: ton = 190; break;
    case 12: ton = 130; break;
    case 13: ton = 80; break;
    case 14: ton = 30; break;
    case 15: ton = 1; break;
  }
  delay(10);  // воспроизведение звука с определенной тональностью и длительностью
  for (int i = 0; i < duration; i++) {
    buzzer.setVoltage(vol1, false);
    buzzer.setVoltage(vol2, false);
    delayMicroseconds(ton);
  }
}

void getLines() {
  for (int i = 0; i < 8; i++) {
    ResetADC();
    ReadRegister(MANUAL_CH_SEL_ADDRESS);
    readAnalogIn(sensors[i]);
    Serial.print(" ");
    Serial.print(returnedValue);
  }
  Serial.println(" ");
}
