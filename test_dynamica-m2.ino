// Не забудьте положить файл tla2528.h в каталог со скетчем!

#include <Wire.h>  //библиотека для I2C интерфейса

#include "MGS_CLMLN8.h"         // библиотека для датчика линии
#include "Adafruit_APDS9960.h"  // Датчик цвета
uint16_t red_data = 0;
uint16_t green_data = 0;
uint16_t blue_data = 0;
uint16_t clear_data = 0;
uint16_t prox_data = 0;

Adafruit_APDS9960 apds9960;
#include <MGB_MDYN2.h>                                          // библиотека для моторной платы
Adafruit_PWMServoDriver mdyn2 = Adafruit_PWMServoDriver(0x79);  // адрес платы

#include <VL53L0X.h>  // библиотека для датчика MGS-D20
VL53L0X lox1;
VL53L0X lox2;

#include <Adafruit_MPU6050.h>  // MGS-A6
#include <Adafruit_Sensor.h>
Adafruit_MPU6050 mpu;

/*
  I2C порт 0x07 - выводы GP16 (SDA), GP17 (SCL)
  I2C порт 0x06 - выводы GP4 (SDA), GP13 (SCL)
  I2C порт 0x05 - выводы GP14 (SDA), GP15 (SCL)
  I2C порт 0x04 - выводы GP5 (SDA), GP23 (SCL)
  I2C порт 0x03 - выводы GP18 (SDA), GP19 (SCL)
*/
#include <MGB_I2C63.h>
MGB_I2C63 mgb_i2c63 = MGB_I2C63(true);
#define GYRO 0x07
#define DIST1 0x05
#define DIST2 0x03

#include <MGB_BUZ1.h>  // библиотека для MGB-BUZ1
Adafruit_MCP4725 buzzer;

void setup() {
  Serial.begin(115200);
  // Инициализация драйвера
  Wire.begin();
  mdyn2.begin();
  // Частота (Гц)
  mdyn2.setPWMFreq(100);
  // Все порты моторной платы выключены
  for (int channel = 0; channel < 16; channel++) {
    mdyn2.setPWM(channel, 0, 0);
  }

  // запуск MGS-A6
  mgb_i2c63.setBusChannel(GYRO);
  if (!mpu.begin(0x69)) {  // (0x68) (также попробуйте просканировать адрес: https://github.com/MAKblC/Codes/tree/master/I2C%20scanner)
    Serial.println("Failed to find MPU6050 chip");
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // запуск датчика цвета
  if (!apds9960.begin()) {
    Serial.println("Failed to initialize device!");
  }
  Serial.println("CLM60");
  // Инициализация режимов работы датчика
  apds9960.enableColor(true);
  apds9960.enableProximity(true);

  // запуск генератора звука
  buzzer.begin(0x60);           // Без перемычки адрес будет 0x61
  buzzer.setVoltage(0, false);  // выключение звука
  buzzer.volume(800);           // громкость (1-999)
  // запуск датчиков расстояния
  mgb_i2c63.setBusChannel(DIST1);
  lox1.init();
  lox1.setTimeout(500);
  lox1.setMeasurementTimingBudget(20000);
  mgb_i2c63.setBusChannel(DIST2);
  lox2.init();
  lox2.setTimeout(500);
  lox2.setMeasurementTimingBudget(20000);
}

void loop() {
  // включаем моторы по часовой стрелке (мотор, скорость, направление)
  mdyn2.motor_setpower(1, 30, false);
  mdyn2.motor_setpower(2, 30, false);
  delay(2000);
  mdyn2.motor_setpower(1, 30, true);
  mdyn2.motor_setpower(2, 30, true);
  delay(20);
  mdyn2.motor_setpower(1, 0, false);
  mdyn2.motor_setpower(2, 0, false);
  delay(2000);
  // включить светодиоды
  mdyn2.rgb_set(1, 0, 255, 0);    // зеленый
  mdyn2.rgb_set(2, 255, 255, 0);  // желтый
  mdyn2.rgb_set(3, 0, 0, 255);    // синий
  mdyn2.rgb_set(4, 255, 0, 0);    // красный
  delay(3000);
  // выключить светодиоды
  mdyn2.rgb_set(1, 0, 0, 0);
  mdyn2.rgb_set(2, 0, 0, 0);
  mdyn2.rgb_set(3, 0, 0, 0);
  mdyn2.rgb_set(4, 0, 0, 0);
  delay(1000);
  // включить звук
  buzzer.setVoltage(0, false);  // выключение звука
  buzzer.note(3, 450);
  buzzer.note(5, 150);
  buzzer.note(6, 450);
  // MGS-A6
  mgb_i2c63.setBusChannel(GYRO);
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
  mgb_i2c63.setBusChannel(DIST1);
  float dist1 = lox2.readRangeSingleMillimeters();
  Serial.println("Расстояние 1 = " + String(dist1, 0) + " mm  ");
  mgb_i2c63.setBusChannel(DIST2);
  float dist2 = lox2.readRangeSingleMillimeters();
  Serial.println("Расстояние 2 = " + String(dist2, 0) + " mm  ");
  // MGS-CLMLN8

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

  Serial.println("Датчик линии:");
  getLines();
}

// считывание датчика линии
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
