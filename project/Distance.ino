#include <Wire.h>
#include <MGB_MDYN2.h>                                          // библиотека для моторной платы
Adafruit_PWMServoDriver mdyn2 = Adafruit_PWMServoDriver(0x79);  // адрес платы

#include <VL53L0X.h>  // библиотека для датчика MGS-D20
VL53L0X loxL;
VL53L0X loxR;

#include <MGB_I2C63.h>
// false - для PW548A, true - для PCA9547
MGB_I2C63 mgb_i2c63 = MGB_I2C63(false);
#define DISTL 0x05
#define DISTR 0x03

void setup() {
  // Инициализация последовательного порта
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  delay(100);
  mdyn2.begin();
  // Частота (Гц)
  mdyn2.setPWMFreq(100);
  // Все порты моторной платы выключены
  for (int channel = 0; channel < 16; channel++) {
    mdyn2.setPWM(channel, 0, 0);
  }
  mgb_i2c63.setBusChannel(DISTL);
  loxL.init();
  loxL.setTimeout(500);
  loxL.setMeasurementTimingBudget(20000);
  mgb_i2c63.setBusChannel(DISTR);
  loxR.init();
  loxR.setTimeout(500);
  loxR.setMeasurementTimingBudget(20000);
}


void loop() {
  mgb_i2c63.setBusChannel(DISTL);
  float distL = loxL.readRangeSingleMillimeters();
  mgb_i2c63.setBusChannel(DISTR);
  float distR = loxR.readRangeSingleMillimeters();
  // Вывод измеренных значений в терминал
  Serial.println("Distance left  = " + String(distL, 0) + " mm  ");
  Serial.println("Distance right  = " + String(distR, 0) + " mm  ");
  // изменение скорости моторов в зависимости от разницы показаний датчиков расстояния
  float L = constrain(25 - 0.05 * (distL - distR), 5, 50);
  float R = constrain(25 + 0.05 * (distL - distR), 5, 50);
  mdyn2.motor_setpower(2, L, false);
  mdyn2.motor_setpower(1, R, true);
}
