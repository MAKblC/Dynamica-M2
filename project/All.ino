#include <Wire.h>
#include <MGB_MDYN2.h>                                          // библиотека для моторной платы
Adafruit_PWMServoDriver mdyn2 = Adafruit_PWMServoDriver(0x79);  // адрес платы

#include "MGS_CLMLN8.h"

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;
uint8_t fifoBuffer[45];  // буфер
int turnSpeed = 15;      // скорость поворота
int compensation = 2;    // компенсация поворота (зависит от скорости поворота)

#include <VL53L0X.h>  // библиотека для датчика MGS-D20
VL53L0X loxL;
VL53L0X loxR;

#include <MGB_I2C63.h>
// false - для PW548A, true - для PCA9547
MGB_I2C63 mgb_i2c63 = MGB_I2C63(false);
#define DISTL 0x05
#define DISTR 0x03
#define GYRO 0x07

#include "Adafruit_APDS9960.h"
Adafruit_APDS9960 apds9960;

float speedDino = 18;  // скорость Робота
int data[8];
byte digitData[8];

int l = 1700;  // граница черного/белого
// коэффициенты ускорения/замедления при смещении линии
int Kf[] = { 120, 90, 50, 15, -15, -50, -90, -120 };
int findKf[8];
int maxKf = 0, minKf = 0;
float Lkf, Rkf;


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

  if (!apds9960.begin()) {
    Serial.println("Failed to initialize device!");
  }
  // Инициализация режимов работы датчика
  apds9960.enableColor(true);

  mgb_i2c63.setBusChannel(GYRO);
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
}

void loop() {
  Serial.println(getColor());  // вывод цвета
  mgb_i2c63.setBusChannel(DISTL);
  float distL = loxL.readRangeSingleMillimeters();
  mgb_i2c63.setBusChannel(DISTR);
  float distR = loxR.readRangeSingleMillimeters();
  // если видим в ближайшем пространстве стены
  if (distL < 500 and distR < 500) {
    Serial.println("Distance left  = " + String(distL, 0) + " mm  ");
    Serial.println("Distance right  = " + String(distR, 0) + " mm  ");
    float L = constrain(20 - 0.05 * (distL - distR), 5, 50);
    float R = constrain(20 + 0.05 * (distL - distR), 5, 50);
    mdyn2.motor_setpower(2, L, false);
    mdyn2.motor_setpower(1, R, true);
  } else {
    // в ином случае ищем линию
    if (digitSensor() == "GO") {  // если видим линию, то просто едем с выключенными светодиодами
      addKf();
      for (int i = 1; i < 5; i++) {
        mdyn2.rgb_set(i, 0, 0, 0);
      }
      // если путаница с линией, то едем прямо
      if (-minKf == maxKf) {
        mdyn2.motor_setpower(1, speedDino, true);
        mdyn2.motor_setpower(2, speedDino, false);
      } else {
        maxKf = maxKf < 0 ? 0 : maxKf;
        minKf = minKf > 0 ? 0 : minKf;
        // расчет коэффицента ускорения/ослабления скорости мотора
        Lkf = speedDino / 100 * (float)minKf;
        Rkf = speedDino / 100 * (float)maxKf;
        //Serial.println(String(speedDino - Rkf) + " / " + String(speedDino + Lkf));
        mdyn2.motor_setpower(2, speedDino - Rkf, false);
        mdyn2.motor_setpower(1, speedDino + Lkf, true);
      }
    } else if (digitSensor() == "RIGHT") {  // если видим поворот направо, то сигнализируем
      mdyn2.rgb_set(1, 255, 0, 0);
      mdyn2.rgb_set(4, 255, 0, 0);
    } else if (digitSensor() == "LEFT") {  // если видим поворот налево, то сигнализируем
      mdyn2.rgb_set(2, 255, 0, 0);
      mdyn2.rgb_set(3, 255, 0, 0);
    } else if (digitSensor() == "STOP") {  // если видим стоп-линию, то останавливаемся
      for (int i = 1; i < 5; i++) {
        mdyn2.rgb_set(i, 255, 0, 0);
      }
      mdyn2.motor_setpower(2, -10, false);
      mdyn2.motor_setpower(1, -10, true);
      delay(50);
      mdyn2.motor_setpower(2, 0, false);
      mdyn2.motor_setpower(1, 0, true);
      angle(true, 180);  // повернуть против часовой на 180
      while (true) {};   // стоим
    }
  }
}


/// функция "цифровизации" датчика линии
String digitSensor() {
  getLines();
  int count = 0;
  for (int i = 1; i < 9; i++) {
    if (data[i - 1] > l) {
      digitData[i - 1] = 1;
      count++;
    } else {
      digitData[i - 1] = 0;
    }
    //  Serial.print(digitData[i - 1]);
  }
  //Serial.println("");

  if (count > 6) {  // если видим черный цвет на более чем 6-ти сенсорах
    return "STOP";
  } else if (count > 2 and count < 6 and digitData[7] == 1) {  // если больше 2 сенсоров видит линию и один из них это правый сенсор
    return "RIGHT";
  } else if (count > 2 and count < 6 and digitData[0] == 1) {  // если больше 2 сенсоров видит линию и один из них это левый сенсор
    return "LEFT";
  } else {
    return "GO";  // в остальных случаях едем по линии
  }
}

// получить аналоговые значения с датчика линии
void getLines() {
  for (int i = 0; i < 8; i++) {
    ResetADC();
    ReadRegister(MANUAL_CH_SEL_ADDRESS);
    readAnalogIn(sensors[i]);
    data[i] = returnedValue;
    /*    Serial.print(" ");
    Serial.print(data[i]);*/
  }
  // Serial.println(" ");
}

// найти минимальный и максимальный кэф для скорости
void addKf() {
  maxKf = 0;
  minKf = 0;
  for (int i = 0; i < 8; i++) {
    findKf[i] = digitData[i] * Kf[i];
    maxKf = max(maxKf, findKf[i]);
    minKf = min(minKf, findKf[i]);
  }
}

String getColor() {
  int red, green, blue;
  while (!apds9960.colorDataReady()) {
    delay(5);
  }
  uint16_t r = 0;
  uint16_t g = 0;
  uint16_t b = 0;
  uint16_t c = 0;
  apds9960.getColorData(&r, &g, &b, &c);
  /* Serial.println("RED   = " + String(r));
  Serial.println("GREEN = " + String(g));
  Serial.println("BLUE  = " + String(b));
  Serial.println("CLEAR  = " + String(c));*/
  if (c > 100) {
    for (int i = 1; i < 5; i++) {
      mdyn2.rgb_set(i, 0, 0, 0);
    }
    return "WHITE";
  } else if (c < 20) {
    return "BLACK";
  } else if (r > g) {
    for (int i = 1; i < 5; i++) {
      mdyn2.rgb_set(i, 255, 0, 0);
    }
    return "RED";
  } else if (g > r and g > b) {
    for (int i = 1; i < 5; i++) {
      mdyn2.rgb_set(i, 0, 255, 0);
    }
    return "GREEN";
  } else if (b > 3 * r) {
    for (int i = 1; i < 5; i++) {
      mdyn2.rgb_set(i, 0, 0, 255);
    }
    return "BLUE";
  }
}

void angle(bool side, int angle) {
  // измеряем текущий угол
  int currentAngle = angleMeasure();
  // если нужно повернуть по часовой (направо)
  if (side == false) {
    // берем остаток от круга и поворачиваем, пока разница не станет меньше 10, но больше 0
    while (((currentAngle + angle) % 360) - angleMeasure() > compensation or ((currentAngle + angle) % 360) - angleMeasure() < 0) {
      // поворот на одном колесе
      mdyn2.motor_setpower(1, turnSpeed, side);  // расскомментировать чтоб поворачивать вокруг оси
      mdyn2.motor_setpower(2, turnSpeed, side);
    }
    // другой алгоритм поворота по часовой
    /*if (currentAngle + angle < 360) {
      while (currentAngle + angle - angleMeasure() > 10) {
        motorA_setpower(5, side);
        motorB_setpower(5, side);
      }
    } else {
      while (angle - (360 - currentAngle) - angleMeasure() > 10 or angle - (360 - currentAngle) - angleMeasure() < 0) {
        motorA_setpower(5, side);
        motorB_setpower(5, side);
      }
    }*/
  } else {
    // если поворачиваем налево (против часовой)
    // в простой ситуации, когда нужно повернуться на угол меньший, чем текущий
    if (currentAngle > angle) {
      // поворачиваем пока разница между углами не станет меньше -10
      while (currentAngle - angle - angleMeasure() < -(compensation)) {
        // поворот на одном колесе
        mdyn2.motor_setpower(1, turnSpeed, side);  // расскомментировать чтоб поворачивать вокруг оси
        mdyn2.motor_setpower(2, turnSpeed, side);
      }
      // в сложной ситуации, когда нужно осилить переход через 0
    } else {
      while (360 - (angle - currentAngle) - angleMeasure() > 0 or 360 - (angle - currentAngle) - angleMeasure() < -(compensation)) {
        // поворот на одном колесе
        mdyn2.motor_setpower(1, turnSpeed, side);  // расскомментировать чтоб поворачивать вокруг оси
        mdyn2.motor_setpower(2, turnSpeed, side);
      }
    }
  }
  // резко тормозим с включением на мгновение обратных моторов
  mdyn2.motor_setpower(1, turnSpeed, !side);
  mdyn2.motor_setpower(2, turnSpeed, !side);
  delay(50);
  mdyn2.motor_setpower(1, 0, side);
  mdyn2.motor_setpower(2, 0, side);
}

// измерение угла в градусах от 0 до 360
int angleMeasure(void) {
  mgb_i2c63.setBusChannel(GYRO);
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    // переменные для расчёта (ypr можно вынести в глобальеы переменные)
    Quaternion q;
    VectorFloat gravity;
    float ypr[3];
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    int angle = map(degrees(ypr[0]), -180, 180, 0, 360);
    Serial.println(angle);
    return angle;
  }
}
