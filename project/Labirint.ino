// скетч для алгоритма прохождения прямоугольного лабиринта
#include <Wire.h>
#include <MGB_MDYN2.h>                                          // библиотека для моторной платы
Adafruit_PWMServoDriver mdyn2 = Adafruit_PWMServoDriver(0x79);  // адрес платы
float power = 0.3;

#include <VL53L0X.h>  // библиотека для датчика MGS-D20
VL53L0X loxL;
VL53L0X loxR;

#include <MGB_I2C63.h>
// false - для PW548A, true - для PCA9547
MGB_I2C63 mgb_i2c63 = MGB_I2C63(false);
#define DISTL 0x05
#define DISTR 0x03
#define GYRO 0x07

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;

uint8_t fifoBuffer[45];  // буфер
int turnSpeed = 18;      // скорость поворота
int compensation = 15;    // компенсация поворота (зависит от скорости поворота)
unsigned long lasttime;

int critical = 120; // расстояние до стены
int error;

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

  mgb_i2c63.setBusChannel(DISTL);
  loxL.init();
  loxL.setTimeout(500);
  loxL.setMeasurementTimingBudget(20000);
  mgb_i2c63.setBusChannel(DISTR);
  loxR.init();
  loxR.setTimeout(500);
  loxR.setMeasurementTimingBudget(20000);

  // инициализация DMP
  mgb_i2c63.setBusChannel(GYRO);
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  // калибровка 15 сек
  while (millis() - lasttime < 5000) {
    angleMeasure();
  }
  Serial.println("калибровка завершена");
  delay(1000);
}

void loop() {
  // едем прямо с указанием расстояния до стены впереди
  forward(critical);
  // смотрим правый датчик расстояния
  mgb_i2c63.setBusChannel(DISTR);
  float distR = loxR.readRangeSingleMillimeters();
  if (distR > critical) {
    // если там есть проезд
    Serial.println("поворачиваю направо");
    angle(false, 90);
  } else {
    // поворот в другую сторону
    Serial.println("поворачиваю налево");
    angle(true, 90);
  }
}

// функция езды вперед
void forward(int distance) {
  Serial.println("еду вперед");
  int angle_1 = angleMeasure();
  Serial.println("Угол  = " + String(angle_1));
  mgb_i2c63.setBusChannel(DISTL);
  float distL = loxL.readRangeSingleMillimeters();
  Serial.println("Distance  = " + String(distL, 0) + " mm  ");
  while (distL > distance) {
    mgb_i2c63.setBusChannel(DISTL);
    distL = loxL.readRangeSingleMillimeters();
    // едем с выравниванием траектории
    orient2(angle_1);
  }
  // остановка
  mdyn2.motor_setpower(1, -10, true);
  mdyn2.motor_setpower(2, -10, false);
  delay(50);
  mdyn2.motor_setpower(1, 0, true);
  mdyn2.motor_setpower(2, 0, false);
}

// поворот на угол
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

// функция выравнивания траектории
void orient2(int angle) {
  int currentAngle = angleMeasure();
  // ошибка = разница заданного курса и текущего
  error = currentAngle - angle;
  // разница добавляется и отнимается в скорость моторов
  // данный простой П-регулятор можно регулировать с коэффициентами либо добавлять -И и -Д составляющие
  mdyn2.motor_setpower(1, 22 + error, true);
  mdyn2.motor_setpower(2, 22 - error, false);
}
