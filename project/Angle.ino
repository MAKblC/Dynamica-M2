#include <Wire.h>
#include <MGB_MDYN2.h>                                          // библиотека для моторной платы
Adafruit_PWMServoDriver mdyn2 = Adafruit_PWMServoDriver(0x79);  // адрес платы

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;

uint8_t fifoBuffer[45];  // буфер
int turnSpeed = 15;      // скорость поворота
int compensation = 2;    // компенсация поворота (зависит от скорости поворота)

#include <MGB_I2C63.h>
// false - для PW548A, true - для PCA9547
MGB_I2C63 mgb_i2c63 = MGB_I2C63(false);
#define GYRO 0x07

unsigned long lasttime;

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

  // инициализация DMP
  mgb_i2c63.setBusChannel(GYRO);
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  // калибровка 15 сек
  while (millis() - lasttime < 15000) {
    angleMeasure();
  }
  Serial.println("калибровка завершена");
  delay(1000);
}

void loop() {
  angle(true, 90);  // повернуть по часовой на 90
  angle(false, 180);  // повернуть против часовой на 180
  angle(true, 90);  // повернуть по часовой на 90
}

// повернуть(сторона (направо 1/налево 0), угол)
void angle(bool side, int angle) {
  // измеряем текущий угол
  int currentAngle = angleMeasure();
  // если нужно повернуть по часовой (направо)
  if (side == true) {
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
