#include <Wire.h>
#include <MGB_MDYN2.h>                                          // библиотека для моторной платы
Adafruit_PWMServoDriver mdyn2 = Adafruit_PWMServoDriver(0x79);  // адрес платы

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;

uint8_t fifoBuffer[45];  // буфер
int error;               // ошибка для пропорционального регулятора

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
}

void loop() {
  orient(180, 3);
  orient(90, 3);
}

// измерение угла в градусах от 0 до 360
int angleMeasure(void) {
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

// ориентируемся угла angle в течение sec секунд
void orient(int angle, int sec) {
  while (millis() - lasttime < sec * 1000) {
    int currentAngle = angleMeasure();
    // ошибка = разница заданного курса и текущего
    error = currentAngle - angle;
    // разница добавляется и отнимается в скорость моторов
    // данный простой П-регулятор можно регулировать с коэффициентами либо добавлять -И и -Д составляющие
    mdyn2.motor_setpower(1, 15 + error, true);
    mdyn2.motor_setpower(2, 15 - error, false);
  }
  lasttime = millis();
}

/*// ориентируемся угла angle (для другого триггера окончания)
void orient2(int angle) {
    int currentAngle = angleMeasure();
    // ошибка = разница заданного курса и текущего
    error = currentAngle - angle;
    // разница добавляется и отнимается в скорость моторов
    // данный простой П-регулятор можно регулировать с коэффициентами либо добавлять -И и -Д составляющие
    mdyn2.motor_setpower(1, 15 + error, true);
    mdyn2.motor_setpower(2, 15 - error, false);
}
*/
