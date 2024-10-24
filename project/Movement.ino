#include <Wire.h>
#include <MGB_MDYN2.h>                                          // библиотека для моторной платы
Adafruit_PWMServoDriver mdyn2 = Adafruit_PWMServoDriver(0x79);  // адрес платы
float power = 0.3;

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
}

void loop() {
  // движение прямо
  mdyn2.motor_setpower(1, 15, true);
  mdyn2.motor_setpower(2, 15, false);
  delay(2000);
  // поворот по часовой стрелке
  mdyn2.motor_setpower(1, 15, true);
  mdyn2.motor_setpower(2, 15, true);
  delay(1500);
  // движение прямо
  mdyn2.motor_setpower(1, -15, false);
  mdyn2.motor_setpower(2, -15, true);
  delay(2000);
  // поворот по часовой стрелке
  mdyn2.motor_setpower(1, 15, true);
  mdyn2.motor_setpower(2, 15, true);
  delay(1500);
}
