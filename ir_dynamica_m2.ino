#include <Wire.h>
#include <MGB_MDYN2.h>                                          // библиотека для моторной платы
Adafruit_PWMServoDriver mdyn2 = Adafruit_PWMServoDriver(0x79);  // адрес платы
float power = 0.3;

#include "IRremote.h"
IRrecv irrecv(27);
decode_results results;

#include <MGB_I2C63.h>
// false - для PW548A, true - для PCA9547
MGB_I2C63 mgb_i2c63 = MGB_I2C63(false);
#define BUZ 0x06

#include <MGB_BUZ1.h>  // библиотека для MGB-BUZ1
Adafruit_MCP4725 buzzer;

void setup() {
  irrecv.enableIRIn();
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

  mgb_i2c63.setBusChannel(BUZ);
  buzzer.begin(0x60);           // Без перемычки адрес будет 0x61
  buzzer.setVoltage(0, false);  // выключение звука
  buzzer.volume(800);           // громкость (1-999)
}

void loop() {
  if (irrecv.decode(&results)) {      // если данные пришли
    if (results.value == 0xFF18E7) {  // если это кнопка "2"
      mdyn2.motor_setpower(1, power * 50, false);
      mdyn2.motor_setpower(2, power * 50, true);
      delay(25);
    }
    if (results.value == 0xFF4AB5) {  // если это кнопка "8"
      mdyn2.motor_setpower(1, power * 50, true);
      mdyn2.motor_setpower(2, power * 50, false);
      delay(25);
    }
    if (results.value == 0xFF10EF) {  // если это кнопка "4"
      mdyn2.motor_setpower(1, power * 50, false);
      mdyn2.motor_setpower(2, power * 50, false);
      delay(25);
    }
    if (results.value == 0xFF5AA5) {  // если это кнопка "6"
      mdyn2.motor_setpower(1, power * 50, true);
      mdyn2.motor_setpower(2, power * 50, true);
      delay(25);
    }
    if (results.value == 0xFF38C7) {  // если это кнопка "5"
      mdyn2.motor_setpower(1, 0, false);
      mdyn2.motor_setpower(2, 0, false);
      delay(25);
    }
    if (results.value == 0xFFE21D) {  // если это кнопка "CH+"
      for (int i = 1; i < 5; i++) {
        mdyn2.rgb_set(i, 255, 0, 0);
      }
      delay(25);
    }
    if (results.value == 0xFFC23D) {  // если это кнопка "►┃┃"
      for (int i = 1; i < 5; i++) {
        mdyn2.rgb_set(i, 0, 255, 0);
      }
      delay(25);
    }
    if (results.value == 0xFF02FD) {  // если это кнопка "►►┃"
      for (int i = 1; i < 5; i++) {
        mdyn2.rgb_set(i, 0, 0, 255);
      }
      delay(25);
    }
    if (results.value == 0xFF906F) {  // если это кнопка "EQ"
      mgb_i2c63.setBusChannel(BUZ);
      buzzer.note(3, 450);
      buzzer.setVoltage(0, false);
      delay(25);
    }
    irrecv.resume();  // принимаем следующую команду
  }
}
