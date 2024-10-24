#include <Wire.h>
#include <MGB_MDYN2.h>                                          // библиотека для моторной платы
Adafruit_PWMServoDriver mdyn2 = Adafruit_PWMServoDriver(0x79);  // адрес платы

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
  /*mdyn2.rgb_set(1, 0, 255, 0);    // зеленый
  mdyn2.rgb_set(2, 255, 255, 0);  // желтый
  mdyn2.rgb_set(3, 0, 0, 255);    // синий
  mdyn2.rgb_set(4, 255, 0, 0);    // красный
  delay(3000);
  // выключить светодиоды
  mdyn2.rgb_set(1, 0, 0, 0);
  mdyn2.rgb_set(2, 0, 0, 0);
  mdyn2.rgb_set(3, 0, 0, 0);
  mdyn2.rgb_set(4, 0, 0, 0);
  delay(1000);*/
  for (int k = 0; k < 256; k++) { // от 0 до 255
    for (int i = 1; i < 5; i++) { // включить красный
      mdyn2.rgb_set(i, k, 0, 0);
    }
    delay(2);
  }
  for (int k = 255; k > 0; k--) { // от 255 до 0
    for (int i = 1; i < 5; i++) { // выключить красный
      mdyn2.rgb_set(i, k, 0, 0);
    }
    delay(2);
  }
  // зеленый
  for (int k = 0; k < 256; k++) {
    for (int i = 1; i < 5; i++) {
      mdyn2.rgb_set(i, 0, k, 0);
    }
    delay(2);
  }
  for (int k = 255; k > 0; k--) {
    for (int i = 1; i < 5; i++) {
      mdyn2.rgb_set(i, 0, k, 0);
    }
    delay(2);
  }
  // синий
  for (int k = 0; k < 256; k++) {
    for (int i = 1; i < 5; i++) {
      mdyn2.rgb_set(i, 0, 0, k);
    }
    delay(2);
  }
  for (int k = 255; k > 0; k--) {
    for (int i = 1; i < 5; i++) {
      mdyn2.rgb_set(i, 0, 0, k);
    }
    delay(2);
  }
}
