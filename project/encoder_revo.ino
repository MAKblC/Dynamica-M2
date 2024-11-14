#define ENC_IN_RIGHT_A 35
volatile long right_wheel_pulse_count = 0;

int circle = 360;                 // количество импульсов на оборот

#include <Wire.h>
#include <MGB_MDYN2.h>                                          // библиотека для моторной платы
Adafruit_PWMServoDriver mdyn2 = Adafruit_PWMServoDriver(0x79);  // адрес платы

void setup() {
  Serial.begin(115200);

  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);

  Wire.begin();
  mdyn2.begin();
  mdyn2.setPWMFreq(100);
  for (int channel = 0; channel < 16; channel++) {
    mdyn2.setPWM(channel, 0, 0);
  }
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);
}

void loop() {
  revo(2); 
  revo(1.5);
  revo(2.5);
}

void right_wheel_pulse() {
  right_wheel_pulse_count++;
}

void revo(float number) { // функция выполнится после number оборотов
  while (right_wheel_pulse_count < circle * number) {
  }
  for (int i = 1; i < 5; i++) {  // выключить красный
    mdyn2.rgb_set(i, random(0, 255), random(0, 255), random(0, 255));
  }
  right_wheel_pulse_count = 0;
}
