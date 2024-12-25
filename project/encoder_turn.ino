#define ENC_IN_RIGHT_2 35 // пин энкодера

// переменная для подсчета импульсов
volatile long left_wheel_pulse_count = 0;

// для понимания этого числа используйте скетч
// https://github.com/MAKblC/Dynamica-M2/blob/main/project/encoder.ino  
int circle = 690;  // количество импульсов на оборот

int kf = 6; // количество оборотов колеса на полный оборот робота на одном колесе

#include <Wire.h>                                               //библиотека для I2C интерфейса
#include <MGB_MDYN2.h>                                          // библиотека для моторной платы
Adafruit_PWMServoDriver mdyn2 = Adafruit_PWMServoDriver(0x79);  // адрес платы
int speed = 30;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mdyn2.begin();
  mdyn2.setPWMFreq(100);
  for (int channel = 0; channel < 16; channel++) {
    mdyn2.setPWM(channel, 0, 0);
  }
  // определение пина энкодера на вход
  pinMode(ENC_IN_RIGHT_2, INPUT_PULLUP);
}

void loop() {
  turn(90);
  delay(500);
  turn(-180); // знак "-" для поворота в обратную сторону
  delay(500);
  turn(90);
  delay(500);
}

void left_wheel_pulse() {
  left_wheel_pulse_count++; // инкремент импульсов
}

void turn(int angle) { // функция поворота на угол angle
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_2), left_wheel_pulse, RISING); // включаем прерывание на пине энкодера
  if (angle > 0) { // если угол положительный
    while (left_wheel_pulse_count < kf*circle*angle/360) { // ждем пока количество импульсов не достигнет требуемого угла поворота
      mdyn2.motor_setpower(1, speed, true); // поворот
    }
    mdyn2.motor_setpower(1, 5, false); // "ручник"
  } else {
    while (left_wheel_pulse_count < abs(kf*circle*angle/360)) {
      mdyn2.motor_setpower(1, speed, false);
    }
    mdyn2.motor_setpower(1, 5, true); // "ручник"
  }
  mdyn2.motor_setpower(1, 0, true); // стоп
  detachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_2)); // отключаем прерывания
  left_wheel_pulse_count = 0; // обнуляем счетчик
}
