#define ENC_IN_RIGHT_2 35 // пины энкодеров колеса 1 и 2
#define ENC_IN_RIGHT_1 33
#include <SimpleTimer.h> // таймер
volatile long right_wheel_pulse_count = 0; // переменные для подсчета импульсов энкодеров
volatile long left_wheel_pulse_count = 0;
#define ENC_UPDATE_TIME 70  // период обновления
SimpleTimer timer_winddir;

// для понимания этого числа используйте скетч
// https://github.com/MAKblC/Dynamica-M2/blob/main/project/encoder.ino  
int circle = 690;  // количество импульсов на оборот

#include <Wire.h>                                               //библиотека для I2C интерфейса
#include <MGB_MDYN2.h>                                          // библиотека для моторной платы
Adafruit_PWMServoDriver mdyn2 = Adafruit_PWMServoDriver(0x79);  // адрес платы
int speed = 30;
int wheel_diameter = 43; // диаметр колеса в мм

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mdyn2.begin();
  mdyn2.setPWMFreq(100);
  for (int channel = 0; channel < 16; channel++) {
    mdyn2.setPWM(channel, 0, 0);
  }
 // пины энкодеров на вход
  pinMode(ENC_IN_RIGHT_1, INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_2, INPUT_PULLUP);
  timer_winddir.setInterval(ENC_UPDATE_TIME, readEnc);  // настройки таймера для обновления

  // включение прерываний для энкодеров
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_1), right_wheel_pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_2), left_wheel_pulse, RISING);

  // запуск моторов
  mdyn2.motor_setpower(1, speed, true);
  mdyn2.motor_setpower(2, speed, false);
}

void loop() {
  timer_winddir.run();  // отслеживание таймера
}

// функция обработки прерывания правого колеса
void right_wheel_pulse() {
  right_wheel_pulse_count++;
}

// функция обработки прерывания левого колеса
void left_wheel_pulse() {
  left_wheel_pulse_count++;
}

void readEnc() {
  Serial.print("мотор1 - об/сек: ");
  float rev1 = 1000 * right_wheel_pulse_count / (circle * ENC_UPDATE_TIME / 1000);
  Serial.print(rev1 / 1000); // обороты в секунду
  Serial.print(" мм/сек: ");
  Serial.print(wheel_diameter * PI * rev1 / 1000); // перевод оборотов в скорость
  Serial.print(" /// мотор2 - об/сек: ");
  float rev2 = 1000 * left_wheel_pulse_count / (circle * ENC_UPDATE_TIME / 1000);
  Serial.print(rev2 / 1000);
  Serial.print(" мм/сек: ");
  Serial.println(wheel_diameter * PI * rev2 / 1000);
  right_wheel_pulse_count = 0; // обнуление счетчика
  left_wheel_pulse_count = 0;
}
