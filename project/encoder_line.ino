#define ENC_IN_RIGHT_2 35 // пины энкодеров
#define ENC_IN_RIGHT_1 33
#include <SimpleTimer.h> // таймер
// переменные для подсчета импульсов энкодеров
volatile long right_wheel_pulse_count = 0;
volatile long left_wheel_pulse_count = 0;
#define ENC_UPDATE_TIME 70  // период обновления
SimpleTimer timer_winddir;
int circle = 360;  // количество импульсов на оборот

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
  // пины энкодеров на вход
  pinMode(ENC_IN_RIGHT_1, INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_2, INPUT_PULLUP);
  timer_winddir.setInterval(ENC_UPDATE_TIME, readEnc);  // настройки таймера для обновления

  // прерывания для энкодеров
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_1), right_wheel_pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_2), left_wheel_pulse, RISING);

  // запуск моторов
  mdyn2.motor_setpower(1, speed, false);
  mdyn2.motor_setpower(2, speed, false);
}

void loop() {
  timer_winddir.run();  // отслеживание таймера
}

// функции обработки прерываний 
void right_wheel_pulse() {
  right_wheel_pulse_count++;
}
void left_wheel_pulse() {
  left_wheel_pulse_count++;
}

void readEnc() {
  // подсчет оборотов в секунду
  Serial.print("мотор1 - об/сек: ");
  float rev1 = 1000 * right_wheel_pulse_count / (circle * ENC_UPDATE_TIME / 1000);
  Serial.print(rev1 / 1000);
  Serial.print(" /// мотор2 - об/сек: ");
  float rev2 = 1000 * left_wheel_pulse_count / (circle * ENC_UPDATE_TIME / 1000);
  Serial.println(rev2 / 1000);
  // исползование ПИД-регулятора для выравнивания оборотов
  int kf = computePID((rev1 - rev2) / 1000, 0, 5, 20, 0.002, 0.07, -70, 70);
  // обнуление счетчиков
  right_wheel_pulse_count = 0;
  left_wheel_pulse_count = 0;
  // изменение скорости моторов с учетом полученных значений с ПИД-регулятора
  mdyn2.motor_setpower(1, speed - kf, false);
  mdyn2.motor_setpower(2, speed + kf, true);
  Serial.print(speed + kf);
  Serial.print("");
  Serial.println(speed - kf);
}

// (вход, установка, п, и, д, период в секундах, мин.выход, макс. выход)
int computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {
  float err = setpoint - input; // ошибка = требуемое - текущее
  Serial.println("ERROR " + String(err));
  static float integral = 0, prevErr = 0; // static для хранения переменных на следующую итерацию
  integral = constrain(integral + (float)err * dt * ki, minOut, maxOut); // интегральная составляющая от ошибки
  float D = (err - prevErr) / dt; // скорость изменения ошибки, дифференцальная составляющая
  prevErr = err; // сохраняем ошибку для следующей итерации
  return constrain(err * kp + integral + D * kd, minOut, maxOut); // подсчет ПИД
}
