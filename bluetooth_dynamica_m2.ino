#define REMOTEXY_MODE__ESP32CORE_BLE
#include <BLEDevice.h>
#define REMOTEXY_BLUETOOTH_NAME "Dynamica_M2"
#include <RemoteXY.h>
// конфигурация интерфейса RemoteXY
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] = { 255, 6, 0, 8, 0, 82, 0, 19, 0, 0, 0, 0, 31, 1, 106, 200, 1, 1, 5, 0,
                            5, 23, 52, 60, 60, 32, 2, 26, 31, 6, 24, 131, 58, 58, 2, 26, 73, 7, 12, 14,
                            32, 4, 128, 0, 2, 26, 0, 0, 0, 0, 0, 0, 200, 66, 0, 0, 0, 0, 73, 86,
                            13, 14, 32, 4, 128, 0, 2, 26, 0, 0, 0, 0, 0, 0, 200, 66, 0, 0, 0, 0,
                            1, 42, 15, 24, 24, 1, 9, 31, 0 };
struct {
  int8_t joystick_01_x;  // oт -100 до 100
  int8_t joystick_01_y;  // oт -100 до 100
  uint8_t rgb_01_r;      // =0..255 значение Красного цвета
  uint8_t rgb_01_g;      // =0..255 значение Зеленого цвета
  uint8_t rgb_01_b;      // =0..255 значение Синего цвета
  uint8_t button_01;     // =1 если кнопка нажата, иначе =0

  // output variables
  float linearbar_01;  // oт 0 до 100
  float linearbar_02;  // oт 0 до 100

  // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0
} RemoteXY;
#pragma pack(pop)

#include <Wire.h>  //библиотека для I2C интерфейса

#include <MGB_MDYN2.h>                                          // библиотека для моторной платы
Adafruit_PWMServoDriver mdyn2 = Adafruit_PWMServoDriver(0x79);  // адрес платы
float power = 0.5; // мощность моторов 0-1

#include <VL53L0X.h>  // библиотека для датчика MGS-D20
VL53L0X lox1;
VL53L0X lox2;

/*
  I2C порт 0x07 - выводы GP16 (SDA), GP17 (SCL)
  I2C порт 0x06 - выводы GP4 (SDA), GP13 (SCL)
  I2C порт 0x05 - выводы GP14 (SDA), GP15 (SCL)
  I2C порт 0x04 - выводы GP5 (SDA), GP23 (SCL)
  I2C порт 0x03 - выводы GP18 (SDA), GP19 (SCL)
*/
#include <MGB_I2C63.h>
// false - для PW548A, true - для PCA9547
MGB_I2C63 mgb_i2c63 = MGB_I2C63(false);
#define DIST1 0x05
#define DIST2 0x03
#define BUZ 0x06

#include <MGB_BUZ1.h>  // библиотека для MGB-BUZ1
Adafruit_MCP4725 buzzer;

void setup() {
  Serial.begin(115200);
  RemoteXY_Init();
  // Инициализация драйвера
  Wire.begin();
  mdyn2.begin();
  // Частота (Гц)
  mdyn2.setPWMFreq(100);
  // Все порты моторной платы выключены
  for (int channel = 0; channel < 16; channel++) {
    mdyn2.setPWM(channel, 0, 0);
  }

  // запуск генератора звука
  mgb_i2c63.setBusChannel(BUZ);
  buzzer.begin(0x60);           // Без перемычки адрес будет 0x61
  buzzer.setVoltage(0, false);  // выключение звука
  buzzer.volume(800);           // громкость (1-999)

  // запуск датчиков расстояния
  mgb_i2c63.setBusChannel(DIST1);
  lox1.init();
  lox1.setTimeout(500);
  lox1.setMeasurementTimingBudget(20000);
  mgb_i2c63.setBusChannel(DIST2);
  lox2.init();
  lox2.setTimeout(500);
  lox2.setMeasurementTimingBudget(20000);
}

void loop() {
  RemoteXY_Handler();
  // управление моторами с джойстика
  // смещение бегунка по оси Y наращивает скорость моторов вперед или назад
  // смещение бегунка по оси X ослабляет скорость одного из колес, тем самым можно повернуть
  // используйте переменную power для регулировки мощности моторов
  if (RemoteXY.joystick_01_x < 0) {
    mdyn2.motor_setpower(1, power * RemoteXY.joystick_01_y, false);
    mdyn2.motor_setpower(2, power * RemoteXY.joystick_01_y * (1 - (abs(RemoteXY.joystick_01_x) / 100)), true);
  } else {
    mdyn2.motor_setpower(1, power * RemoteXY.joystick_01_y * (1 - (RemoteXY.joystick_01_x) / 100), false);
    mdyn2.motor_setpower(2, power * RemoteXY.joystick_01_y, true);
  }
  // включение светодиодов с помощью RGB-круга
  for (int i = 1; i < 5; i++) {
    mdyn2.rgb_set(i, RemoteXY.rgb_01_r, RemoteXY.rgb_01_g, RemoteXY.rgb_01_b);
  }
  // включение звука по кнопке
  if (RemoteXY.button_01) {
    // кнопка нажата
    mgb_i2c63.setBusChannel(BUZ);
    buzzer.note(3, 450);
  }
  // вывод расстояния до препятствий в индикаторы
  mgb_i2c63.setBusChannel(DIST1);
  float dist1 = lox1.readRangeSingleMillimeters();
  mgb_i2c63.setBusChannel(DIST2);
  float dist2 = lox2.readRangeSingleMillimeters();
  if (0 < dist1 < 200) {
    RemoteXY.linearbar_01 = map(dist1, 0, 200, 100, 0);
  }
  if (0 < dist2 < 200) {
    RemoteXY.linearbar_02 = map(dist2, 0, 200, 100, 0);
  }
}
