#include <Wire.h>
#include "MGS_CLMLN8.h"

#include <MGB_MDYN2.h>                                          // библиотека для моторной платы
Adafruit_PWMServoDriver mdyn2 = Adafruit_PWMServoDriver(0x79);  // адрес платы

#include <Adafruit_APDS9960.h>
Adafruit_APDS9960 apds9960;

float speedDino = 20;  // скорость Робота
int data[8];
int massSens[8];
byte digitData[8];

// коэффициенты ускорения/замедления при смещении линии
int Kf[] = { 120, 90, 50, 15, -15, -50, -90, -120 };
int findKf[8];
int maxKf = 0, minKf = 0;
float Lkf, Rkf;
bool flagmove = true;
uint32_t time = 0;

void setup() {
  // Инициализация последовательного порта
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  delay(100);
  mdyn2.begin();
  // Частота (Гц)
  mdyn2.setPWMFreq(100);
  // Все порты моторной платы выключены
  for (int channel = 0; channel < 16; channel++) {
    mdyn2.setPWM(channel, 0, 0);
  }

  if (!apds9960.begin()) {
    Serial.println("Failed to initialize device!");
  }
  // Инициализация режимов работы датчика
  apds9960.enableColor(true);
  delay(1000);
}


void loop() {
  // Измерение
  digitSensor();
  addKf();
  // если путаница с линией, то едем прямо
  if (-minKf == maxKf) {
    mdyn2.motor_setpower(1, speedDino, false);
    mdyn2.motor_setpower(2, speedDino, true);
  } else {
    maxKf = maxKf < 0 ? 0 : maxKf;
    minKf = minKf > 0 ? 0 : minKf;
    // расчет коэффицента ускорения/ослабления скорости мотора
    Lkf = speedDino / 100 * (float)minKf;
    Rkf = speedDino / 100 * (float)maxKf;
    //Serial.println(String(speedDino - Rkf) + " / " + String(speedDino + Lkf));
    mdyn2.motor_setpower(2, speedDino - Rkf, true);
    mdyn2.motor_setpower(1, speedDino + Lkf, false);
  }
  Serial.println(getColor()); // вывод цвета
}

/// функция "цифровизации" датчика линии
void digitSensor() {
  getLines();
  int l = Targetline();
  digitData[0] = data[0] > l ? 1 : 0;
  //Serial.print(digitData[0]);
  digitData[1] = data[1] > l ? 1 : 0;
  //Serial.print(digitData[1]);
  digitData[2] = data[2] > l ? 1 : 0;
  //Serial.print(digitData[2]);
  digitData[3] = data[3] > l ? 1 : 0;
  //Serial.print(digitData[3]);
  digitData[4] = data[4] > l ? 1 : 0;
  //Serial.print(digitData[4]);
  digitData[5] = data[5] > l ? 1 : 0;
  //Serial.print(digitData[5]);
  digitData[6] = data[6] > l ? 1 : 0;
  //Serial.print(digitData[6]);
  digitData[7] = data[7] > l ? 1 : 0;
  //Serial.print(digitData[7]);
}

// получить аналоговые значения с датчика линии
void getLines() {
  for (int i = 0; i < 8; i++) {
    ResetADC();
    ReadRegister(MANUAL_CH_SEL_ADDRESS);
    readAnalogIn(sensors[i]);
    data[i] = returnedValue;
    /* Serial.print(" ");
    Serial.print(data[i]);*/
  }
  /*Serial.println(" ");*/
}

// найти минимальный и максимальный кэф для скорости
void addKf() {
  maxKf = 0;
  minKf = 0;
  for (int i = 0; i < 8; i++) {
    findKf[i] = digitData[i] * Kf[i];
    maxKf = max(maxKf, findKf[i]);
    minKf = min(minKf, findKf[i]);
  }
}

// сравнение линии и дороги
int Targetline() {
  int l = 0;
  for (int i = 0; i < 8; i++) {
    l = max(l, data[i]);
  }
  if (l == 0) l = 3600;
  l -= 20;
  return l;
}


// получить цвет
String getColor() {
  int red, green, blue;
  while (!apds9960.colorDataReady()) {
    delay(5);
  }
  uint16_t r = 0;
  uint16_t g = 0;
  uint16_t b = 0;
  uint16_t c = 0;
  apds9960.getColorData(&r, &g, &b, &c);
  /* Serial.println("RED   = " + String(r));
  Serial.println("GREEN = " + String(g));
  Serial.println("BLUE  = " + String(b));
  Serial.println("CLEAR  = " + String(c));*/
  if (c > 100) {
    for (int i = 1; i < 5; i++) {
      mdyn2.rgb_set(i, 0, 0, 0);
    }
    return "WHITE";
  } else if (c < 20) {
    return "BLACK";
  } else if (r > g) {
    for (int i = 1; i < 5; i++) {
      mdyn2.rgb_set(i, 255, 0, 0);
    }
    return "RED";
  } else if (g > r and g > b) {
    for (int i = 1; i < 5; i++) {
      mdyn2.rgb_set(i, 0, 255, 0);
    }
    return "GREEN";
  } else if (b > 3 * r) {
    for (int i = 1; i < 5; i++) {
      mdyn2.rgb_set(i, 0, 0, 255);
    }
    return "BLUE";
  }
}
