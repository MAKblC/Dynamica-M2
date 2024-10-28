#include <Wire.h>
#include "MGS_CLMLN8.h"

#include <MGB_MDYN2.h>                                         
Adafruit_PWMServoDriver mdyn2 = Adafruit_PWMServoDriver(0x79); 

float speedDino = 20;  // скорость Робота
int data[8];
byte digitData[8];

int l = 1700;  // граница черного/белого

// коэффициенты ускорения/замедления при смещении линии
int Kf[] = { 120, 90, 50, 15, -15, -50, -90, -120 };
int findKf[8];
int maxKf = 0, minKf = 0;
float Lkf, Rkf;

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
  delay(1000);
}


void loop() {
  // Измерение
  // Serial.println(digitSensor());
  if (digitSensor() == "GO") { // если видим линию, то просто едем с выключенными светодиодами
    addKf();
    for (int i = 1; i < 5; i++) {
      mdyn2.rgb_set(i, 0, 0, 0);
    }
    // если путаница с линией, то едем прямо
    if (-minKf == maxKf) {
      mdyn2.motor_setpower(1, speedDino, true);
      mdyn2.motor_setpower(2, speedDino, false);
    } else {
      maxKf = maxKf < 0 ? 0 : maxKf;
      minKf = minKf > 0 ? 0 : minKf;
      // расчет коэффицента ускорения/ослабления скорости мотора
      Lkf = speedDino / 100 * (float)minKf;
      Rkf = speedDino / 100 * (float)maxKf;
      //Serial.println(String(speedDino - Rkf) + " / " + String(speedDino + Lkf));
      mdyn2.motor_setpower(2, speedDino - Rkf, false);
      mdyn2.motor_setpower(1, speedDino + Lkf, true);
    }
  } else if (digitSensor() == "RIGHT") { // если видим поворот направо, то сигнализируем
    mdyn2.rgb_set(1, 255, 0, 0);  
    mdyn2.rgb_set(4, 255, 0, 0);  
  } else if (digitSensor() == "LEFT") { // если видим поворот налево, то сигнализируем
    mdyn2.rgb_set(2, 255, 0, 0); 
    mdyn2.rgb_set(3, 255, 0, 0);  
  } else if (digitSensor() == "STOP") { // если видим стоп-линию, то останавливаемся
    for (int i = 1; i < 5; i++) {
      mdyn2.rgb_set(i, 255, 0, 0);
    }
    mdyn2.motor_setpower(2, -10, false);
    mdyn2.motor_setpower(1, -10, true);
    delay(50);
    mdyn2.motor_setpower(2, 0, false);
    mdyn2.motor_setpower(1, 0, true);
    while (true) {}; // стоим
  }
}

/// функция "цифровизации" датчика линии
String digitSensor() {
  getLines();
  int count = 0;
  for (int i = 1; i < 9; i++) {
    if (data[i - 1] > l) {
      digitData[i - 1] = 1;
      count++;
    } else {
      digitData[i - 1] = 0;
    }
    //  Serial.print(digitData[i - 1]);
  }
  //Serial.println("");
  
  if (count > 6) { // если видим черный цвет на более чем 6-ти сенсорах
    return "STOP";
  } else if (count > 2 and count < 6 and digitData[7] == 1) { // если больше 2 сенсоров видит линию и один из них это правый сенсор
    return "RIGHT";
  } else if (count > 2 and count < 6 and digitData[0] == 1) { // если больше 2 сенсоров видит линию и один из них это левый сенсор
    return "LEFT";
  } else {
    return "GO"; // в остальных случаях едем по линии
  }
}

// получить аналоговые значения с датчика линии
void getLines() {
  for (int i = 0; i < 8; i++) {
    ResetADC();
    ReadRegister(MANUAL_CH_SEL_ADDRESS);
    readAnalogIn(sensors[i]);
    data[i] = returnedValue;
    /*    Serial.print(" ");
    Serial.print(data[i]);*/
  }
  // Serial.println(" ");
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
