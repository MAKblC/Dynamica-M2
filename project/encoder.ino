// покрутите колесо на 1 оборот, чтобы узнать, сколько импульсов считывается
#define ENC_IN_RIGHT_A 35
volatile long right_wheel_pulse_count = 0;

void setup() {
  Serial.begin(115200);
  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);
}

void loop() {
  Serial.println(right_wheel_pulse_count);
}

void right_wheel_pulse() {
  right_wheel_pulse_count++;
}
