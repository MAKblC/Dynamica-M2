int encoderPin1 = 35;            
int encoderPin2 = 34;            
volatile int lastEncoded = 0;   
volatile long encoderValue = 0;  
int circle = 360;               

void setup() {
  Serial.begin(115200);

  pinMode(encoderPin1, INPUT_PULLUP);
  pinMode(encoderPin2, INPUT_PULLUP);
  digitalWrite(encoderPin1, HIGH);
  digitalWrite(encoderPin2, HIGH);


  attachInterrupt(encoderPin1, updateEncoder, CHANGE);
  attachInterrupt(encoderPin2, updateEncoder, CHANGE);
}

void loop() {
  float rev = 1000 * encoderValue / (circle * 4);
  Serial.println(rev / 1000);
}

void updateEncoder() {
  int MSB = digitalRead(encoderPin1);  
  int LSB = digitalRead(encoderPin2);  

  int encoded = (MSB << 1) | LSB;          
  int sum = (lastEncoded << 2) | encoded;  

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue++;

  lastEncoded = encoded;  
}
