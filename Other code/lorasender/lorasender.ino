
HardwareSerial &ss = Serial1;


void setup() {
  Serial.begin(115200);
  ss.begin(9600);
  // Wire.begin();
}

void loop() {
  ss.println("kfski");
  Serial.println("hello");
  
  delay(200);
}
  