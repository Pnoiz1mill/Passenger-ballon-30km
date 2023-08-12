// ต่อ pin 9 !! 
const int pumpPin = 9;

void setup() {
  pinMode(pumpPin, OUTPUT);
}

void loop() {
  turnOnPump();
  delay(5000); 

  turnOffPump();
  delay(5000); 
}

void turnOnPump() {
  digitalWrite(pumpPin, HIGH);  // เปิดปั้ม
}

void turnOffPump() {
  digitalWrite(pumpPin, LOW);   // ปิดปั้ม
}
