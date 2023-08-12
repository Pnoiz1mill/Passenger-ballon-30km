#include <Wire.h>
#define ledpin 10
#define ledpin2 11

#define lora Serial2
#define lora2 Serial3
String msg1 = "";
String msg2 = "";

void setup() {
  Serial.begin(9600);
  lora.begin(9600);
  lora2.begin(9600);
  Wire.begin();
  // pinMode(ledpin,OUTPUT);
  // pinMode(ledpin2,OUTPUT);
  if (!lora) {
    Serial.println("Can't find Lora1");
    // digitalWrite(ledpin,HIGH);
    // delay(1000);
    // digitalWrite(ledpin,LOW);
    // delay(1000);
  } else {
    Serial.println("Lora1");
  }

  if (!lora2) {
    Serial.println("Can't find Lora2");
    // digitalWrite(ledpin2,HIGH);
    // delay(1000);
    // digitalWrite(ledpin2,LOW);
    // delay(1000);
  } else {
    Serial.println("Lora2");
  }
  Serial.println("Set Up");
}

void loop() {
  

  while (lora.available()) {
    char charac = lora.read();
    if (charac == '\n') {
      Serial.println(msg1);
      msg1 = "";

    } else {
      msg1 += charac;
    }
  }

  while (lora2.available()) {
    char charac = lora2.read();
    if (charac == '\n') {
      Serial.println(msg2);
      msg2 = "";

    } else {
      msg2 += charac;
    }
  }
}