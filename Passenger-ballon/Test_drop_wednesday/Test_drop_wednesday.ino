#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <SD.h> 

#define BME280_I2C_ADDR1 0x76
Adafruit_BME280 bme1;
const int lm35Pin = A0;
HardwareSerial &ss = Serial2;

String p_com = "";
SFE_UBLOX_GNSS myGNSS;

const int pumpPin = 9;
unsigned long startTimePump = 0;
bool pumpRunning = false;
const int fanPin = 7;
unsigned long startTimeFan = 0;
bool fanRunning = false;
unsigned long starttime = 0;

const int chipSelect = 10; 
File dataFile; 

float getLM35Temperature() {
  float voltage = analogRead(lm35Pin) * 5.0 / 1023.0;
  float tempCelsius = (voltage - 0.5) * 100.0;
  return tempCelsius;
}

void turnOnPump() {
  digitalWrite(pumpPin, HIGH);
}

void turnOffPump() {
  digitalWrite(pumpPin, LOW);
}

void turnOnFan() {
  digitalWrite(fanPin, HIGH);
}

void turnOffFan() {
  digitalWrite(fanPin, LOW);
}

void setup() {
  pinMode(pumpPin, OUTPUT);
  pinMode(fanPin, OUTPUT);
  // pinMode(50,OUTPUT);
  // pinMode(51, OUTPUT);
  // digitalWrite(50, LOW);
  // digitalWrite(51, LOW);
  Wire.begin();
  Serial.begin(115200);

  ss.begin(9600);
  turnOnPump();
  if (!bme1.begin(BME280_I2C_ADDR1)) {
    Serial.println("Could not find a valid BME280 sensor. Check wiring!");
    while (1);
  }
}

void loop() {
  float temp1 = bme1.readTemperature();
  float temp2 = getLM35Temperature();
  float press1 = bme1.readPressure() / 100.0; // Convert pressure to Pa
  float hum1 = bme1.readHumidity();

  // Calculate altitude using the BME280 pressure reading
  const float standardPressure = 1013.25; // Standard atmospheric pressure at sea level in hPa
  float altitude = 44330.77 * (1 - pow(press1 / standardPressure, 0.1902632));
  if (millis() - starttime >= 1000){
  starttime = millis();
  p_com = String(temp1) + ',';
  p_com += String(temp2) + ',';
  p_com += String(press1) + ',';
  p_com += String(hum1) + ',';
  p_com += String(altitude); 
  
  Serial.println(p_com);
  ss.println(p_com);
  
  }
  


  // Check if altitude is greater than or equal to 100 meters to open the pump
  if (altitude >= 50) {
    if (!pumpRunning) {
      turnOnPump();
      startTimePump = millis();
      pumpRunning = true;
    } else if (millis() - startTimePump >= 5000) { // 5000 milliseconds = 5 seconds
      turnOffPump();
      pumpRunning = false;
    }  , turn it off
    if (pumpRunning) {
      turnOffPump();
      pumpRunning = false;
    }
  }
      
      
+

      if (altitude >= 50) {
    if (!fanRunning) {
      turnOnFan();
      startTimeFan = millis();
      fanRunning = true;
    } else if (millis() - startTimeFan >= 5000) { // 5000 milliseconds = 5 seconds
      turnOffFan();
      fanRunning = false;
    }
  } else {
    // If pump is already running and altitude drops below 100 meters, turn it off
    if (fanRunning) {
      turnOffFan();
      fanRunning = false;
    }
  }


  delay(1000);
}
