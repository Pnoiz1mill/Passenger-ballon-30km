#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <SD.h> 
#include <SPI.h>

Adafruit_BME280 bme1;
Adafruit_BME280 bme2;
File myFile;
HardwareSerial &ss = Serial1; // RX,TX PORT

// String stateStr[3] = {"PRELAUNCH", "LAUNCH", "LAND"};


SFE_UBLOX_GNSS myGNSS; // Move the declaration of myGNSS to the global scope

#define BME280_I2C_ADDR1 0x76
#define BME280_I2C_ADDR2 0x77
#define SEALEVELPRESSURE_HPA (1013.25)



float temp1;
float temp2;
float press1;
float press2;
// const int pumpPin = 0;
// unsigned long startTimePump = 0;
// bool pumpRunning = false;
const int fanPin = 0; // Changed fanPin to 7 to avoid conflicts
unsigned long startTimeFan = 0;
bool fanRunning = false;
int lastDoStateLogic = 0;
int lastTransmit = 0;
float lat = 0;
float lng = 0;
float galt = 0;
float altitude = 0;
float sensorAlt = 0;
float groundAlt= 0;
int counter =1;
int lastcount=0;
String p_com = "";
uint32_t t;

const int chipSelect = 10; 

void turnOnFan() {
  digitalWrite(fanPin, HIGH);
}

void turnOffFan() {
  digitalWrite(fanPin, LOW);
}

void gpsData(){

   lat = myGNSS.getLatitude();
   lng = myGNSS.getLongitude();
   galt = myGNSS.getAltitudeMSL()/1000; // Altitude above Mean Sea Level
   t = myGNSS.getUnixEpoch()/1000;  // Time above Mean Sea Level
}    
   
void getBMEData() {   
    if (groundAlt == 0){
       groundAlt = bme1.readAltitude(SEALEVELPRESSURE_HPA);
    }

    temp1 = bme1.readTemperature();
    temp2 = bme2.readTemperature();
    press1 = bme1.readPressure() / 100.0;
    press2 = bme2.readPressure() / 100.0;
    float sensorAlt = bme1.readAltitude(SEALEVELPRESSURE_HPA);
    altitude = sensorAlt - groundAlt;
}

void setup() {
  // pinMode(pumpPin, OUTPUT);
  pinMode(fanPin, OUTPUT);
  Wire.begin();
  ss.begin(9600);
  Serial.begin(9600);

  while (myGNSS.begin(0x42) == false) {  // Connect to the u-blox module using Wire port
    Serial.println(F("u-blox GNSS not detected at default I2C address. Retrying..."));

  }
  myGNSS.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR);
  myGNSS.setNavigationFrequency(5, VAL_LAYER_RAM_BBR);
  myGNSS.setAutoPVT(true, VAL_LAYER_RAM_BBR);
  myGNSS.setDynamicModel(DYN_MODEL_AIRBORNE2g, VAL_LAYER_RAM_BBR);

  if (!bme1.begin(BME280_I2C_ADDR1)) {
    Serial.println("Could not find a valid BME280 sensor at address 0x76");
    while (1);
  }

  if (!bme2.begin(BME280_I2C_ADDR2)) {
    Serial.println("Could not find a valid BME280 sensor at address 0x77");
    while (1);
  }

  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    return;
  }

}

void getData(){                            
  getBMEData();
  gpsData();

}

void loop() {
    if (millis() - lastDoStateLogic >= 1000)
    {
        lastDoStateLogic = millis();
       getData();
    }
    if (millis()-lastTransmit >= 3000){
      lastTransmit=millis();
      counter++;
      p_com += String(counter) + ',';
      p_com = String(temp1,2) + ',';
      p_com += String(temp2,2) + ',';
      p_com += String(press1,2) + ',';
      p_com += String(press2,2) + ',';
      p_com += String(lat,6) + ',';
      p_com += String(lng,6) + ',';
      p_com += String(galt,2) + ',';
      p_com += String(t) ;
      myFile = SD.open("test.txt", FILE_WRITE);
      myFile.println(p_com); // สั่งให้เขียนข้อมูล
      myFile.flush(); // ปิดไฟล์
      Serial.println(p_com);
    
      ss.println(p_com);

      }
    
    //fan
    if (temp1 >= 60) {
    turnOnFan();
    fanRunning = true;
  } else {
    unsigned long currentTime = millis();
    if (currentTime - startTimeFan >= 5000) {
      startTimeFan = millis();
      turnOffFan();
      fanRunning = false;
      delay(5000);
    }
  }
  delay(1000);

}
