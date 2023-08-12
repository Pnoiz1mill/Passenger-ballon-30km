#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <SD.h> 
#include <SPI.h>

Adafruit_BME280 bme1;
Adafruit_BME280 bme2;
File myFile;
HardwareSerial &ss = Serial1;

// String stateStr[3] = {"PRELAUNCH", "LAUNCH", "LAND"};

String p_com = "";
SFE_UBLOX_GNSS myGNSS; // Move the declaration of myGNSS to the global scope

#define BME280_I2C_ADDR1 0x76
#define BME280_I2C_ADDR2 0x77
#define SEALEVELPRESSURE_HPA (1013.25)

float temp1;
float temp2;
float press1;
float press2;
const int pumpPin = 29;
unsigned long startTimePump = 0;
bool pumpRunning = false;
const int fanPin = 22; // Changed fanPin to 7 to avoid conflicts
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
int counter =0;
int lastcount=0;

const int chipSelect = 53; 
File dataFile; 

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

void gpsData(){
   lat = myGNSS.getLatitude();
   lng = myGNSS.getLongitude();
   galt = myGNSS.getAltitudeMSL();


}
void getBMEData()
{   
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
  pinMode(pumpPin, OUTPUT);
  pinMode(fanPin, OUTPUT);
  Wire.begin();
  ss.begin(9600);
  Serial.begin(9600);
  

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
  Serial.println("SD card initialized successfully!");

  dataFile = SD.open("data.txt", FILE_WRITE);

  if (dataFile) {
    Serial.println("Data file opened successfully!");
    dataFile.println("Hello, this is a sample text."); 
    dataFile.close();
    Serial.println("Text data written to the SD card.");
  } else {
    Serial.println("Error opening data file.");
  }
}

void getData(){
  getBMEData();
  gpsData();

}

void loop() {
    if (millis() - lastDoStateLogic >= 40)
    {
        lastDoStateLogic = millis();
       getData();
    }
    if (millis()-lastTransmit >= 1000){
      p_com += String(counter) + ',';
      p_com += String(temp1,2) + ',';
      p_com += String(temp2,2) + ',';
      p_com += String(press1,2) + ',';
      p_com += String(press2,2) + ',';
      p_com += String(altitude,2) + ',';
      p_com += String(lat,6) + ',';
      p_com += String(lng,6) + ',';
      p_com += String(galt,2) ;
  
      Serial.println(p_com);
      ss.println(p_com);
    
    //fan
    if (!fanRunning && temp1 >= 60) {
          turnOnFan();
          fanRunning = true;
          } else { 
            turnOffFan();
            fanRunning = false; 
         }
    //counter++, pump
    if(altitude >= 100){
      if(millis()-lastcount>= 1000){
        lastcount=millis();
        counter++;
        turnOnPump();
        pumpRunning = true;
        Serial.println(counter);
      }
    }
    // pump
    if(counter >= 9000) {
      turnOffPump();
      pumpRunning = false;
  }      
      // if (myFile) {
      //   Serial.print("Writing to test.txt...");
      //   myFile.println(p_com); // สั่งให้เขียนข้อมูล
      //   myFile.close(); // ปิดไฟล์
      //   Serial.println("done.");
      //   } else {
      //   // ถ้าเปิดไฟลืไม่สำเร็จ ให้แสดง error 
      //   Serial.println("error opening test.txt");
      //   }

   }
      

}
