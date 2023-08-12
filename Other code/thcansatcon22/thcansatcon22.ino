#include <Adafruit_BME280.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BNO08x.h>

#include <EEPROM.h>
//#include <InternalTemperature.h>
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include <SdFat.h>
//#include <SimpleKalmanFilter.h>
#include <TimeLib.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <utility/imumaths.h>
#include <vector>

#define SERVO_PARA_PIN 4
#define SERVO_BREAK_PIN 5

#define VOLTAGE_PIN 21
#define R1_OHM 2000.0F
#define R2_OHM 1250.0F

#define SEALEVELPRESSURE_HPA 1013.25
#define gpsSerial Serial3

#define loraCN Serial5
#define BNO08X_RESET -1

#define groundAltAddr 30
#define stateAddr 10
#define pkgAddr 0
#define timeAddr 50


const int chipSelect = 10;
time_t RTCTime;
Adafruit_BME280 bme;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);
Adafruit_BNO08x bno08x(BNO08X_RESET);

float groundAlt;
float apogee = INT_MIN;
const String PK_HEADER = "CAELUS";

int counter = 0;
float Gyro_X, Gyro_Y, Gyro_Z, Accel_X, Accel_Y, Accel_Z, Mag_X, Mag_Y, Mag_Z, Pointing_er , Accel_X2, Accel_Y2, Accel_Z2;
float roll, pitch, yaw, yaw_prev, delta_yaw, delta_point, EulerX, EulerY;
;
float altitude;
float temp;
float voltage;
char gpsTime[9] = "xx:xx:xx";
char timec[9] = "xx:xx:xx";

float gpsLat;
float gpsLng;
float gpsAlt;
int gpsSat;
int state = 0;
int lastDoStateLogic = 0;
int lastTransmit = 0;
int lastSD = 0;
float chosenAlt = 0;
float sensorAlt = 0;
float fil_alt = 0;
String p_com = "";
sh2_SensorValue_t sensorValue;

TinyGPSPlus gps;
Servo servoParachute;
Servo servoBreak;
SdFat sd;
SdFile myFile;


char cFileName[100];


String stateStr[5] = {"PRELAUNCH", "LAUNCH", "APOGEE", "PARADEPLOY", "LAND"};

inline String getStateString() 
{
    if (state < 0 || state > 4)
    {
        return "UNKNOWN";
    }
    return stateStr[state];
}
float KALMAN(float U);

float KALMAN(float U){
  static const float R = 3.00;
  static const float H = 1.00;
  static float Q = 15;
  static float P = 0;
  static float U_hat =0 ;
  static float K = 0;

  K = P*H/(H*P*H+R);
  U_hat = U_hat + K*(U-H*U_hat);

  P=(1-K*H)*P+Q;

  return U_hat;

}


void recovery()
{   
    //reset
    // EEPROM.put(groundAltAddr, 0);
    // EEPROM.update(pkgAddr, 0);
    // EEPROM.update(stateAddr,0);
    counter = EEPROM.read(pkgAddr);
    state = EEPROM.read(stateAddr);
    
    EEPROM.get(groundAltAddr, groundAlt);
    Serial.println(groundAlt);
    groundAlt = isnan(groundAlt) ? 0 : groundAlt;
}

void getGPSData()
{   
    gpsLat = gps.location.lat();
    gpsLng = gps.location.lng();
    sprintf(gpsTime, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    gpsAlt = gps.altitude.meters();
    gpsSat = gps.satellites.value();
}

void getBMEData()
{   
    if (groundAlt == 0){
       groundAlt = bme.readAltitude(SEALEVELPRESSURE_HPA);
       Serial.println("IM IN");
       Serial.println(groundAlt);
    }

    temp = bme.readTemperature();
    float sensorAlt = bme.readAltitude(SEALEVELPRESSURE_HPA);
    // Serial.println("alt");
    // Serial.println(sensorAlt);
    altitude = sensorAlt - groundAlt;
    // Serial.println("not fil");
    // Serial.println(altitude);
    fil_alt = KALMAN(altitude);
    // Serial.println("FILTER");
    // Serial.println(fil_alt);


    if (fil_alt >= apogee) {
      apogee = fil_alt;
    
}


void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
    Serial.println("Could not enable accelerometer");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("Could not enable gyroscope");
  }
  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
    Serial.println("Could not enable magnetic field calibrated");
  }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
    Serial.println("Could not enable linear acceleration");
  }
  if (!bno08x.enableReport(SH2_GRAVITY)) {
    Serial.println("Could not enable gravity vector");
  }
//  if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
//    Serial.println("Could not enable rotation vector");
//  }
//  if (!bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR)) {
//    Serial.println("Could not enable geomagnetic rotation vector");
//  }
//  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
//    Serial.println("Could not enable game rotation vector");
//  }
  if (!bno08x.enableReport(SH2_STEP_COUNTER)) {
    Serial.println("Could not enable step counter");
  }
//  if (!bno08x.enableReport(SH2_STABILITY_CLASSIFIER)) {
//    Serial.println("Could not enable stability classifier");
//  }
  if (!bno08x.enableReport(SH2_RAW_ACCELEROMETER)) {
    Serial.println("Could not enable raw accelerometer");
  }
  if (!bno08x.enableReport(SH2_RAW_GYROSCOPE)) {
    Serial.println("Could not enable raw gyroscope");
  }
  if (!bno08x.enableReport(SH2_RAW_MAGNETOMETER)) {
    Serial.println("Could not enable raw magnetometer");
  }
//  if (!bno08x.enableReport(SH2_SHAKE_DETECTOR)) {
//    Serial.println("Could not enable shake detector");
//  }
//  if (!bno08x.enableReport(SH2_PERSONAL_ACTIVITY_CLASSIFIER)) {
//    Serial.println("Could not enable personal activity classifier");
//  }
}
void getBNOData()
{

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }
  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }
  switch (sensorValue.sensorId) {

  case SH2_LINEAR_ACCELERATION:
//    Serial.print("Accelerometer - x: ");
    Accel_X=(sensorValue.un.linearAcceleration.x)
    // Accel_X= KALMAN(Accel_X2);
//    Serial.print(" y: ");
    Accel_Y=(sensorValue.un.linearAcceleration.y);
    // Accel_Y= KALMAN(Accel_Y2);
//    Serial.print(" z: ");
    Accel_Z=(sensorValue.un.linearAcceleration.z);
    // Accel_Z= KALMAN(Accel_Z2);
    break;
  case SH2_GYROSCOPE_CALIBRATED:
//    Serial.print("Gyro - x: ");
    Gyro_X=(sensorValue.un.gyroscope.x);
//    Serial.print(" y: ");
    Gyro_Y=(sensorValue.un.gyroscope.y);
//    Serial.print(" z: ");
    Gyro_Z=(sensorValue.un.gyroscope.z);
    break;
  case SH2_RAW_MAGNETOMETER:
//    Serial.print("Magnetic Field - x: ");
    Mag_X=(sensorValue.un.rawMagnetometer.x);
//    Serial.print(" y: ");
    Mag_Y=(sensorValue.un.rawMagnetometer.y);
//    Serial.print(" z: ");
    Mag_Z=(sensorValue.un.rawMagnetometer.z);
    break;
}
}

void getBattery()
{
    float apparentVoltage = analogRead(VOLTAGE_PIN) * 3.3 / 1023.0;
    voltage = apparentVoltage * ((R1_OHM + R2_OHM) / R2_OHM);
}

time_t getTeensy3Time() {
    return Teensy3Clock.get();
}

void stateLogic()
{
    getBMEData();
    getBNOData();
    chosenAlt = fil_alt;

    switch (state)
    {
    // PRELAUNCH
    case 0:
        EEPROM.put(groundAltAddr, groundAlt);
        // LAUNCH
        if (chosenAlt >= 60)
        {
            state = 1;
        }
        break;

    case 1:
        // APOGEE
        if (apogee - chosenAlt >= 10 && chosenAlt >= 60)
        {
            state = 2 ;
        }
        break;

    case 2:
        // PARADEPLOY
        if (chosenAlt <= apogee - 15)
        {
        //  servoParachute.write(180);
         state = 3;
        }
        break;

    case 3:
        // LAND
        if (chosenAlt <= 5)
        {
            state = 4;
        }
    }
    EEPROM.update(stateAddr, state);
}

// EEPROM.update(pkgAddr, counter);
// EEPROM.update(stateAddr, state);
// EEPROM.update(groundAltAddr, groundAlt);


  
  
  
  

void setup()
{ 
    Serial.begin(115200);
    gpsSerial.begin(9600);
    loraCN.begin(9600);
    bno08x.begin_I2C(0x4A,&Wire1,74);
    bme.begin(0x76, &Wire1);
    servoParachute.attach(SERVO_PARA_PIN);
    servoBreak.attach(SERVO_BREAK_PIN);
    pinMode(VOLTAGE_PIN, INPUT);
    setSyncProvider(getTeensy3Time);
    if (!sd.begin(chipSelect, SPI_HALF_SPEED)) sd.initErrorHalt();
    
  // open the file for write at end like the Native SD library

    
    recovery();
    
}

void loop()
{   
    while (gpsSerial.available())
        gps.encode(gpsSerial.read());
        getGPSData();

    if ( loraCN.available())
    {
        sprintf(timec , "%02d:%02d:%02d", hour(), minute(), second());
//        counter++;
    }

    if (millis() - lastDoStateLogic >= 40)
    {
        lastDoStateLogic = millis();
        sprintf(timec , "%02d:%02d:%02d", hour(), minute(), second());
        getBattery();
        stateLogic();
    }

    if (millis() - lastTransmit >= 1000)
    {
        lastTransmit = millis();
        counter++;
        p_com = PK_HEADER + ',';
        p_com += String(counter) + ',';
        p_com += String("C") + ',';
        p_com += String(timec) + ',';
        p_com += String(gpsLat,6) + ',';
        p_com += String(gpsLng,6) + ',';
        p_com += String(gpsAlt,2) + ',';
        // p_com += String(gpsSat) + ',';
        p_com += String(temp) + ',';
        p_com += String(voltage) + ',';
        p_com += String(fil_alt) + ',';
        // p_com += String(Gyro_X, 2) + ",";
        // p_com += String(Gyro_Y, 2) + ",";
        // p_com += String(Gyro_Z, 2) + ",";
        p_com += String(Accel_X, 2) + ",";
        p_com += String(Accel_Y, 2) + ",";
        p_com += String(Accel_Z, 2) + ",";
        // p_com += String(Mag_X, 2) + ",";
        // p_com += String(Mag_Y, 2) + ",";
        // p_com += String(Mag_Z, 2) + ",";
        p_com += String(stateStr[state]);

        loraCN.println(p_com);
        EEPROM.update(pkgAddr, counter);
        Serial.println(p_com);
        //Write file
     

        // if (!myFile.open("test1.csv", O_READ)) {
        //   sd.errorHalt("opening cansattest.csv for read failed");
          // }
        if (!myFile.open("rockettest.csv", O_RDWR | O_CREAT | O_AT_END)) {
        sd.errorHalt("opening test.txt for write failed");
        }
           myFile.println(p_com); 

        // Serial.println("test.txt:");

        // read from the file until there's nothing else in it:
        // int data;

        // while ((data = myFile.read()) >= 0) Serial.write(data);
        myFile.close();
  


        }
    
}
