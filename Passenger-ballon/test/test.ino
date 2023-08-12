#include <Adafruit_BNO08x.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>

#include <Wire.h>

#define BNO08X_RESET -1

Adafruit_BME280 bme;
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

float Gyro_X, Gyro_Y, Gyro_Z, Accel_X, Accel_Y, Accel_Z, Mag_X, Mag_Y, Mag_Z, Pointing_er, Accel_X2, Accel_Y2, Accel_Z2;
float roll, pitch, yaw, yaw_prev, delta_yaw, delta_point, EulerX, EulerY;

float temperature = bme.readTemperature();
float humidity = bme.readHumidity();
float pressure = bme.readPressure() / 100.0;
String p_com = "";

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

  if (!bno08x.enableReport(SH2_STEP_COUNTER)) {
    Serial.println("Could not enable step counter");
  }

  if (!bno08x.enableReport(SH2_RAW_ACCELEROMETER)) {
    Serial.println("Could not enable raw accelerometer");
  }
  if (!bno08x.enableReport(SH2_RAW_GYROSCOPE)) {
    Serial.println("Could not enable raw gyroscope");
  }
  if (!bno08x.enableReport(SH2_RAW_MAGNETOMETER)) {
    Serial.println("Could not enable raw magnetometer");
  }
}

void getBNOData() {

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
      Accel_X = (sensorValue.un.linearAcceleration.x);
      // Accel_X= KALMAN(Accel_X2);
      //    Serial.print(" y: ");
      Accel_Y = (sensorValue.un.linearAcceleration.y);
      // Accel_Y= KALMAN(Accel_Y2);
      //    Serial.print(" z: ");
      Accel_Z = (sensorValue.un.linearAcceleration.z);
      // Accel_Z= KALMAN(Accel_Z2);
      break;
    case SH2_GYROSCOPE_CALIBRATED:
      //    Serial.print("Gyro - x: ");
      Gyro_X = (sensorValue.un.gyroscope.x);
      //    Serial.print(" y: ");
      Gyro_Y = (sensorValue.un.gyroscope.y);
      //    Serial.print(" z: ");
      Gyro_Z = (sensorValue.un.gyroscope.z);
      break;
  }
}

  void setup() {
    Serial.begin(115200);
    Serial.begin(9600);
    bme.begin(0x76, &Wire);
    bno08x.begin_I2C(0x4A, &Wire, 74);

    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();
    float pressure = bme.readPressure() / 100.0;
  }

  void loop() {

    p_com += String(temperature, 2) + ",";
    p_com += String(humidity, 2) + ",";
    p_com += String(pressure, 2) + ",";
    p_com += String(Accel_X, 2) + ",";
    p_com += String(Accel_Y, 2) + ",";
    p_com += String(Accel_Z, 2) + ",";

    Serial.println(p_com);

    delay(1000);
  }
