#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define BME280_I2C_ADDR1 0x76 // I2C address of the first BME280 sensor
#define BME280_I2C_ADDR2 0x77 // I2C address of the second BME280 sensor
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme1;
Adafruit_BME280 bme2;
String p_com = "";
float sensorAlt = bme1.readAltitude(SEALEVELPRESSURE_HPA);

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // Initialize the first BME280 sensor
  if (!bme1.begin(BME280_I2C_ADDR1)) {
    Serial.println("Could not find a valid BME280 sensor at address 0x76");
    while (1);
  }

  // Initialize the second BME280 sensor
  if (!bme2.begin(BME280_I2C_ADDR2)) {
    Serial.println("Could not find a valid BME280 sensor at address 0x77");
    while (1);
  }
}

void loop() {
  // BME 1st
  float temp1 = bme1.readTemperature();
  float press1 = bme1.readPressure() / 100.0;
  float hum1 = bme1.readHumidity();

  // BME 2nd
  float temp2 = bme2.readTemperature();
  float press2 = bme2.readPressure() / 100.0;
  float hum2 = bme2.readHumidity();

  // Print the sensor values
  p_com = String(temp1) + ',';
  p_com += String(press1) + ',';
  p_com += String(hum1) + ',';
  p_com += String(sensorAlt) + ',';
  
  p_com += String(temp2) + ',';
  p_com += String(press2) + ',';
  p_com += String(hum2) + ',';

  Serial.println(p_com);
  
  delay(2000); // Delay between readings
  
}
