#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)
HardwareSerial &ss = Serial1;
int counter= 0;
Adafruit_BME280 bme;
int lastcount=0;
float alt;

void setup() {
	Serial.begin(9600);
  Serial.begin(115200);
  ss.begin(9600);

	if (!bme.begin(0x77, &Wire)) {
		ss.println("Could not find a valid BME280 sensor, check wiring!");
		while (1);
	}
}

void loop() {
	ss.print("Temperature = ");
	ss.print(bme.readTemperature());
	ss.println("*C");

	Serial.print("Pressure = ");
	Serial.print(bme.readPressure() / 100.0F);
	Serial.println("hPa");

	Serial.print("Approx. Altitude = ");
	Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  alt=bme.readAltitude(SEALEVELPRESSURE_HPA);
	Serial.println("m");

	Serial.print("Humidity = ");
	Serial.print(bme.readHumidity());
	Serial.println("%");

	Serial.println();
	delay(1000);

      
    
}