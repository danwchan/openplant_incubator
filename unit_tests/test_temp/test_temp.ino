/*************************************************** 
  This is an test case to look at the function of the temperature
  sensor when the entire 

  Designed specifically to work with the SHT31-D sensor from Adafruit
  ----> https://www.adafruit.com/products/2857

  These sensors use I2C to communicate, 2 pins are required to  
  interface
 ****************************************************/
 
#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"

/*
 *This next section defines a a subclass of the tempurature sensing
 *hardware as Tempsensor which will hold the information from the last
 *time the sensor was polled
 *
 */

class Tempsensor: public Adafruit_SHT31 {
  private:
    byte readInterval;            // the time in seconds between readings
    unsigned long lastUpdate;     // to store the millis

  public:
    float temp;                   // to store the tempurature data
    float humidity;               // to store the humidity data

/*
 *Constructor, also should call the base class constrcutor
 *
 */
   
  Tempsensor(byte seconds) {
    readInterval = seconds;
    temp = 0;
    humidity = 0;
    lastUpdate = millis();
  }

  void Update() {
    unsigned long currentMillis = millis();

    if((currentMillis - lastUpdate) >= (readInterval * 1000)) {
      lastUpdate = millis();
      temp = readTemperature();
      humidity = readHumidity();
    }
  }

  void testprint() {
    unsigned long currentMillis = millis();

    if((currentMillis - lastUpdate) >= (readInterval * 1000)) {
      Serial.print("Current millis() = "); Serial.println(currentMillis);
      Serial.print("Millis of last poll = "); Serial.println(lastUpdate);
      Serial.print("Temp *C = "); Serial.println(temp);
      Serial.print("Hum. % = "); Serial.println(humidity);
      Serial.println();
    }
  }
};

Tempsensor testsensor = Tempsensor(2); // initialize the sensor

void setup() {

/*
 *Set the pins which are attached to other devices to output to
 *prevent a floating pin which can casues issues
 *
 */
  pinMode(9, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(3, OUTPUT);
  
  Serial.begin(9600);

  Serial.println("Si7021 test");
  if (! testsensor.begin()) {   // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
    while (1);
  }
}


void loop() {
  
  testsensor.testprint();
  testsensor.Update();
}
