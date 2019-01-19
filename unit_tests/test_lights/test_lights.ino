#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>

/* IC2 addr 0x39. If you set the ADDR pin high
   or low, use TSL2561_ADDR_HIGH (0x49) or TSL2561_ADDR_LOW
   (0x29) respectively.
*/

class Lightsensor: public Adafruit_TSL2561_Unified {
  private:
    byte readInterval;            // the time in seconds between readings

  public:
    sensors_event_t event;        // a structure defined in the unified sensor library which contains the measurements
/*
 *Constructor, also calls the base class constrcutor
 *
 */   
  Lightsensor(byte seconds) : Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 333) {  // initialize constructor with base class constructor
    readInterval = seconds;
  }

  void Update() {
    unsigned long currentMillis = millis();
    
    if((currentMillis - event.timestamp) >= (readInterval * 1000)) {
      getEvent(&event);
      }
    }

  void Configure() {
//    enableAutoRange(true);                                       // set the gain to automatic
    setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);           // set the medium resolution and speed 
  }

  uint32_t Readlux() {
    getEvent(&event);
    return event.light;
  }

  void testprint() {
    unsigned long currentMillis = millis();

    if((currentMillis - event.timestamp) >= (readInterval * 1000)) {
      Serial.print("Current millis() = "); Serial.println(currentMillis);
      Serial.print("Millis of last poll = "); Serial.println(event.timestamp);
      Serial.print("lux = "); Serial.println(event.light);
      Serial.println();
    }
  }
};

class Ledbar {
  private:
    byte outPin;               // the pin which the LED transistor is attached to
    byte outPwm;               // the PWM output of the pin
    byte setInterval;          // the time in seconds between checking if the
    unsigned long lastUpdate;  // to store the millis 
    bool night;                // if true then the LED should be off

  public:
    unsigned int setLux;       // the set point of the system in lux

  Ledbar(byte pin, byte seconds, unsigned int set = 200) {
    night = false;
    outPin = pin;
    setLux = set;
    setInterval = seconds;
    lastUpdate = millis();
    outPwm = 120;
  }

  void Configure() {
    pinMode(outPin, OUTPUT);
    analogWrite(outPin, outPwm);
  }

  void set_night(unsigned long dusk, unsigned long dawn, unsigned long present) {
    if (present >= dusk && present <= dawn) {
      analogWrite(outPin, 0);
      night = true;
    }
    else {
      analogWrite(outPin, outPwm);
      night = false;
    }
  }

  void Update(Lightsensor sensor) {
    unsigned long currentMillis = millis();
    unsigned int measuredLux = sensor.Readlux();

    if((currentMillis - lastUpdate) >= (setInterval * 1000)) {                       // if the time is right to update
      lastUpdate = millis();
//      Serial.println("inside first if");    
      if (night == false) {                                                          // [and] if the night is not toggled
//        Serial.println(setLux < measuredLux);
//        Serial.println(setLux > measuredLux);
//        Serial.println(abs(setLux - measuredLux));
//        Serial.println(abs(setLux - measuredLux) >= 20);
        while ((setLux < measuredLux) && (abs(setLux - measuredLux) >= 20)) {       // adjust the PWM on the transistor for the LED downwards if the light are too high
          analogWrite(outPin, --outPwm);
          measuredLux = sensor.Readlux();
//          delay(1000);
          Serial.print(measuredLux); Serial.println(": going down");
        }
        while ((setLux > measuredLux) && (abs(setLux - measuredLux) >= 20)) {       // adjust the PWM on the transistor for the LED upward if the lights are too low
          analogWrite(outPin, ++outPwm);
          measuredLux = sensor.Readlux();
//          delay(1000);
          Serial.print(measuredLux); Serial.println(": going up");
        }
      }
    }
  }
  
};


Lightsensor testsensor = Lightsensor(5);    // initialize the sensor
Ledbar lights = Ledbar(9, 30, 4000);        // initialize the lights

void setup() {

  pinMode(3, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  Serial.begin(9600);
  
  while (!Serial)
    delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("COnfigure the sensor and start the lights");
  testsensor.Configure();  // configure the sensor before the lights!
  delay(1000);
  lights.Configure();
  Serial.println("LED testing begins now");

}

void loop() {
  testsensor.testprint();
  testsensor.Update();
//  Serial.print("lux = "); Serial.println(testsensor.Readlux());
  lights.Update(testsensor);
  unsigned long nighttime = millis();
  lights.set_night(40000, 70000, nighttime);
//  Serial.println("night toggled");
}
