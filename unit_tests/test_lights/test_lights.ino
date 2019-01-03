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
    enableAutoRange(true);                                       // set the gain to automatic
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
  }

  void toggle_night() {
    night = ~night;
  }

  void Update(Lightsensor sensor) {
    unsigned long currentMillis = millis();
    unsigned int measuredLux = sensor.Readlux();

    if((currentMillis - lastUpdate) >= (setInterval * 1000)) {    
      if (night == false) {
        while (setLux < measuredLux) {
          analogWrite(outPin, ++outPwm);
          measuredLux = sensor.Readlux();
//          delay(1000);
          Serial.print(setLux - measuredLux); Serial.println(": going up");
        }
        while (setLux > measuredLux) {
          analogWrite(outPin, --outPwm);
          measuredLux = sensor.Readlux();
//          delay(1000);
          Serial.print(setLux -  measuredLux); Serial.println(": going dowm");
        }
      }  
    }
  }
  
};


Lightsensor testsensor = Lightsensor(2);    // initialize the sensor
Ledbar lights = Ledbar(9, 10, 2000);              // initialize the lights

void setup() {

  pinMode(5, OUTPUT);
  pinMode(3, OUTPUT);
  Serial.begin(9600);
  
  while (!Serial)
    delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("LED testing begins now");
  lights.Configure();
  testsensor.Configure();

}

void loop() {
  testsensor.testprint();
  testsensor.Update();
//  digitalWrite(9, 255);
  Serial.print("lux = "); Serial.println(testsensor.Readlux());
  lights.Update(testsensor);
}
