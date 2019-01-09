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

Lightsensor testsensor = Lightsensor(2);  // initialize the sensor

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/

void displaySensorDetails(void)
{
  sensor_t sensor;
  testsensor.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}


/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/

void setup(void) 
{
  /* Set the other attached pins so they don't float and cause issues */
  pinMode(9, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(3, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("Light Sensor Test"); Serial.println("");

  testsensor.Configure();
  
  /* Initialise the sensor */
  //use tsl.begin() to default to Wire, 
  //tsl.begin(&Wire2) directs api to use Wire2, etc.
  if(!testsensor.begin())
  {
    /* There was a problem detecting the TSL2561 ... check your connections */
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  displaySensorDetails();

}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void) 
{  
  testsensor.testprint();
  testsensor.Update();
//  Serial.print("lux = "); Serial.println(testsensor.event.light);
}
