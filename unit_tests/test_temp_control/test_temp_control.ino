#include <PID_v1.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Si7021.h>

/*
 *This next section defines a a subclass of the tempurature sensing
 *hardware as Tempsensor which will hold the information from the last
 *time the sensor was polled
 *
 */

class Tempsensor: public Adafruit_Si7021 {
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
//    Serial.print("Current millis() = "); Serial.println(currentMillis);

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

class Fan {
  private:
    byte outPin;      // the pin which the fan control is attached to

  public:
    byte byteControl; // number from 0-255 which is proportional to the speed of the fan
    
  Fan(byte pin) {
    outPin = pin;     
    /* adjust timer 2 to give pin 3 and 11 25kHz output*/
    TCCR2A = 0x23;
    TCCR2B = 0x09;    // select timer2 clock
    OCR2A = 79;       // aiming for 25kHz, this is the upper limit of speed
    OCR2B = 30;       // set the PWM duty cycle, this is the lower limit of speed
  }

  void Configure() {
    pinMode(outPin, OUTPUT);
  }
  void set_speed(double output) {
    byteControl = byte(output);                     // ideally will be the PWM output of the PID controller
    OCR2B = map(byteControl, 0, 255, 30, 79);       // fan speed will thus be proportional to the PID output
  }
};

class Peltier: public PID {
  private:
    byte outPin;                // the pin which the peltier control is attached to
    byte readInterval;  // the time in milliseconds between readings
    unsigned long lastUpdate;
    const double Kp = 150;
    const double Ki = 20;
    const double Kd = 1;

  public:
    double pidInput;
    double pidOutput;
    double setPoint;
    double gap;

  Peltier(byte pin, byte seconds, double newSetPoint = 25) : PID(&pidInput, &pidOutput, &setPoint, Kp, Ki, Kd, REVERSE) {
    outPin = pin;
//    pidInput = 100;
//    pidOutput = 100;
    readInterval = seconds;
    setPoint = newSetPoint;
    lastUpdate = millis();
  }

//  void Configure() {
//    SetSampleTime(readInterval);
//  }

  void set_peltier(double newSetPoint) {
    setPoint = newSetPoint;
  }

  void Update(Tempsensor sensor) {
    unsigned long currentMillis = millis();
//    Serial.print("inside update"); Serial.println();

    if((currentMillis - lastUpdate) >= (readInterval * 1000)) {
      lastUpdate = millis();
      pidInput = sensor.temp;
      gap = pidInput - setPoint;
//      Serial.print("update triggered"); Serial.println();
//      Serial.print("gap = "); Serial.println(gap);
      
      if (gap > 1) {
        pidOutput = 255;
      }
      else {
        Compute();
//        Serial.print("PWM output = "); Serial.println(pidOutput);       
      }
      
      analogWrite(outPin, pidOutput);
    }
  }

  void testprint() {
    unsigned long currentMillis = millis();

    if((currentMillis - lastUpdate) >= (readInterval * 1000)) {
      Serial.print("Current millis() = "); Serial.println(currentMillis);
      Serial.print("Millis of last poll = "); Serial.println(lastUpdate);
      Serial.print("gap = "); Serial.println(gap);
      Serial.print("Temp *C = "); Serial.println(pidInput);
      Serial.print("PID output = "); Serial.println(pidOutput);
      Serial.println();
    }
  }
};

Fan testfan = Fan(3);                        // initialize the fan
Peltier testpeltier = Peltier(5, 2, 23);     // initialize the peltier
Tempsensor testsensor = Tempsensor(2);       // initialize the sensor


/**************************************************************************/
/*
    Setup code
*/
/**************************************************************************/

void setup() {

  /* set up the output pins */

  pinMode(9, OUTPUT);
  pinMode(5, OUTPUT);

  /* set up serial */
  Serial.begin(9600);

  while (!Serial)
    delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Si7021 test");
  if (! testsensor.begin()) {   // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
    while (1);
  }
  
  /*turn the PID on*/
//  testpeltier.Configure();
  testpeltier.SetMode(AUTOMATIC); 

  testfan.Configure();

  /*test the hardware*/
  Serial.print("touch peltier, keep the fan in front of you test begins in"); Serial.println();
  Serial.print("3"); Serial.println();
  delay(1000);
  Serial.print("2"); Serial.println();
  delay(1000);
  Serial.print("1"); Serial.println();
  delay(1000);
  Serial.print("fan low and peltier on"); Serial.println();
  analogWrite(5, 255);
  delay(5000);
  Serial.print("fan low and peltier off"); Serial.println();
  analogWrite(5, 0);
  delay(5000);
  Serial.print("fan high and peltier on"); Serial.println();
  testfan.set_speed(255);
  analogWrite(5, 255);
  delay(5000);
  testfan.set_speed(0);
  analogWrite(5, 0);
  Serial.print("fan test on pin 3 and peltier on pin 5 complete"); Serial.println();
}
  
/**************************************************************************/
/*
    Main loop code
*/
/**************************************************************************/

void loop() {
  testsensor.Update();
//  testsensor.testprint();
//  Serial.print("tick tick tick"); Serial.println();
  testpeltier.testprint();
  testpeltier.Update(testsensor);
  testfan.set_speed(testpeltier.pidOutput);
}
