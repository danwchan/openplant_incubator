#include <Arduino.h>            // arduino built in functions
#include <Wire.h>               // i2c library for connecting over SDA and SCL
#include <SPI.h>                // SPI library for connecting over MOSI, MISO, CLK
#include <SD.h>                 // SD card wrapper library
#include "RTClib.h"             // Date and time functions using a DS1307 RTC connected via I2C and Wire lib
#include <PID_v1.h>             // PID library
#include "Adafruit_SHT31.h"    // tempurature sensor library
#include <Adafruit_Sensor.h>    // Adafruit unified sensor library
#include <Adafruit_TSL2561_U.h> // light sensor library
//#include <Adafruit_GFX.h>       // Adafruit graphics library
//#include <Adafruit_SSD1306.h>   // OLED display library

//#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
//   #define Serial SerialUSB
//#endif
/*
class Screen: public Adafruit_SSD1306 {
  private:
    uint8_t ic2Address;
    unsigned long lastUpdate;
    unsigned long readInterval;  // this is in milliseconds unlike the rest of the sensors which are in seconds

  public:

  Screen(uint8_t addr, unsigned long millisec = 200) : Adafruit_SSD1306(128, 64, &Wire, -1) { // base class constructor params: screen_width, screen_height (in pixels), wire library pointer, oled_reset (pin)
    ic2Address = addr;
    readInterval = millisec;
    lastUpdate = millis();
  }

  Configure() {
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!begin(SSD1306_SWITCHCAPVCC, ic2Address)) {
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }
  }

  void Update(DateTime *clockEvent, float *lux, unsigned int *setLux, float *temp, double *setTemp) {
    unsigned long currentMillis = millis();
//    Serial.print("Current millis() = "); Serial.println(currentMillis);

    if((currentMillis - lastUpdate) >= (readInterval)) {  // this is in milliseconds unlike the rest of the sensors which are in seconds
      lastUpdate = millis();
      
      clearDisplay();

      setTextSize(1); // Draw 1X-scale text
      setTextColor(WHITE);
//      cp437(true);

      setCursor(0, 0);      // the status bar month and day 
      print(clockEvent->month());print(F("/"));print(clockEvent->day());
      setCursor(95, 0);    // the status bar time
      print(clockEvent->hour());print(F(":"));print(clockEvent->minute());

      
      //This is the grid which lays out the set points
      
      setCursor(45, 17);    // the set column label
      print(F("Set"));
      setCursor(80, 16);    // the actual column label
      print(F("Actual"));
      setCursor(5, 28);     // the light row label
      println(F("Light"));
      print(F("(lux)"));
      setCursor(5, 48);     // the temp row label
      println(F("Temp")); print(F("(")); write(9); print(F("C)"));

      setCursor(80, 32);    // light actual level
      print(*lux);
      setCursor(45, 32);    // light set point
      print(*setLux);
      setCursor(80, 52);    // temp actual level
      print(*temp);
      setCursor(45, 52);    // temp set point
      print(*setTemp);

      display();            // write to display
    }
  }

  
  //  This code is for some features which have yet to be implemented
  //    1) a place to display warnings
  //    2) arrows to indicate the adjustments the incubator is making
  
  Draw_extra() {

    setTextColor(BLACK, WHITE);// the preferred style to make it pop

    setCursor(0, 8);           // the row for error messages
    print(F("WARNINGS!!!"));

    setCursor(118, 32);        // the light row position for arrows
    write(24);                 // an up arrow
    setCursor(118, 52);        // the temp row position for arrows
    write(25);                 // a down arrow
  }
  
};
*/
class Sdlogger: public SDClass {
  private:
    byte readInterval;
    byte chipSelect;
    unsigned long lastUpdate;

  public:

  Sdlogger(byte pin, byte seconds){
    chipSelect = pin;
    readInterval = seconds;
    lastUpdate = millis();
  }

  void Configure() {
    // see if the card is present and can be initialized:
    if (!begin(chipSelect)) {
      Serial.println("Card failed, or not present");
      // don't do anything more:
      while (1);
    }
  }

  bool Update(String timeID, float *temp, float *humidity, float *lux, double *pidOutput) {
    unsigned long currentMillis = millis();

    if((currentMillis - lastUpdate) >= (readInterval * 1000)) {
      lastUpdate = millis();
      String fileName = timeID.substring(2,8);
      fileName += ".log";
      File dataFile = open(fileName, FILE_WRITE);
//      Serial.println(fileName);
      
      if (dataFile) {                                                // if the file is available, write to it:
        dataFile.print(timeID); dataFile.print(", ");
        dataFile.print(*temp); dataFile.print(", ");
        dataFile.print(*humidity); dataFile.print(", ");
        dataFile.print(*lux); dataFile.print(", ");
        dataFile.println(*pidOutput);
        dataFile.close();
        Serial.print("data written to: "); Serial.println(fileName);
        return true;
      }
      else {                                                         // if the file isn't open, pop up an error:
        return false;
      }
    }
    return false;
  }
};

class Lightsensor: public Adafruit_TSL2561_Unified {
  private:
    byte readInterval;            // the time in seconds between readings

  public:
    sensors_event_t event;        // a structure defined in the unified sensor library which contains the measurements
  /*
   *Constructor, also calls the base class constrcutor
   * IC2 addr 0x39. If you set the ADDR pin high or low, use TSL2561_ADDR_HIGH (0x49) or TSL2561_ADDR_LOW (0x29) respectively.
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
//    enableAutoRange(true);                                     // set the gain to automatic
    setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);           // set the medium resolution and speed 
  }

  uint32_t Readlux() {
    getEvent(&event);
    return event.light;
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
        while ((setLux < measuredLux) && (abs(setLux - measuredLux) >= 20)) {        // adjust the PWM on the transistor for the LED downwards if the light are too high
          analogWrite(outPin, --outPwm);
          measuredLux = sensor.Readlux();
//          delay(1000);
//          Serial.print(measuredLux); Serial.println(": going down");
        }
        while ((setLux > measuredLux) && (abs(setLux - measuredLux) >= 20)) {        // adjust the PWM on the transistor for the LED upward if the lights are too low
          analogWrite(outPin, ++outPwm);
          measuredLux = sensor.Readlux();
//          delay(1000);
//          Serial.print(measuredLux); Serial.println(": going up");
        }
      }
    }
  }
};

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
    float temp;                     // to store the tempurature data
    float humidity;                 // to store the humidity data

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
//      Serial.println("temperature sesnsor polling...");
      lastUpdate = millis();
      temp = readTemperature();
      humidity = readHumidity();
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
    byte ncPos;                 // the pin that controls the relay when normally closed has positive voltage
    byte ncNeg;                 // the pin that controls the relay when normally closed has negative voltage
    byte readInterval;          // the time in seconds between readings
    unsigned long lastUpdate;
    const double Kp = 150;
    const double Ki = 20;
    const double Kd = 1;

  public:
    double pidInput;
    double pidOutput;
    double setPoint;
    double gap;

    #define COOL true
    #define WARM false

  Peltier(byte pin, byte positive, byte negative, byte seconds, double newSetPoint = 25) : PID(&pidInput, &pidOutput, &setPoint, Kp, Ki, Kd, REVERSE) {
    outPin = pin;
    ncPos = positive;
    ncNeg = negative;
    pidInput = newSetPoint;
//    pidOutput = 100;
    readInterval = seconds;
    setPoint = newSetPoint;
    lastUpdate = millis();
  }

  void Configure() {
    SetTunings(Kp, Ki, Kd);
    pinMode(ncPos, OUTPUT);
    pinMode(ncNeg, OUTPUT);
    digitalWrite(ncPos, LOW);
    digitalWrite(ncNeg, LOW);
//    SetSampleTime(readInterval);
  }

  void set_peltier(double newSetPoint) {
    setPoint = newSetPoint;
  }

  void Update(Tempsensor sensor) {
    unsigned long currentMillis = millis();

    if((currentMillis - lastUpdate) >= (readInterval * 1000)) {
      lastUpdate = millis();
      pidInput = sensor.temp;
      gap = pidInput - setPoint;
//      Serial.print("peltier update triggered"); Serial.println();
//      Serial.print("gap = "); Serial.println(gap);
      
      if (gap < -0.5  && pidOutput == 0) {
        toggle_heat(WARM);
      }
      else if (gap > 0.5 && pidOutput == 0) {
        toggle_heat(COOL);
      }
      
      if (abs(gap) > 1.5) {
        pidOutput = 255;
      }
      else {
        Compute();
//        bool test = Compute();
        Serial.print("PWM intput = "); Serial.println(pidInput);
        Serial.print("PWM output = "); Serial.println(pidOutput);
//        Serial.print("Did compute run?: "); Serial.println(test);
//        Serial.print("Kp (via function): "); Serial.println(GetKp());
//        Serial.print("Kp (via variable call): "); Serial.println(Kp);
      }
      
      analogWrite(outPin, pidOutput);
    }
  }

  void toggle_heat(bool mode) {
    if (!mode) {                            // for WARM
      SetControllerDirection(DIRECT);       // set the PID direction
      analogWrite(outPin, 0);               // turn off the current while the relays swtich
      digitalWrite(ncPos, HIGH);            // switch the relays away from their normal closed state to the alternate
      digitalWrite(ncNeg, HIGH);
      delay(1000);                          // wait for the relay to work
      analogWrite(outPin, pidOutput);       // before applying the PID again
    }
    else {
      SetControllerDirection(REVERSE);      // for COOL, do the same procedue to set the relays back to their normally closed path
      digitalWrite(ncPos, LOW);
      digitalWrite(ncNeg, LOW);
      delay(1000);
      analogWrite(outPin, pidOutput);
    }
  }

/*
    void testprint() {
    unsigned long currentMillis = millis();

    if((currentMillis - lastUpdate) >= (readInterval * 1000)) {
      Serial.print("Current millis() = "); Serial.println(currentMillis);
      Serial.print("Millis of last poll = "); Serial.println(lastUpdate);
      Serial.print("gap = "); Serial.println(gap);
      Serial.print("PID input = "); Serial.println(pidInput);
      Serial.print("PID output = "); Serial.println(pidOutput);
      Serial.println();
    }
  }
*/
};

class Rtclock: public RTC_PCF8523 {
  private:
    byte readInterval;            // the time in seconds between readings

  public:
    DateTime event;        // a structure defined in the RTC library which contains the time
/*
 *Constructor, also calls the base class constrcutor
 *
 */   
  Rtclock() {
  }

  String getTime() {
    event = now();
    char buffer[18];
    snprintf(buffer, 18, "%04d%02d%02d_%02d:%02d:%02d", event.year(), event.month(), event.day(), event.hour(), event.minute(), event.second());
    return buffer;
  }

  void Configure() {
    begin();
    adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
};

Rtclock RTC = Rtclock();                          // initialize the RTC
Fan CPUfan = Fan(3);                              // initialize the fan (pin)
Peltier Heatercooler = Peltier(5, 7, 6, 2, 21);   // initialize the peltier (pin, update in sec, set point)
Tempsensor Tempbreakout = Tempsensor(2);          // initialize the sensor (update in sec)
Lightsensor Luxbreakout = Lightsensor(5);         // initialize the sensor (update in sec)
Ledbar Lights = Ledbar(9, 30, 3800);              // initialize the lights (pin, update in sec, set point)
Sdlogger Logger = Sdlogger(10, 2);                // initialize the SD card (CS pin, update in sec)
//Screen Display = Screen(0x3C);                  // initialize the OLED display (IC2 Address 0x3D for 128x64)

void setup () {
  
//  #ifndef ESP8266
//    while (!Serial);   // for Leonardo/Micro/Zero
//  #endif

  Serial.begin(9600);  // start serial connection

  while (!Serial)
    delay(10);         // will pause Zero, Leonardo, etc until serial console opens

  /*configure the clock*/
  RTC.Configure();
//  Serial.println("clock variable set");
  if (! RTC.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  Serial.print("The time string is: "); Serial.print(RTC.getTime());
  
  /*configure the tempurature sensor*/
//    Serial.println("Configuring the tempurature sensor");
  if (! Tempbreakout.begin()) {   // Set to 0x45 for alternate i2c addr
//    Serial.println("Couldn't find tempurature sensor");
    while (1);
  }
  
  /*turn the PID on the peltier on*/
//  Serial.println("Configure PID peltier control and the fan");
  Heatercooler.Configure();
  Heatercooler.SetMode(AUTOMATIC); 

  /*configure the fan*/
  CPUfan.Configure();
  
  /*configure the sensor before the lights!*/
//  Serial.println("Configure the sensor and start the lights");
  Luxbreakout.Configure();  // 

  /*configure the lights*/
  Lights.Configure();

  /*configure the SD log*/
//  Serial.println("Configure the SD card");
  Logger.Configure();

  /*configure the OLED display*/
//  Serial.println("Configure the OLED display");
//  Display.Configure();
}

void loop () {
  Tempbreakout.Update();                      // the temperature sensor polls the tempurature
  Heatercooler.Update(Tempbreakout);          // the peltier responds to the tempurature
//  Heatercooler.testprint();
  CPUfan.set_speed(Heatercooler.pidOutput);   // the fan speed is set proportionally to the pidOutput
  Luxbreakout.Update();                       // the light sensor polls the lux level
  Lights.Update(Luxbreakout);                 // the LEDs respond to the lux level
  
  // below the data is logged to the SD card
  Logger.Update(RTC.getTime(), &Tempbreakout.temp, &Tempbreakout.humidity, &Luxbreakout.event.light, &Heatercooler.pidOutput);

  // below the OLED screen is updated with information
//   Display.Update(&RTC.event, &Luxbreakout.event.light, &Lights.setLux, &Tempbreakout.temp, &Heatercooler.setPoint);
  
  Lights.set_night(4, 12, RTC.event.hour());  // toggle the night at midnight and toggle it back at 4AM
}
