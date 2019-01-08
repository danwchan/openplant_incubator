// Date and time functions using a DS1307 RTC connected via I2C and Wire lib
#include <Wire.h>
#include "RTClib.h"

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
   #define Serial SerialUSB
#endif


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

Rtclock testclock = Rtclock();

void setup () {
  pinMode(9, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(3, OUTPUT);
  
#ifndef ESP8266
  while (!Serial); // for Leonardo/Micro/Zero
#endif

  Serial.begin(9600);

  testclock.Configure();
  Serial.println("clock variable set");
  if (! testclock.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
}

void loop () {
    Serial.println(testclock.getTime());
    delay(3000);
}
