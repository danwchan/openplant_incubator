/**************************************************************************
 This is an example for our Monochrome OLEDs based on SSD1306 drivers

 Pick one up today in the adafruit shop!
 ------> http://www.adafruit.com/category/63_98

 This example is for a 128x32 pixel display using I2C to communicate
 3 pins are required to interface (two I2C and one reset).

 Adafruit invests time and resources providing this open
 source code, please support Adafruit and open-source
 hardware by purchasing products from Adafruit!

 Written by Limor Fried/Ladyada for Adafruit Industries,
 with contributions from the open source community.
 BSD license, check license.txt for more information
 All text above, and the splash screen below must be
 included in any redistribution.
 **************************************************************************/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

class Screen: public Adafruit_SSD1306 {
  private:
    uint8_t ic2Address;
    unsigned long lastUpdated;
    unsigned long readInterval;  // this is in milliseconds unlike the rest of the sensors which are in seconds

  public:

  Screen(uint8_t addr, unsigned long millisec = 200) : Adafruit_SSD1306(128, 64, &Wire, -1) { // base class constructor params: screen_width, screen_height (in pixels), wire library pointer, oled_reset (pin)
    ic2Address = addr;
    readInterval = millisec;
    lastUpdated = millis();
  }

  Configure() {
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC, ic2Address)) {
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }
  }

  void Update() {
    unsigned long currentMillis = millis();
//    Serial.print("Current millis() = "); Serial.println(currentMillis);

    if((currentMillis - lastUpdate) >= (readInterval)) {  // this is in milliseconds unlike the rest of the sensors which are in seconds
      lastUpdate = millis();
      clearDisplay();

      setTextSize(1); // Draw 1X-scale text
      setTextColor(WHITE);
//      cp437(true);

      setCursor(0, 0);      // the status bar month and day 
      print(F("MON XX"));
      setCursor(95, 0);    // the status bar time
      print(F("XX:XX"));

      /*
        This is the grid which lays out the set points
      */
      setCursor(45, 17);    // the set column label
      print(F("Set"));
      setCursor(80, 16);    // the actual column label
      print(F("Actual"));
      setCursor(5, 28);     // the light row label
      println(F("Light"));
      print(F("(lux)"));
      setCursor(5, 48);     // the temp row label
      println(F("Temp")); print(F("(")); write(9); print(F("C)"));

      setCursor(45, 32);    // light set point
      print(F("XXXX"));
      setCursor(80, 32);    // light actual level
      print(F("XXXX"));
      setCursor(45, 52);    // temp set point
      print(F("XX"));
      setCursor(80, 52);    // temp actual level
      print(F("XX"));

      display();            // write to display
    }
  }

  Draw_extra() {
  /*
    This code is for some features which have yet to be implemented
      1) a place to display warnings
      2) arrows to indicate the adjustments the incubator is making
  */
    setTextColor(BLACK, WHITE);// the preferred style to make it pop

    setCursor(0, 8);           // the row for error messages
    print(F("WARNINGS!!!"));

    setCursor(118, 32);        // the light row position for arrows
    write(24);                 // an up arrow
    setCursor(118, 52);        // the temp row position for arrows
    write(25);                 // a down arrow
  }
};

Screen display = Screen(0x3C);  // Address 0x3D for 128x64

void setup() {

  pinMode(9, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(3, OUTPUT);
  
  Serial.begin(9600);

  display.Configure();

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds
  // Clear the buffer
  display.clearDisplay();

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
  delay(2000);
  // display.display() is NOT necessary after every single drawing command,
  // unless that's what you want...rather, you can batch up a bunch of
  // drawing operations and then update the screen all at once by calling
  // display.display(). These examples demonstrate both approaches...

  dashboard();
  delay(5000);
}

void loop() {
  display.Update();
}

/*
  This a function for testing the layout of the screen
*/
void dashboard(void) {
  display.clearDisplay();

  display.setTextSize(1); // Draw 1X-scale text
  display.setTextColor(WHITE);
//  display.cp437(true);
  
  display.setCursor(0, 0);
  display.print(F("MON XX"));
  display.setCursor(95, 0);
  display.print(F("XX:XX"));

  /*
    This is the grid which lays out the set points
  */
  display.setCursor(45, 17);    // the set column label
  display.print(F("Set"));
  display.setCursor(80, 16);    // the actual column label
  display.print(F("Actual"));
  display.setCursor(5, 28);     // the light row label
  display.println(F("Light"));
  display.print(F("(lux)"));
  display.setCursor(5, 48);     // the temp row label
  display.println(F("Temp"));
  display.print(F("("));
  display.write(9);
  display.print(F("C)"));
  
  display.setCursor(45, 32);    // light set point
  display.print(F("XXXX"));
  display.setCursor(80, 32);    // light actual level
  display.print(F("XXXX"));
  display.setCursor(45, 52);    // temp set point
  display.print(F("XX"));
  display.setCursor(80, 52);    // temp actual level
  display.print(F("XX"));

  /*
    This code is for some features which have yet to be implemented
      1) a place to display warnings
      2) arrows to indicate the adjustments the incubator is making
  */
  display.setTextColor(BLACK, WHITE);// the preferred style to make it pop

  display.setCursor(0, 8);           // the row for error messages
  display.print(F("WARNINGS!!!"));

  display.setCursor(118, 32);        // the light row position for arrows
  display.write(24);                 // an up arrow
  display.setCursor(118, 52);        // the temp row position for arrows
  display.write(25);                 // a down arrow

  display.display();      // Show initial text
}
