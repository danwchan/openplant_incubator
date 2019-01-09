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
    String fromClock;
    uint32_t *fromLightsensor;
    float *fromTempsensor;
    unsigned int *setLux;

  public:
    unsigned long lastUpdated; 

  Screen() : Adafruit_SSD1306(128, 64, &Wire, -1) { // screen_width, screen_height (in pixels), wire library pointer, oled_reset (pin)
   }
};

Screen display = Screen();

void setup() {

  pinMode(9, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(3, OUTPUT);
  
  Serial.begin(9600);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

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
//  testdrawstyles();
//  delay(5000);
//  testdrawchar();
}

void loop() {
  dashboard();
  delay(5000);
}


void testdrawstyles(void) {
  display.clearDisplay();

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("Hello, world!"));

  display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
  display.println(3.141592);

  display.setTextSize(2);             // Draw 2X-scale text
  display.setTextColor(WHITE);
  display.print(F("0x")); display.println(0xDEADBEEF, HEX);

  display.display();
  delay(2000);
}

void dashboard(void) {
  display.clearDisplay();

  display.setTextSize(1); // Draw 1X-scale text
  display.setTextColor(WHITE);
//  display.cp437(true);
  
  display.setCursor(0, 0);
  display.print(F("MON XX"));
  display.setCursor(95, 0);
  display.print(F("XX:XX"));

  display.setCursor(45, 17);
  display.print(F("Set"));
  display.setCursor(80, 16);
  display.print(F("Actual"));
  display.setCursor(5, 28);
  display.println(F("Light"));
  display.print(F("(lux)"));
  display.setCursor(5, 48);
  display.println(F("Temp"));
  display.print(F("(")); display.write(9); display.print(F("C)"));
  
  display.setCursor(45, 32);
  display.print(F("XXXX"));
  display.setCursor(80, 32);
  display.print(F("XXXX"));
  display.setCursor(45, 52);
  display.print(F("XX"));
  display.setCursor(80, 52);
  display.print(F("XX"));

  display.setCursor(0, 8);  
  display.setTextColor(BLACK, WHITE); 
  display.print(F("WARNINGS!!!"));

  display.setCursor(118, 32);   // the light row
  display.write(24);            // an up arrow
  display.setCursor(118, 52);   // the temp row
  display.write(25);            // a down arrow

  display.display();      // Show initial text
}
