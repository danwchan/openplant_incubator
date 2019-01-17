  /*
  SD card test

  This example shows how use the utility libraries on which the'
  SD library is based in order to get info about your SD card.
  Very useful for testing a card when you're not sure whether its working or not.

  The circuit:
    SD card attached to SPI bus as follows:
 ** MOSI - pin 11 on Arduino Uno/Duemilanove/Diecimila
 ** MISO - pin 12 on Arduino Uno/Duemilanove/Diecimila
 ** CLK - pin 13 on Arduino Uno/Duemilanove/Diecimila
 ** CS - depends on your SD card shield or module.
 		Pin 4 used here for consistency with other Arduino examples


  created  28 Mar 2011
  by Limor Fried
  modified 9 Apr 2012
  by Tom Igoe
*/
// include the SD library:
#include <SPI.h>
#include <SD.h>

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;
//
byte chipPin = 10;

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

  bool Update(String fileName, String timeID, float temp, float humidity, uint32_t lux, double pidOutput) {
    unsigned long currentMillis = millis();

    if((currentMillis - lastUpdate) >= (readInterval * 1000)) {
      lastUpdate = millis();
      fileName += ".log";
      File dataFile = open(fileName, FILE_WRITE);
      //Serial.println(fileName);
      
      if (dataFile) {                                                // if the file is available, write to it:
        dataFile.print(timeID); dataFile.print(", ");
        dataFile.print(temp); dataFile.print(", ");
        dataFile.print(humidity); dataFile.print(", ");
        dataFile.print(lux); dataFile.print(", ");
        dataFile.println(pidOutput);
        dataFile.close();
        return true;
      }
      else {                                                         // if the file isn't open, pop up an error:
        return false;
      }
    }
    return false;
  }
};

Sdlogger testlogger = Sdlogger(10, 5);

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.print("\nInitializing SD card...");

  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
  
  if (!card.init(SPI_HALF_SPEED, chipPin)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    while (1);
  } else {
    Serial.println("Wiring is correct and a card is present.");
  }

  // print the type of card
  Serial.println();
  Serial.print("Card type:         ");
  switch (card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println("SD1");
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println("SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("Unknown");
  }

  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    while (1);
  }

  Serial.print("Clusters:          ");
  Serial.println(volume.clusterCount());
  Serial.print("Blocks x Cluster:  ");
  Serial.println(volume.blocksPerCluster());

  Serial.print("Total Blocks:      ");
  Serial.println(volume.blocksPerCluster() * volume.clusterCount());
  Serial.println();

  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print("Volume type is:    FAT");
  Serial.println(volume.fatType(), DEC);

  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize /= 2;                           // SD card blocks are always 512 bytes (2 blocks are 1KB)
  Serial.print("Volume size (Kb):  ");
  Serial.println(volumesize);
  Serial.print("Volume size (Mb):  ");
  volumesize /= 1024;
  Serial.println(volumesize);
  Serial.print("Volume size (Gb):  ");
  Serial.println((float)volumesize / 1024.0);

  Serial.println("\nFiles found on the card (name, date and size in bytes): ");
  root.openRoot(volume);

  // list all files in the card with date and size
  root.ls(LS_R | LS_DATE | LS_SIZE);
  

  //configure the SD logger class
  testlogger.Configure();
}

void loop(void) {
  if (testlogger.Update("file", "time123", 666.00, 0.01, 1234.56, 999)) {
    Serial.println("file written");
  }
}
