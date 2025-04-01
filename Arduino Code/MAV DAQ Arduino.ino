#include <Adafruit_GFX.h>    // Core graphics library
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include "Adafruit_RA8875.h"
#include <Adafruit_STMPE610.h>
#include <math.h>

// Library only supports hardware SPI at this time
// Connect SCLK to UNO Digital #13 (Hardware SPI clock)
// Connect MISO to UNO Digital #12 (Hardware SPI MISO)
// Connect MOSI to UNO Digital #11 (Hardware SPI MOSI)

// link to pin out: https://www.electronics-lab.com/wp-content/uploads/2020/05/teensy-pinout-e1590420824325.png

//Pins used for the display
#define RA8875_INT 15
#define RA8875_CS 10
#define RA8875_RESET 14
#define BUFFPIXEL 80

//pins used for file system
int fileNumber = 0;    // Initialize file number
File dataFile;
String targetChar = "d"; // Character to detect

//states for the guage cluster to show -- not implemented yet
#define BOOT 0 //fancy boot up images
#define MENU 1 //unused
#define CLUSTER 2 //sets up the background boxes for the cluster
#define CLUSTERUPDATE 3 //populates and updates the numbers within the boxes
#define DOOM 4 //sets mode to play Doom: Baja Edition
#define SETTINGS 5 //unused

//custom colors for the cluster screen to use -- if new colors are desired use this website to get the correct rgb code --> https://rgbcolorpicker.com/565  Arduino uses RGB565 format
#define RA8875_ROYALPURPLE 0x7a95 //more of a lavender on the screen lol, but technically this is the royal purple code
#define RA8875_GRAY 0x8430
#define RA8875_DARKGRAY 0x4228

//Sensor input pins stated here
#define CVT_THERMISTOR A2 // Thermistor connected to analog pin A2 (aka pin 16)
#define ENG_THERMISTOR A4 // Thermistor connected to analog pin A4 (aka pin 18)
#define FRONT_BRAKE_PRESSURE A8 //pin 22 //was 23/A9
#define REAR_BRAKE_PRESSURE A5 //pin 19 //was 20/A6
#define CVT_RPM_SENSOR 17 
#define ENG_RPM_SENSOR 41

//Button input pins setup here
#define TOP_LEFT_BUTTON 2
#define TOP_MIDDLE_BUTTON 3
#define TOP_RIGHT_BUTTON 4
#define BOTTOM_LEFT_BUTTON 5
#define BOTTOM_MIDDLE_BUTTON 6
#define BOTTOM_RIGHT_BUTTON 7

//Doom: Baja Edition vars and constants
bool isDooming = false;
int doomGuyHealth = 100;
int doomGuyShield = 0;
int doomGuyShotgunShells = 10;
int doomGuyPistolAmmo = 20;

//constants for the pressure sensor equation
const float pressureZero = 146; //analog reading of pressure transducer at 0psi
const float pressureMax = 877; //analog reading of pressure transducer at 1600psi
const float pressuretransducermaxPSI = 1600; //psi value of transducer being used

uint32_t delayTimer = 0; //used for setting the refresh rate of the cluster sensors (set to 100ms, aka, 10 times/sec later)
uint32_t recordTimer = 0; //used for the flashing red dot to indicate data is being recorded (set to 1000ms, aka, 1 time/sec later)
uint32_t redlineFlashTimer = 0; //used to flash the redline warnings (set to 100ms, aka, 10 times/sec later)
uint32_t datalogTimer = 0; //datalog timer (based off of, but seperate from millis())
uint32_t driverTimer = 0; //driver timer (based off of, but seperate from millis())
bool isLogging = false; //whether or not data is being logged or not
bool timerIsRunning = false; //whether or not the dash timer is running
bool recordLightOn = true; //toggle for flashing record light -- purely aesthetic
bool redlineFlasher = true; //toggle for flashing dash warning light(s) -- purely aesthetic
bool driver2 = false; //tells cluster if first driver change has occured
bool driver3 = false; //tells cluster if second driver change has occured

//sets up all variables relating to the engine rpm hall effect sensor
float EngineRpm = 0;
uint32_t MillisSinceLastENGPulse = 0;
String sEngineRpm = String(EngineRpm);

//sets up all variables relating to the cvt rpm hall effect sensor
float CVTOutRpm = 0;
uint32_t MillisSinceLastCVTPulse = 0;
String sCVTOutRpm = String(CVTOutRpm);

//sets up all variables relating to the engine head temperature thermistor
float rawEngTemp = 0;
double engHeadTemp = 0;
String sEngHeadTemp = String(engHeadTemp);

//sets up all variables relating to the cvt belt housing temperature thermistor
float rawCVTTemp = 0;
double CVTBoxTemp = 0;
String sCVTBoxTemp = String(CVTBoxTemp);

//sets up all variables relating to the front brake pressure sensor
int rawFrontBrakePres = 0;
int frontBrakePres = 0;
String sFBPres = String(frontBrakePres);

//sets up all variables relating to the front brake pressure sensor
int rawRearBrakePres = 0;
int rearBrakePres = 0;
String sRBPres = String(rearBrakePres);

//sets cluster state to "boot"
int state = BOOT;

//creates address for the daughter board so it can be called and used by the code -- "tft.method"
Adafruit_RA8875 tft = Adafruit_RA8875(RA8875_CS, RA8875_RESET);

void setup() {
  Serial.begin(9600);

  //sets the hall effect sensor pins to the correct mode
  pinMode(CVT_RPM_SENSOR, INPUT_PULLUP);
  pinMode(ENG_RPM_SENSOR, INPUT_PULLUP);

  //sets the buttons to the correct mode
  pinMode(TOP_LEFT_BUTTON, INPUT_PULLUP);
  pinMode(TOP_MIDDLE_BUTTON, INPUT_PULLUP);
  pinMode(TOP_RIGHT_BUTTON, INPUT_PULLUP);
  pinMode(BOTTOM_LEFT_BUTTON, INPUT_PULLUP);
  pinMode(BOTTOM_MIDDLE_BUTTON, INPUT_PULLUP);
  pinMode(BOTTOM_RIGHT_BUTTON, INPUT_PULLUP);

  // Attach ISRs to the rising edge of each sensor's signal
  attachInterrupt(digitalPinToInterrupt(CVT_RPM_SENSOR), CVTRpmISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENG_RPM_SENSOR), EngRpmISR, RISING);

  // Attach ISRs to each button
  attachInterrupt(digitalPinToInterrupt(TOP_LEFT_BUTTON), TLButtonISR, FALLING); //TLButtonISR
  attachInterrupt(digitalPinToInterrupt(TOP_MIDDLE_BUTTON), TMButtonISR, FALLING); //TMButtonISR
  attachInterrupt(digitalPinToInterrupt(TOP_RIGHT_BUTTON), TRButtonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(BOTTOM_LEFT_BUTTON), BLButtonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(BOTTOM_MIDDLE_BUTTON), BMButtonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(BOTTOM_RIGHT_BUTTON), BRButtonISR, FALLING);

  //checks for, and then activates the SD card if present
  Serial.print(F("Initializing SD card..."));
  if (!SD.begin(BUILTIN_SDCARD))
  {
    Serial.println("initialization failed!");
    //if SD card fails or is not present, it will skip the boot images and jump to guage screen
    state = CLUSTER;
    return;
  }
  else
  {
    Serial.print(F("Success!"));
  }

  //Find the highest numbered file -- file names are formated as such: datalog0.csv -- checks for the most recent (highest number) file
  while (SD.exists(getFilename(fileNumber))) {
    fileNumber++;
  }

  //confirms state is set to boot
  state = BOOT;

  //activates the screen module and sets the correct screen resolution
  if (!tft.begin(RA8875_800x480)) {
    Serial.println("RA8875 Not Found!");
    while (1);
  }

  //more tft initialization, do not have much of an idea about what it is doing, but it works
  tft.displayOn(true);
  tft.GPIOX(true);      // Enable TFT - display enable tied to GPIOX
  tft.PWM1config(true, RA8875_PWM_CLK_DIV1024); // PWM output for backlight (brightness)
  tft.PWM1out(255);

  //Set system to graphics mode -- there are two modes, "graphics" (shapes/images) and "text" (numbers and letters)
  tft.graphicsMode();
  tft.fillScreen(RA8875_BLACK); //clear image and set it to black

  createNewDatalogFile();
}

void loop() {
  if (Serial.available() > 0) { //serial input functions to edit and pull data from SD
    String data = Serial.readString(1);
    
    if (data == targetChar) {
      dumpFile();
      Serial.println("dumped");
    }
    if (data == "c") {
      clearAllLogs();
    }
    if (data == "r") {
      bool working = true;
      String inString;
      while (working)
      {
        Serial.println("Enter Log Number to delete: ");
        while (Serial.available() == 0) {}
        while (Serial.available() > 0) {
          char inChar = Serial.read();
          if (isDigit(inChar)) {
            // convert the incoming byte to a char and add it to the string:
            inString += (char)inChar;
          } else {
            Serial.println("Character was not a number, try again...");
          }
          int fileNum = inString.toInt();
          if (fileNum >= 0 && fileNum <= fileNumber) {
            clearSingleLog(fileNum);
            working = false;
          } else {
            Serial.println("Character was not a valid number, try again...");
          }
        }
      }
    }

    if (data == "g"){ //go
      TLButtonISR();
    }
    if (data == "p") { //pause
      TMButtonISR();
    }
    if (data == "n") { //new file -- stop old one
      TRButtonISR();
    }
  }

  switch(state)
  {
    case BOOT: //Boot screen (should only ran once)
    tft.graphicsMode();
    bmpDraw("BigBaja.bmp", 0, 0);
    delay(1000); //just a delay to keep the image on the screen for longer

    tft.fillScreen(RA8875_BLACK); //clear screen
    bmpDraw("j53.bmp", 0, 0); //j53 stands for just53 -- meaning only the number and not the name for car 53 -- change file name to reflect new car number
    //DO NOT CLEAR SCREEN HERE -- the images are set up to overwrite each other for a cooler animation effect
    bmpDraw("e53.bmp", 0, 0); //e53 stands for everything53 -- meaning number and name for car 53 -- change file name to reflect new car number
    delay(2000); //just a delay to keep the image on the screen for longer

    state = CLUSTER; //sets display into the cluster init mode
    break; //end boot routine

    case CLUSTER: //Gauge Cluster Init
    tft.graphicsMode(); //set to image mode
    tft.fillScreen(RA8875_ROYALPURPLE); //clear screen
    tft.fillRoundRect(0, 0, 385, 225, 25, RA8875_DARKGRAY); //adds rectangles to screen, leading 2 numbers is the pixel coordinate to start at, (0,0) is top left
    tft.fillRoundRect(415, 0, 385, 225, 25, RA8875_DARKGRAY); //the next two number are the (width, height) of said rectangle, extends to the right and downward
    tft.fillRoundRect(0, 255, 385, 225, 25, RA8875_DARKGRAY); //the last number is the radius of the corners for rounding, and the last item is the color code
    tft.fillRoundRect(415, 255, 385, 225, 25, RA8875_DARKGRAY);
    
    tft.textMode(); //set to text mode
    tft.textColor(RA8875_WHITE, RA8875_DARKGRAY); //sets text color in first spot and background (pretty much the highlight) color behind text
    tft.textEnlarge(1); //sets text font size

    tft.textSetCursor(75, 10); //location to start typing
    tft.print("Frnt Brake Pres");
    tft.textSetCursor(75, 185);
    tft.print("Rear Brake Pres");

    tft.textSetCursor(525, 10); //might need to change these values to properly center title in each box
    tft.print("Engine RPM");
    tft.textSetCursor(515, 185);
    tft.print("Rear Whl RPM");

    tft.textSetCursor(85, 265);
    tft.print("CVT Case Temp");
    tft.textSetCursor(85, 435);
    tft.print("Eng Head Temp");

    tft.textSetCursor(485, 265);
    tft.print("Wheel Speed MPH");
    tft.textSetCursor(475, 435);
    tft.print("Elapsed Race Time");

    state = CLUSTERUPDATE; //set to cluster update mode, this sets and updates the actual data to be displayed
    break; //end cluster init routine

    case CLUSTERUPDATE: //Guage Cluster Update Routine
    tft.textMode(); //text mode
    tft.textColor(RA8875_WHITE, RA8875_DARKGRAY); //set text color

    if (delayTimer <= millis()) //If enough time has elapsed, update cluster display
    {
      rawCVTTemp = analogRead(CVT_THERMISTOR); //Read and store raw data from pin
      CVTBoxTemp = Thermister(rawCVTTemp); //Do math on raw value to get temp in degrees F
      
      rawEngTemp = analogRead(ENG_THERMISTOR); //Read and store raw data from pin
      engHeadTemp = Thermister(rawEngTemp); //Do math on raw value to get temp in degrees F

      rawFrontBrakePres = analogRead(FRONT_BRAKE_PRESSURE); //Read and store raw data from pin
      frontBrakePres = Pressure(rawFrontBrakePres); //Do math on raw value to get pressure in psi

      rawRearBrakePres = analogRead(REAR_BRAKE_PRESSURE); //Read and store raw data from pin
      rearBrakePres = Pressure(rawRearBrakePres); //Do math on raw value to get pressure in psi

      if (millis() - MillisSinceLastCVTPulse > 6000) { //Zeros out CVT RPM display after 6 seconds of inactivity
        CVTOutRpm = 0;
      }
      if (millis() - MillisSinceLastENGPulse > 1000) { //Zeros out Eng RPM display after 1 second of inactivity
        EngineRpm = 0;
      }

      //Converts the final numbers to strings and adds on the correct units to the end, the extra space at the end is there to clear off trailing characters when the numbers get too small and move over a decimal place
      sEngineRpm = String(EngineRpm).concat(" RPM ");
      sCVTOutRpm = String(CVTOutRpm).concat(" RPM ");
      sCVTBoxTemp = String(CVTBoxTemp).concat(" F ");
      sEngHeadTemp = String(engHeadTemp).concat(" F ");
      sFBPres = String(frontBrakePres).concat(" Psi  ");
      sRBPres = String(rearBrakePres).concat(" Psi  ");
      
      if (isLogging) {writeDataToLog();} //This needs to be commented out if no data logging is wanted
      delayTimer = millis() + 100; //Resets timer to have program wait another tenth of a second to update again.
    }

    tft.textEnlarge(8); //set font size

    //Set brake pressures (top left) //set to 1650
    if (rearBrakePres > 1800) {tft.textColor(RA8875_WHITE, RA8875_RED);} //sets redline for front brakes to 1650psi
    else {tft.textColor(RA8875_WHITE, RA8875_DARKGRAY);}
    tft.textSetCursor(55, 45);
    tft.print(sFBPres);
    if (frontBrakePres < 100) {tft.print(" "); }
    tft.textColor(RA8875_WHITE, RA8875_DARKGRAY);
    if (frontBrakePres > 1800) {tft.textColor(RA8875_WHITE, RA8875_RED);} //sets redline for front brakes to 1650psi
    else {tft.textColor(RA8875_WHITE, RA8875_DARKGRAY);}
    tft.textSetCursor(55, 115);
    tft.print(sRBPres);
    if (rearBrakePres < 100) {tft.print(" "); }
    tft.textColor(RA8875_WHITE, RA8875_DARKGRAY);

    //Set Engine/CVT RPM (Top right)
    if (EngineRpm > 3700) {tft.textColor(RA8875_WHITE, RA8875_RED);} //sets redline for engine to 3700rpm
    else {tft.textColor(RA8875_WHITE, RA8875_DARKGRAY);}
    tft.textSetCursor(475, 45);
    tft.print(sEngineRpm);
    tft.textColor(RA8875_WHITE, RA8875_DARKGRAY); //no wheel speed redline
    tft.textSetCursor(475, 115);
    tft.print(sCVTOutRpm);


    //Set temp sensors (Bottom Left)
    if (CVTBoxTemp > 160.0) {tft.textColor(RA8875_WHITE, RA8875_RED);} //sets warning (solid red) for CVT box temp to 160F
    else if (CVTBoxTemp > 180.0) { //sets redline (flashing red) for CVT box temp to 180F
      if (redlineFlasher) {
        tft.textColor(RA8875_WHITE, RA8875_RED);
      }
      else {
        tft.textColor(RA8875_WHITE, RA8875_DARKGRAY);
      }
    }
    else {tft.textColor(RA8875_WHITE, RA8875_DARKGRAY);}
    tft.textSetCursor(55, 300);
    tft.print(sCVTBoxTemp);
    tft.textColor(RA8875_WHITE, RA8875_DARKGRAY); //no temp warning for eng head temp
    tft.textSetCursor(55, 370);
    tft.print(sEngHeadTemp);
    
    //Set MPH/Timer (Bottom Right)
    if (convertRpmToMph(CVTOutRpm) > 30) {tft.textColor(RA8875_WHITE, RA8875_RED);} //sets redline for vehicle speed to 30mph
    else {tft.textColor(RA8875_WHITE, RA8875_DARKGRAY);}
    tft.textSetCursor(445, 300); //was 495, 340
    tft.print(String(convertRpmToMph(CVTOutRpm)).concat(" MPH "));
    if (convertRpmToMph(CVTOutRpm) < 10) { tft.print(" "); }
    tft.textColor(RA8875_WHITE, RA8875_DARKGRAY);
    driverTimer = millis() - datalogTimer;
    // if between 70min and 85min as well as no confirmed driver change | or | if between 150min and 165min as well as no confirmed driver change
    if ((((driverTimer > 4200000) & (driverTimer < 5100000)) & !driver2) | (((driverTimer > 9000000) & (driverTimer < 9900000)) & !driver3)) { //sets warning (solid orange) for 10 MINUTES TILL DRIVER CHANGE
      tft.textColor(RA8875_WHITE, RA8875_ROYALPURPLE);
      }
    // if between 80min and 85min as well as no confirmed driver change | or | if between 160min and 165min as well as no confirmed driver change
    else if ((((driverTimer > 4800000) & (driverTimer < 5100000)) & !driver2) | (((driverTimer > 9600000) & (driverTimer < 9900000)) & !driver3)) { //sets redline (flashing orange) for DRIVE CHANGE NOW
      if (recordLightOn) {
        tft.textColor(RA8875_WHITE, RA8875_ROYALPURPLE);
      }
      else {
        tft.textColor(RA8875_WHITE, RA8875_DARKGRAY);
      }
    }
    else {tft.textColor(RA8875_WHITE, RA8875_DARKGRAY);}
    tft.textSetCursor(445, 370);
    if (timerIsRunning) {tft.print(getTimeStamp());}
    else if (datalogTimer > 0) {}
    else {tft.print("00:00:00.0");}

    if (isLogging) {
      tft.graphicsMode(); //recordTimer <= millis()

      if (recordLightOn) {
        tft.fillCircle(775, 25, 15, RA8875_RED);
      }
      else {
        tft.fillCircle(775, 25, 15, RA8875_BLACK);
      }
    }
    else {
      tft.fillCircle(775, 25, 15, RA8875_BLACK);
    }

    if (recordTimer <= millis()) {
      if (recordLightOn) {
        recordLightOn = false;
      }
      else {
        recordLightOn = true;
      }
      recordTimer = millis() + 1000;
    }

    if (redlineFlashTimer <= millis()) {
      if (redlineFlasher) {
        redlineFlasher = false;
      }
      else {
        redlineFlasher = true;
      }
      redlineFlashTimer = millis() + 100;
    }

    break;

    case DOOM: 
    delay(2000);
    isDooming = false;
    state = CLUSTER;
    break;
  }

}

double Thermister(float RawADC) {  //Function to perform the fancy math of the Steinhart-Hart equation
  double Temp;
  Temp = log(((10240000 / RawADC) - 10000));
  Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp)) * Temp);
  Temp = Temp - 273.15;              // Convert Kelvin to Celsius
  Temp = (Temp * 9.0) / 5.0 + 32.0;  // Celsius to Fahrenheit - comment out this line if you need Celsius
  return Temp;
}

//does the math to convert the raw analog signal (1-1023) to a usuable temp number -- calibrated 5/08/24 -- and was within .5 degrees F from ambient to 202
int Pressure(int RawADC) {
  //Serial.println(analogRead(FRONT_BRAKE_PRESSURE));
  // ((RawADC-pressureZero)*pressuretransducermaxPSI)/(pressureMax-pressureZero); //conversion equation to convert analog reading to psi
  float complexVal = ((float(RawADC) - pressureZero)/(pressureMax - pressureZero)) * pressuretransducermaxPSI;
  int simpleVal = (int)complexVal;
  if (simpleVal < 0) {simpleVal = 0;}
  //Serial.println("((" + String(RawADC) + " - " + String(pressureZero) + ")/(" + String(pressureMax) + " - " + String(pressureZero) + ")) * " + String(pressuretransducermaxPSI) + " = " + String(complexVal) + " or " + String(simpleVal));
  return simpleVal;
}

//used to properly draw a .bmp (Bitmap) image
void bmpDraw(const char *filename, int x, int y) {
  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel in buffer (R+G+B per pixel)
  uint16_t lcdbuffer[BUFFPIXEL];  // pixel out buffer (16-bit per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col, xpos, ypos;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();
  uint8_t  lcdidx = 0;

  if((x >= tft.width()) || (y >= tft.height())) return;

  Serial.println();
  Serial.print(F("Loading image '"));
  Serial.print(filename);
  Serial.println('\'');

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == false) {
    Serial.println(F("File not found"));
    return;
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) { // BMP signature
    Serial.println(F("File size: "));
    Serial.println(read32(bmpFile));
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    Serial.print(F("Image Offset: "));
    Serial.println(bmpImageoffset, DEC);

    // Read DIB header
    Serial.print(F("Header size: "));
    Serial.println(read32(bmpFile));
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);

    if(read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      Serial.print(F("Bit Depth: "));
      Serial.println(bmpDepth);
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed
        goodBmp = true; // Supported BMP format -- proceed!
        Serial.print(F("Image size: "));
        Serial.print(bmpWidth);
        Serial.print('x');
        Serial.println(bmpHeight);

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if((x+w-1) >= tft.width())  w = tft.width()  - x;
        if((y+h-1) >= tft.height()) h = tft.height() - y;

        // Set TFT address window to clipped image bounds
        ypos = y;
        for (row=0; row<h; row++) { // For each scanline...
          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;

          if (bmpFile.position() != pos) { // Need seek?
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }
          xpos = x;
          for (col=0; col<w; col++) { // For each column...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
              // Push LCD buffer to the display first
              if(lcdidx > 0) {
                tft.drawPixels(lcdbuffer, lcdidx, xpos, ypos);
                xpos += lcdidx;
                lcdidx = 0;
              }

              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
            }

            // Convert pixel from BMP to TFT format
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            lcdbuffer[lcdidx++] = color565(r,g,b);
            if (lcdidx >= sizeof(lcdbuffer) || (xpos - x + lcdidx) >= w) {
              tft.drawPixels(lcdbuffer, lcdidx, xpos, ypos);
              lcdidx = 0;
              xpos += lcdidx;
            }
          } // end pixel
            ypos++;
        } // end scanline

        // Write any remaining data to LCD
        if(lcdidx > 0) {
          tft.drawPixels(lcdbuffer, lcdidx, xpos, ypos);
          xpos += lcdidx;
        }

        Serial.print(F("Loaded in "));
        Serial.print(millis() - startTime);
        Serial.println(" ms");

      } // end goodBmp
    }
  }

  bmpFile.close();
  if(!goodBmp) Serial.println(F("BMP format not recognized."));

}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

//helper method for bmpDraw
uint16_t read16(File f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

//helper method for bmpDraw
uint32_t read32(File f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}

//helper method for bmpDraw
uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

//helper method for bmpDraw
byte decToBcd(byte val){
  // Convert normal decimal numbers to binary coded decimal
  return ( (val/10*16) + (val%10) );
}

//creates a properly formatted file name
char* getFilename(int number) {
  String helper = "datalog" + String(number) + ".csv";
  return strdup(helper.c_str()); //dont even ask, I have ZERO idea, this somehow makes the SD.Exists method happy, so by extension, I am happy
}

//Coeffecient to convert RPMs from wheel speed sensor to MPH -- this is a car specific number 
double convertRpmToMph(int rpm) {
  double mph = rpm * 0.0654498469;
  if (mph < 1) { CVTOutRpm = 0; return 0; }
  else { return rpm * 0.0654498469; }
}

//ISR that calculates the time between pulses from the wheel speed (CVT) sensor
void CVTRpmISR() {
  uint32_t currentTime = millis(); 
  // Calculate RPM for each sensor
  float daMafs = (60000.0 / float((currentTime - MillisSinceLastCVTPulse))) / 2.0;
  if (daMafs < 500) { CVTOutRpm = daMafs; }
  //CVTOutRpm = (60000.0 / float((currentTime - MillisSinceLastCVTPulse))) / 2.0; //divide by 2 since there are two pulses per rotation
  MillisSinceLastCVTPulse = currentTime;
  Serial.println("CVT Rpm: " + String(CVTOutRpm));
}

//ISR that calculates the time between pulses from the engine rpm sensor
void EngRpmISR() {
  uint32_t currentTime = millis();
  // Calculate RPM for each sensor
  float daMafs = (60000.0 / float((currentTime - MillisSinceLastENGPulse)));
  if (daMafs < 5000) { EngineRpm = daMafs; }
  //EngineRpm = 60000.0 / float(currentTime - MillisSinceLastENGPulse); //NOT divided by two since there is only one pulse per rotation (despite there being two magnets on Banshee, only one registers...)
  MillisSinceLastENGPulse = currentTime;
  Serial.println("Eng Rpm: " + String(EngineRpm));
}

//This method takes the current data being displayed on the dash and adds it to the log file, the log file is updated each time the screen updates, which at the time of writing is 10 times/sec
void writeDataToLog() {
  dataFile = SD.open(getFilename(fileNumber), FILE_WRITE);
  if (dataFile) {
    //populates each column in the csv file
    dataFile.println(getTimeStamp() + "," + String(CVTBoxTemp) + "," + String(engHeadTemp) + "," + String(EngineRpm) + "," + String(CVTOutRpm) + "," + String(frontBrakePres) + "," + String(rearBrakePres) + "," + String(convertRpmToMph(CVTOutRpm)));
    dataFile.close();
  }
}

//Creates a timestamp 
String getTimeStamp() {
  unsigned long currentTime = millis() - datalogTimer; // Get elapsed time in milliseconds
  unsigned long seconds = currentTime / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  unsigned long tenthsOfSecond = (currentTime % 1000) / 100; // Tenths of a second

  // Format the timestamp
  char timestamp[12]; // Adjust the size as needed
  snprintf(timestamp, sizeof(timestamp), "%02lu:%02lu:%02lu.%lu", hours, minutes % 60, seconds % 60, tenthsOfSecond);

  return String(timestamp);
}

//Reads and prints all files to the serial port, kind of inconvinent for pulling those csv files since it prints any and all of them, but you can pull them over usb if you use this method
void dumpFile() {
  // Your code to read and print a file from the SD card
  int dumpedFileNum = 0;
  while (dumpedFileNum < fileNumber)
  {
    File dumpFile = SD.open(getFilename(dumpedFileNum));
    if (dumpFile) {
      Serial.write(dumpFile.name());
      Serial.println("");
      while (dumpFile.available()) {
        Serial.write(dumpFile.read());
      }
      Serial.println("");
      dumpFile.close();
    } else {
      Serial.println("Error opening datalogXX.txt");
    }
    dumpedFileNum++;
  }
}

//Deletes the entire log file directory
void clearAllLogs() {
  int dumpedFileNum = 0;
  while (dumpedFileNum < fileNumber)
  {
    SD.remove(getFilename(dumpedFileNum));
    dumpedFileNum++;
  }
  fileNumber = 0;
}

//Deletes a single log file
void clearSingleLog(int logNum) {
  int dumpedFileNum = logNum;
  SD.remove(getFilename(dumpedFileNum));
  dumpedFileNum++;
  while (dumpedFileNum < fileNumber)
  {
    SD.rename(getFilename(dumpedFileNum), getFilename(dumpedFileNum - 1));
    dumpedFileNum++;
  }
}

//Effectively ends previous file (if present) and creates a new one 
void createNewDatalogFile() {
  //Once the newest file is identified, it will create a new file that is one number higher: datalog1.csv, datalog2.csv, etc.
  char* newFilename = getFilename(fileNumber);
  dataFile = SD.open(newFilename, FILE_WRITE);
  if (dataFile) {
    //creates headers for each column in the csv file
    dataFile.println("Timestamp, BeltHousingTemp, CylHeadTemp, EngRpm, CVTRpm, FrntBrakePres, RearBrakePres, WheelSpeed");
    dataFile.close();
    Serial.println("Created file: " + String(newFilename));
  } else {
    Serial.println("Error creating file: " + String(newFilename));
  }

  fileNumber++;
}

//Start Dataloging
void TLButtonISR() {
  if (!isDooming) {
    if (!timerIsRunning) {
      datalogTimer = millis();
      driverTimer = datalogTimer;
    }
    isLogging = true;
    timerIsRunning = true;
  }
}

//Pause Datalog/Timer (keep same file)
void TMButtonISR() {
  if (!isDooming) {
    //isLogging = false;
  }
}

//End Datalog/Timer (new file)
void TRButtonISR() {
  if (!isDooming) {
    //isLogging = false;
    //timerIsRunning = false;
    //createNewDatalogFile();
  }
}

// Not Sure yet lol
void BLButtonISR() {
  if (!isDooming) {

  }
}

//Cancel driver change alarm
void BMButtonISR() {
  if (!isDooming) {

  }
}

//Settings? Doom Game?
void BRButtonISR() {
  if (!isDooming) { 
    //isDooming = true; 
    //bmpDraw("DoomMenuScreen.bmp", 0, 0);
    //bmpDraw("DoomProject.bmp", 0, 0);
    //bmpDraw("DoomProject2.bmp", 0, 0);
    //state = DOOM;
  }
}


















//end