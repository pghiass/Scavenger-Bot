// 
// Sorting System Code
//
// Uses a stepper motor and a TCS34725 colour sensor
// - Base code for stepper motor and colour sensor taken from 2202 project OWL page 
//
//  Language: Arduino (C++)
//  Target:   ESP32-S3
//

#define PRINT_COLOUR                                  // uncomment to turn on output of colour sensor data

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_TCS34725.h>
#include <Stepper.h>

// Function declarations
void doHeartbeat();
void pause(int msDelay);

// Constants
const int cHeartbeatInterval = 75;                    // heartbeat update interval, in milliseconds
const int cSmartLED          = 21;                    // when DIP switch S1-4 is on, SMART LED is connected to GPIO21
const int cSmartLEDCount     = 1;                     // number of Smart LEDs in use
const int cSDA               = 47;                    // GPIO pin for I2C data
const int cSCL               = 48;                    // GPIO pin for I2C clock
const int cTCSLED            = 14;                    // GPIO pin for LED on TCS34725
const int cLEDSwitch         = 46;                    // DIP switch S1-2 controls LED on TCS32725    
const int stepsInNinetyDegrees = 1035;                // the number of steps on the stepper motor to make it turn 90 degrees
const long interval = 3000;

// Variables
boolean heartbeatState       = true;                  // state of heartbeat LED
unsigned long lastHeartbeat  = 0;                     // time of last heartbeat state change
unsigned long curMillis      = 0;                     // current time, in milliseconds
unsigned long prevMillis     = 0;                     // start time for delay cycle, in milliseconds
unsigned long previousMicros;                      // last microsecond count
unsigned long currentMicros;                       // current microsecond count
// boolean wantedObject = false; 
int stepDir = 1;                                      // direction of stepper motor
unsigned long lastStepMove = 0; // Time of last servo position update

Stepper stepper (stepsInNinetyDegrees, 39, 40);       // constructor for stepper object on pins 39 and 40

// Declare SK6812 SMART LED object
//   Argument 1 = Number of LEDs (pixels) in use
//   Argument 2 = ESP32 pin number 
//   Argument 3 = Pixel type flags, add together as needed:
//     NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//     NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//     NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//     NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//     NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel SmartLEDs(cSmartLEDCount, cSmartLED, NEO_RGB + NEO_KHZ800);

// Smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0; 
unsigned char LEDBrightnessLevels[] = {0, 0, 0, 5, 15, 30, 45, 60, 75, 90, 105, 120, 135, 
                                       150, 135, 120, 105, 90, 75, 60, 45, 30, 15, 5, 0};

// TCS34725 colour sensor with 2.4 ms integration time and gain of 4
// see https://github.com/adafruit/Adafruit_TCS34725/blob/master/Adafruit_TCS34725.h for all possible values
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;                                     // TCS34725 flag: 1 = connected; 0 = not found

void setup() {
  stepper.setSpeed(80);                               // set speed of stepper motor to 80 rpm
  
  Serial.begin(115200);                               // Standard baud rate for ESP32 serial monitor

  // Set up SmartLED
  SmartLEDs.begin();                                  // initialize smart LEDs object
  SmartLEDs.clear();                                  // clear pixel
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(0,0,0)); // set pixel colours to black (off)
  SmartLEDs.setBrightness(0);                         // set brightness [0-255]
  SmartLEDs.show();                                   // update LED

  Wire.setPins(cSDA, cSCL);                           // set I2C pins for TCS34725
  pinMode(cTCSLED, OUTPUT);                           // configure GPIO to control LED on TCS34725
  pinMode(cLEDSwitch, INPUT_PULLUP);                  // configure GPIO to set state of TCS34725 LED 

  // Connect to TCS34725 colour sensor
  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
  }
  else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }
}
bool temp = true;
  uint16_t r, g, b, c, r1, g1, b1;                                // RGBC values from TCS34725
void loop() {
  unsigned long currentMillis = millis();
  
  digitalWrite(cTCSLED, !digitalRead(cLEDSwitch));    // turn on onboard LED if switch state is low (on position)
  if (tcsFlag) {                                      // if colour sensor initialized
    // tcs.getRawData(&r, &g, &b, &c);                   // get raw RGBC values
    if (true) {
      // if (temp) {
      //   // stepDir = -1;
      //   // stepper.step(stepDir * stepsInNinetyDegrees/2); // rotate 45 degrees
      //   temp = false;
      //   r1 = r;
      //   g1 = g;
      //   b1 = b;
      // }
      // tcs.getRawData(&r, &g, &b, &c); // second rock
      // if (!temp) {
      //   r1 = r;
      //   g1 = g;
      //   b1 = b;
      // }
      // if (temp) {
      //   Serial.printf("Not green ");
      //   Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
      //   stepDir = -1;
      //   stepper.step(stepDir * stepsInNinetyDegrees);
      //   temp = false;
      // }
      // else if ((g1>r1&&g1>b1)&&(g1>1.15*b1)&&(g1>1.15*r1)){ // first rock condition
     
      if (isGreen(r,g,b)) {
        Serial.printf("Green ");
        Serial.printf("R: %d, G: %d, B: %d, C: %d\n", r, g, b, c);
        delay(3000);
        tcs.getRawData(&r, &g, &b, &c);
        stepDir = 1;                                                  // motor turns towards green stone path
        stepper.step(stepDir * stepsInNinetyDegrees);
      }
      else {
        Serial.printf("Not green ");
        Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
        delay(3000);
        tcs.getRawData(&r, &g, &b, &c);
        stepDir = -1;                                                 // motor turns towards other stones path
        stepper.step(stepDir * stepsInNinetyDegrees);
      }
      
    }
    else {
      Serial.printf("No Stone R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
    }
  }
  pause(500);
    
  
  doHeartbeat();                                      // update heartbeat LED
}

// update heartbeat LED
void doHeartbeat() {
  curMillis = millis();                               // get the current time in milliseconds
  // check to see if elapsed time matches the heartbeat interval
  if ((curMillis - lastHeartbeat) > cHeartbeatInterval) {
    lastHeartbeat = curMillis;                        // update the heartbeat time for the next update
    LEDBrightnessIndex++;                             // shift to the next brightness level
    if (LEDBrightnessIndex > sizeof(LEDBrightnessLevels)) { // if all defined levels have been used
      LEDBrightnessIndex = 0;                         // reset to starting brightness
    }
    SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]); // set brightness of heartbeat LED
    SmartLEDs.setPixelColor(0, SmartLEDs.Color(0, 250, 0)); // set pixel colours to green
    SmartLEDs.show();                                 // update LED
  }
}

  // pauses the program by 'msDelay' milliseconds
  void pause(int msDelay) {
    bool timeUp = false;
    unsigned long timerVal = 0;
    while(!timeUp) {
      currentMicros = micros();
      if ((currentMicros-previousMicros)>=1000) {
        previousMicros = currentMicros;
        timerVal += 1;
        if (timerVal >= msDelay) {
          timeUp = true;
        }
      }
    }
  }

  // returns true if the rgb values represent a stone
  // returns false if the rgb values represent absence of a stone
   bool hasStone(int r, int g, int b) {
      if ((44>=r&&r>=36)&&(45>=g&&g>=37)&&(43>=b&&b>=36)) { // NOTE: rgb values may be lowered when the slope increases
        return false;
      }
      else if ((r+g+b)/3>=120) { // filter random spikes
        return false;
      }
      else {
        return true;
      }
    }

  bool isGreen(int r, int g, int b) {
    if ((r+4)<=g&&(b+5)<=g) { // NOTE: rgb values may be lowered when the slope increases
      return true;
    }
    else {
      return false;
    }
  }