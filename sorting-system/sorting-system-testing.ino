// 
//  MSE 2202 Lab 2-Exercise 4
// 
//  Uses a potentiometer to set the position of an RC servo motor
//
//  Language: Arduino (C++)
//  Target:   ESP32-S3
//  Author:   Michael Naish
//  Date:     2024 01 21 
//  
//  To program and use ESP32-S3
//   
//  Tools->Board->Boards Manager...
//    esp32 by Espressif v2.0.11
//  Tools->:
//    Board: "Adafruit Feather ESP32-S3 No PSRAM"
//    Upload Speed: "921600"
//    USB CDC On Boot: "Enabled"
//    USB Firmware MSC on Boot: "Disabled"
//    USB DFU On Bot: "Disabled"
//    Upload Mode:"UART0/Hardware CDC"
//    SPU Frequency: "240MHz (WiFi)"
//    Flash Mode: "QIO 80MHz"
//    Flash SIze: "4MB (32Mb)"
//    Partition Scheme: "Default 4MB with spiffs (1.2MB app/1.5MB SPIFFS)"
//    Core Debug Level: "Verbose"
//    PSRAM: 'Disabled"
//    Arduino Runs On: "Core 1"
//    Events Run On: "Core 1"
//
//  To program, press and hold the reset button then press and hold program button, release the reset 
//  button then release the program button 
//

//#define OUTPUT_ON                                    // uncomment to turn on output debugging information

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Function declarations
void doHeartbeat();
long degreesToDutyCycle(int deg);
void sweep(int index);
void pause(int msDelay);

// push button on GPIO0
const int pushButtonP0 = 0;

// Constants
const int cHeartbeatInterval = 75;                 // heartbeat update interval, in milliseconds
const int cSmartLED          = 21;                 // when DIP switch S1-4 is on, SMART LED is connected to GPIO21
const int cSmartLEDCount     = 1;                  // number of Smart LEDs in use
//const int cPotPin            = 1;                  // when DIP switch S1-3 is on, pot (R1) is connected to GPIO1 (ADC1-0)
const int cServoPin          = 41;                 // GPIO pin for servo motor
const int leftServoChannel      = 4;                  // PWM channel used for the RC servo motor
const int rightServoChannel     = 5;
const int cServoChannel = 6;
const int leftBucketServo = 42;
const int rightBucketServo = 44;

// Variables
boolean heartbeatState       = true;               // state of heartbeat LED
unsigned long lastHeartbeat  = 0;                  // time of last heartbeat state change
unsigned long curMillis      = 0;                  // current time, in milliseconds
unsigned long prevMillis     = 0;                  // start time for delay cycle, in milliseconds
unsigned long previousMicros;                      // last microsecond count
unsigned long currentMicros;                       // current microsecond count
unsigned long timer30ms = 0;
bool timeUp30ms = false;
//int potVal;                                        // input value from the potentiometer
int servoPos;                                      // desired servo angle
bool hasDumped = false;
bool hasSwept = false;
bool isSorting = false;

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

void setup() {
#ifdef OUTPUT_ON
    Serial.begin(115200);                            // Standard baud rate for ESP32 serial monitor
#endif
  // Set up SmartLED
  SmartLEDs.begin();                               // initialize smart LEDs object
  SmartLEDs.clear();                               // clear pixel
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(0,0,0)); // set pixel colours to black (off)
  SmartLEDs.setBrightness(0);                      // set brightness [0-255]
  SmartLEDs.show();                                // update LED
  
  // Set up servo
  pinMode(cServoPin, OUTPUT);                      // configure servo GPIO for output
  ledcSetup(cServoChannel, 50, 14);                // setup for channel for 50 Hz, 14-bit resolution
  ledcAttachPin(cServoPin, cServoChannel);         // assign servo pin to servo channel

  pinMode(leftBucketServo, OUTPUT);                      // configure servo GPIO for output
  ledcSetup(leftServoChannel, 50, 14);                // setup for channel for 50 Hz, 14-bit resolution
  ledcAttachPin(leftBucketServo, leftServoChannel);         // assign servo pin to servo channel

  pinMode(rightBucketServo, OUTPUT);                      // configure servo GPIO for output
  ledcSetup(rightServoChannel, 50, 14);                // setup for channel for 50 Hz, 14-bit resolution
  ledcAttachPin(rightBucketServo, rightServoChannel);         // assign servo pin to servo channel

  // Set up push button
  pinMode(pushButtonP0, INPUT_PULLUP); // configure GPIO for push button with internal pullup resistor
}

void loop() {

  if (!hasSwept) { // sweeps if currently open/has not swept
    sweep(1);
  }
  else { // closes the 
    if (!hasDumped) {
      dump(1);
      pause(1000);
      dump(2);
    }
      sweep(2);
  }




  doHeartbeat();                                   // update heartbeat LED
}

// update heartbeat LED
void doHeartbeat() {
  curMillis = millis();                            // get the current time in milliseconds
  // check to see if elapsed time matches the heartbeat interval
  if ((curMillis - lastHeartbeat) > cHeartbeatInterval) {
    lastHeartbeat = curMillis;                     // update the heartbeat time for the next update
    LEDBrightnessIndex++;                          // shift to the next brightness level
    if (LEDBrightnessIndex > sizeof(LEDBrightnessLevels)) { // if all defined levels have been used
      LEDBrightnessIndex = 0;                      // reset to starting brightness
    }
    SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]); // set brightness of heartbeat LED
    SmartLEDs.setPixelColor(0, SmartLEDs.Color(0, 250, 0)); // set pixel colours to green
    SmartLEDs.show();                              // update LED
  }
}

// Converts servo position in degrees into the required duty cycle for an RC servo motor control signal 
// assuming 14-bit resolution (i.e., value represented as fraction of 16383). 
// Note that the constants for minimum and maximum duty cycle may need to be adjusted for a specific motor
long degreesToDutyCycle(int deg) {
  const long cMinDutyCycle = 400;                     // duty cycle for 0 degrees
  const long cMaxDutyCycle = 2100;                    // duty cycle for 180 degrees

  long dutyCycle = map(deg, 0, 180, cMinDutyCycle, cMaxDutyCycle);  // convert to duty cycle

#ifdef OUTPUT_ON
  float percent = dutyCycle * 0.0061039;              // (dutyCycle / 16383) * 100
  Serial.printf("Degrees %d, Duty Cycle Val: %ld = %f%%\n", servoPos, dutyCycle, percent);
#endif

  return dutyCycle;
}
  // used to push rocks into the bucket (and keep hold them) before dumping into the funnel
  void sweep(int index) {
    int i = 0;
    switch (index) {
      case 1: // closes the sweeper
        for (i = 0; i <= 100; i+=1) {
          ledcWrite(cServoChannel, degreesToDutyCycle(i));
          pause(5);
        }
        hasSwept = true;
      break;
      case 2: // opens the sweeper
        for (i = 100; i >= 0; i-=1) {
          ledcWrite(cServoChannel, degreesToDutyCycle(i));
          pause(5);
        }
      break;
    }
  }

  void dump(int index) {
    int l, r;
    switch (index) {
      case 1: // bucket goes up
        for (l = 110, r = 0; l>=0; l-=2, r+=2) { 
            ledcWrite(leftServoChannel, degreesToDutyCycle(l)); // set the desired servo position
            ledcWrite(rightServoChannel, degreesToDutyCycle(r));
            pause(30);
          }
      break;
      case 2: // bucket goes down
        for (l = 0, r = 110; l<=110; l+=2, r-=2) { 
            ledcWrite(leftServoChannel, degreesToDutyCycle(l)); // set the desired servo position
            ledcWrite(rightServoChannel, degreesToDutyCycle(r));
            pause(30);
          }
          hasDumped = true;
      break;
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