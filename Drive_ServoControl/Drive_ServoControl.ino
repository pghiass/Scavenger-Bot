/*

Drive, bulldozer arms and backdoor control

- Final version used for showcase
- IR detection
- Based on MSE 2202 MSEBot base code for Lab 4
 Language: Arduino

 */


//  To program and use ESP32-S3
//
//  Tools->:
//  Board: "Adafruit Feather ESP32-S3 No PSRAM"
//  Upload Speed: "921600"
//  USB CDC On Boot: "Enabled"
//  USB Firmware MSC on Boot: "Disabled"
//  USB DFU On Bot: "Disabled"
//  Upload Mode:"UART0/Hardware CDC"
//  SPU Frequency: "240MHz (WiFi)"
//  Flash Mode: "QIO 80MHz"
//  Flash SIze: "4MB (32Mb)"
//  Partition Scheme: "Default 4MB with spiffs (1.2MB app/1.5MB SPIFFS)"
//  Core Debug Level: "Verbose"
//  PSRAM: 'Disabled"
//  Arduino Runs On: "Core 1"
//  Events Run On: "Core 1"
//
//  To program, press and hold the reset button then press and hold program button, release the reset button then
//  release the program button


// Uncomment keywords to enable debugging output
// #define DEBUG_DRIVE_SPEED    1
// #define DEBUG_ENCODER_COUNT  1


#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>
#include <NewPing.h>

// Function declarations
void Indicator();  // for mode/heartbeat on Smart LED
long degreesToDutyCycle(int deg); // used for servo control
void backdoor(int index);
void sweep(int index);
void pause(int msDelay);
void dump(int index);

// Port pin constants
#define LEFT_MOTOR_A 35        // GPIO35 pin 28 (J35) Motor 1 A
#define LEFT_MOTOR_B 36        // GPIO36 pin 29 (J36) Motor 1 B
#define RIGHT_MOTOR_A 37       // GPIO37 pin 30 (J37) Motor 2 A
#define RIGHT_MOTOR_B 38       // GPIO38 pin 31 (J38) Motor 2 B
#define ENCODER_LEFT_A 15      // left encoder A signal is connected to pin 8 GPIO15 (J15)
#define ENCODER_LEFT_B 16      // left encoder B signal is connected to pin 8 GPIO16 (J16)
#define ENCODER_RIGHT_A 11     // right encoder A signal is connected to pin 19 GPIO11 (J11)
#define ENCODER_RIGHT_B 12     // right encoder B signal is connected to pin 20 GPIO12 (J12)
#define MODE_BUTTON 0          // GPIO0  pin 27 for Push Button 1
#define MOTOR_ENABLE_SWITCH 3  // DIP Switch S1-1 pulls Digital pin D3 to ground when on, connected to pin 15 GPIO3 (J3)
#define POT_R1 1               // when DIP Switch S1-3 is on, Analog AD0 (pin 39) GPIO1 is connected to Poteniometer R1
#define SMART_LED 21           // when DIP Switch S1-4 is on, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT 1      // number of SMART LEDs in use
#define IR_DETECTOR 14         // GPIO14 pin 17 (J14) IR detector input
#define TRIG_PIN 9             // J9 pin for trigger pin of ultrasonic sensor
#define ECHO_PIN 10            // J10 pin for echo pin of ultrasonic sensor
#define MAX_DISTANCE 10        // the max distance that the ultrasonic sensor is able to detect


// Constants
const int cDisplayUpdate = 100;           // update interval for Smart LED in milliseconds
const int cPWMRes = 8;                    // bit resolution for PWM
const int cMinPWM = 150;                  // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;  // PWM value for maximum speed
const int cCountsRev = 1096;              // number of encoder counts per revolution
const int cServoPin = 41;                 // GPIO pin for servo motor
const int cServoChannel = 6;


const int cServoPinScoop = 45;   // GPIO pin for servo motor
const int leftServoChannel = 4;  // PWM channel used for the RC servo motor
const int rightServoChannel = 5;
const int cServoChannelScoop = 7;
const int leftBucketServo = 42;
const int rightBucketServo = 44;
//=====================================================================================================================
//
// IMPORTANT: The constants in this section need to be set to appropriate values for your robot.
//            You will have to experiment to determine appropriate values.

const int cLeftAdjust = 0;   // Amount to slow down left motor relative to right
const int cRightAdjust = 0;  // to account for excess left movement            // Amount to slow down right motor relative to left

//=====================================================================================================================


// Variables
boolean motorsEnabled = true;         // motors enabled flag
boolean timeUp3sec = false;           // 3 second timer elapsed flag
boolean timeUp2sec = false;                                                    // 2 second timer elapsed flag

boolean timeUp200msec = false;        // 200 millisecond timer elapsed flag
unsigned char leftDriveSpeed;         // motor drive speed (0-255)
unsigned char rightDriveSpeed;        // motor drive speed (0-255)
unsigned char driveIndex;             // state index for run mode
unsigned int modePBDebounce;          // pushbutton debounce timer count
unsigned int potClawSetpoint;         // desired position of claw servo read from pot
unsigned int potShoulderSetpoint;     // desired position of shoulder servo read from pot
unsigned long timerCount3sec = 0;     // 3 second timer count in milliseconds
unsigned long timerCount2sec = 0;     // 2 second timer count in milliseconds
unsigned long timerCount200msec = 0;  // 200 millisecond timer count in milliseconds
unsigned long displayTime;            // heartbeat LED update timer
unsigned long previousMicros;         // last microsecond count
unsigned long currentMicros;          // current microsecond count
int numOfLoops = 0;                   // number of cycles the robot will go back and forth

char IRdata; 
float duration, distance, duration1, distance1;

// Declare SK6812 SMART LED object
//   Argument 1 = Number of LEDs (pixels) in use
//   Argument 2 = ESP32 pin number
//   Argument 3 = Pixel type flags, add together as needed:
//     NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//     NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//     NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//     NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//     NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel SmartLEDs(SMART_LED_COUNT, SMART_LED, NEO_RGB + NEO_KHZ800);


// smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0;
unsigned char LEDBrightnessLevels[] = { 5, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180, 195, 210, 225, 240, 255,
                                        240, 225, 210, 195, 180, 165, 150, 135, 120, 105, 90, 75, 60, 45, 30, 15 };

unsigned int robotModeIndex = 0;  // robot operational state
unsigned int modeIndicator[6] = {
  // colours for different modes
  SmartLEDs.Color(255, 0, 0),    //   red - stop
  SmartLEDs.Color(0, 255, 0),    //   green - run
  SmartLEDs.Color(0, 0, 255),    //   blue - empty case
  SmartLEDs.Color(255, 255, 0),  //   yellow - empty case
  SmartLEDs.Color(0, 255, 255),  //   cyan - empty case
  SmartLEDs.Color(255, 0, 255)   //   magenta - empty case
};



// Motor, encoder, and IR objects (classes defined in MSE2202_Lib)
Motion Bot = Motion();               // Instance of Motion for motor control
Encoders LeftEncoder = Encoders();   // Instance of Encoders for left encoder data
Encoders RightEncoder = Encoders();  // Instance of Encoders for right encoder data
IR Scan = IR();                                                                // instance of IR for detecting IR signals
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);  // Instance of ultrasonic sensor


void setup() {
#if defined DEBUG_DRIVE_SPEED || DEBUG_ENCODER_COUNT
  Serial.begin(115200);
#endif


  // Set up motors and encoders
  Bot.driveBegin("D1", LEFT_MOTOR_A, LEFT_MOTOR_B, RIGHT_MOTOR_A, RIGHT_MOTOR_B);  // set up motors as Drive 1
  LeftEncoder.Begin(ENCODER_LEFT_A, ENCODER_LEFT_B, &Bot.iLeftMotorRunning);       // set up left encoder
  RightEncoder.Begin(ENCODER_RIGHT_A, ENCODER_RIGHT_B, &Bot.iRightMotorRunning);   // set up right encoder

  Scan.Begin(IR_DETECTOR, 1200);                                              //set up IR Detection @ 1200 baud

  // Set up SmartLED
  SmartLEDs.begin();                                     // initialize smart LEDs object (REQUIRED)
  SmartLEDs.clear();                                     // clear pixel
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(0, 0, 0));  // set pixel colors to 'off'
  SmartLEDs.show();                                      // send the updated pixel colors to the hardware


  pinMode(MOTOR_ENABLE_SWITCH, INPUT_PULLUP);  // set up motor enable switch with internal pullup
  pinMode(MODE_BUTTON, INPUT_PULLUP);          // Set up mode pushbutton
  modePBDebounce = 0;                          // reset debounce timer count
  pinMode(TRIG_PIN, OUTPUT);                   // trigger pin will output signal
  pinMode(ECHO_PIN, INPUT);                    // echo pin will receive signal

  pinMode(cServoPin, OUTPUT);               // configure servo GPIO for output
  ledcSetup(cServoChannel, 50, 14);         // setup for channel for 50 Hz, 14-bit resolution
  ledcAttachPin(cServoPin, cServoChannel);  // assign servo pin to servo channel

  pinMode(cServoPinScoop, OUTPUT);                    // configure servo GPIO for output
  ledcSetup(cServoChannelScoop, 50, 14);              // setup for channel for 50 Hz, 14-bit resolution
  ledcAttachPin(cServoPinScoop, cServoChannelScoop);  // assign servo pin to servo channel

  pinMode(leftBucketServo, OUTPUT);                  // configure servo GPIO for output
  ledcSetup(leftServoChannel, 50, 14);               // setup for channel for 50 Hz, 14-bit resolution
  ledcAttachPin(leftBucketServo, leftServoChannel);  // assign servo pin to servo channel

  pinMode(rightBucketServo, OUTPUT);                   // configure servo GPIO for output
  ledcSetup(rightServoChannel, 50, 14);                // setup for channel for 50 Hz, 14-bit resolution
  ledcAttachPin(rightBucketServo, rightServoChannel);  // assign servo pin to servo channel
}

void loop() {

  // mycode
  LeftEncoder.getEncoderRawCount();  //read left encoder count
  RightEncoder.getEncoderRawCount();
  // Serial.print("Left Encoder count = ");
  // Serial.print(LeftEncoder.lRawEncoderCount);
  // Serial.print("Right Encoder count = ");
  // Serial.println(RightEncoder.lRawEncoderCount);
  long pos[] = {0, 0};                                                        // current motor positions
  int pot = 0;

  currentMicros = micros();                        // get current time in microseconds
  if ((currentMicros - previousMicros) >= 1000) {  // enter when 1 ms has elapsed
    previousMicros = currentMicros;                // record current time in microseconds

    // 3 second timer, counts 3000 milliseconds
    timerCount3sec = timerCount3sec + 1;  // increment 3 second timer count
    if (timerCount3sec > 3000) {          // if 3 seconds have elapsed
      timerCount3sec = 0;                 // reset 3 second timer count
      timeUp3sec = true;                  // indicate that 3 seconds have elapsed
    }

    // 200 millisecond timer, counts 200 milliseconds
    timerCount200msec = timerCount200msec + 1;  // Increment 200 millisecond timer count
    if (timerCount200msec > 200)                // If 200 milliseconds have elapsed
    {
      timerCount200msec = 0;  // Reset 200 millisecond timer count
      timeUp200msec = true;   // Indicate that 200 milliseconds have elapsed
    }

    // Mode pushbutton debounce and toggle
    if (!digitalRead(MODE_BUTTON)) {  // if pushbutton GPIO goes LOW (nominal push)
      // Start debounce
      if (modePBDebounce <= 25) {             // 25 millisecond debounce time
        modePBDebounce = modePBDebounce + 1;  // increment debounce timer count
        if (modePBDebounce > 25) {            // if held for at least 25 mS
          modePBDebounce = 1000;              // change debounce timer count to 1 second
        }
      }

      if (modePBDebounce >= 1000) {  // maintain 1 second timer count until release
        modePBDebounce = 1000;
      }
    } else {                       // pushbutton GPIO goes HIGH (nominal release)
      if (modePBDebounce <= 26) {  // if release occurs within debounce interval
        modePBDebounce = 0;        // reset debounce timer count
      } else {
        modePBDebounce = modePBDebounce + 1;    // increment debounce timer count
        if (modePBDebounce >= 1025) {           // if pushbutton was released for 25 mS
          modePBDebounce = 0;                   // reset debounce timer count
          robotModeIndex++;                     // switch to next mode
          robotModeIndex = robotModeIndex & 7;  // keep mode index between 0 and 7
          timerCount3sec = 0;                   // reset 3 second timer count
          timeUp3sec = false;                   // reset 3 second timer
        }
      }
    }


    // modes
    // 0 = Default after power up/reset. Robot is stopped.
    // 1 = Press mode button once to enter.        Run robot
    // 2 = Press mode button twice to enter.       Test IR receiver
    // 3 = Press mode button three times to enter. Test claw servo
    // 4 = Press mode button four times to enter.  Test shoulder servo
    // 5 = Press mode button five times to enter.  Add your code to do something
    // 6 = Press mode button six times to enter.   Add your code to do something
    switch (robotModeIndex) {
      case 0:  // Robot stopped
        Bot.Stop("D1");
        LeftEncoder.clearEncoder();  // clear encoder counts
        RightEncoder.clearEncoder();
        driveIndex = 0;  // reset drive index
        break;


      case 1:              // driving and collection system
        if (timeUp3sec) {  // pause for 3 sec before running case 1 - robotModeIndex code
          leftDriveSpeed = 255 - cLeftAdjust;
          rightDriveSpeed = 255 - cRightAdjust;
          #ifdef DEBUG_DRIVE_SPEED
                    if (timeUp200msec) {
                      timeUp200msec = false;
                      Serial.print(F(" Left Drive Speed: Pot R1 = "));
                      Serial.print(pot);
                      Serial.print(F(", mapped = "));
                      Serial.println(leftDriveSpeed);
                    }
          #endif
          #ifdef DEBUG_ENCODER_COUNT

                    LeftEncoder.getEncoderRawCount();   // read left encoder count
                    RightEncoder.getEncoderRawCount();  // read right encoder count
                    Serial.print(F("Left Encoder count = "));
                    Serial.print(LeftEncoder.lRawEncoderCount);
                    Serial.print(F("  Right Encoder count = "));
                    Serial.print(RightEncoder.lRawEncoderCount);
                    Serial.print("\n");

          #endif

          switch (driveIndex) {
            case 0:  // drive forward and collect stones

              Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed);  // drive ID, left speed, right speed
              if (abs(LeftEncoder.lRawEncoderCount) >= 4153/3) {   // 8306 => 100 cm, 4153/3 ~ 16cm
                LeftEncoder.clearEncoder();
                RightEncoder.clearEncoder();

                Bot.Stop("D1"); // stop driving
                pause(1000);
                sweep(1);       // close the sweeper
                dump(1);        // lift the bucket and dump the contents into the funnel
                pause(1000);
                dump(2);        // return bucket to original position
                sweep(2);       // return sweeper to original position
                pause(1000);
               if (numOfLoops >= 3) {  // loop this case 3 times
                  numOfLoops = 0;
                  driveIndex = 1;
                } else {
                  driveIndex = 0;
                  numOfLoops++;  // initially zero
                }
              }

              break;

            case 1:  // rotate 90 degrees CW
              Bot.Right("D1", 255, 255);
              if (abs(LeftEncoder.lRawEncoderCount) >= 1000) {   // 1000 encoder counts ~ 90 degrees
                LeftEncoder.clearEncoder();                      // 1273 => 180 degree rotation (for LAB 3 bot)
                RightEncoder.clearEncoder();
                driveIndex = 2;
              }
              break;

            case 2:  // drive forward 25 cm
              Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); 
              if (abs(LeftEncoder.lRawEncoderCount) >= 2077) {   // 2077 ~ 25 cm
                LeftEncoder.clearEncoder();
                RightEncoder.clearEncoder();
                driveIndex = 3;
              }
              break;

            case 3:  // rotate 90 degrees CW
              Bot.Right("D1", 255, 255);
              if (abs(LeftEncoder.lRawEncoderCount) >= 1000) {   // 1000 encoder counts ~ 90 degrees
                LeftEncoder.clearEncoder();
                RightEncoder.clearEncoder();
                driveIndex = 4;
              }
              break;

            case 4:  // drive forwards and collect stones
              Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed);
              if (abs(LeftEncoder.lRawEncoderCount) >= 3322/3) {   // 3322/3 ~ 13 cm
                LeftEncoder.clearEncoder();
                RightEncoder.clearEncoder();
                Bot.Stop("D1");
                pause(1000);
                sweep(1);
                dump(1);
                pause(1000);
                dump(2);
                sweep(2);
                pause(1000);
                if (numOfLoops >= 3) {  // loop this case 3 times
                  numOfLoops = 0;
                  driveIndex = 7;
                } else {
                  driveIndex = 4;
                  numOfLoops++;  // initially zero
                }
              }

              break;
            /* // These 2 cases were not used in the final run
            case 5:
              Bot.Left("D1", 255, 255);
              if (abs(LeftEncoder.lRawEncoderCount) >= 1050) {  // 1273 => 180 degree rotation (for LAB 3 bot)
                LeftEncoder.clearEncoder();                     // condition was counting right encoder counts
                RightEncoder.clearEncoder();
                Serial.printf("case 5 ends");
                driveIndex = 6;  //go to case 2
              }
              break;

            case 6:
              Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed);  //drive ID, left speed, right speed
              if (abs(LeftEncoder.lRawEncoderCount) >= 2077) {     // 2077 => 25 cm forward
                LeftEncoder.clearEncoder();
                RightEncoder.clearEncoder();
                Serial.printf("case 6 ends");
                driveIndex = 7;
              }
              break;
            */
           
            case 7:  // rotate 90 degrees CW
              Bot.Right("D1", 255, 255);
              if (abs(LeftEncoder.lRawEncoderCount) >= 1050) {  // 1273 => 180 degree rotation (for LAB 3 bot)
                LeftEncoder.clearEncoder();
                RightEncoder.clearEncoder();
                driveIndex = 9;
              }
              break;
            /* // This case was not used in the final run
            case 8:
              Bot.Left("D1", 255, 255);                         // turn robot to go back to start
              if (abs(LeftEncoder.lRawEncoderCount) >= 1050) {  // 1050 ~ 90 degrees
                LeftEncoder.clearEncoder();
                RightEncoder.clearEncoder();
                driveIndex = 9;
              }
              break;
            */
            case 9:  // drive forwards ~ 26 cm
              Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed);
              if (abs(LeftEncoder.lRawEncoderCount) >= 2180) {   // 2180 ~ 26 cm
                LeftEncoder.clearEncoder();
                RightEncoder.clearEncoder();
                driveIndex = 10;
              }

              break;


            case 10:  // turn to face the same direction as initial direction
              Bot.Right("D1", 255, 255);
              if (abs(LeftEncoder.lRawEncoderCount) >= 1050) {  // 1050 ~ 90 degrees
                LeftEncoder.clearEncoder();
                RightEncoder.clearEncoder();
                driveIndex = 11;
              }
              break;

            case 11:  // stop driving and dump contents into funnel
              Bot.Stop("D1");
              pause(1000);
              sweep(1);    // close the sweper
              dump(1);     // lift the bucket
              pause(1000);
              LeftEncoder.clearEncoder();
              RightEncoder.clearEncoder();
              driveIndex = 12;
              break;

            case 12:  // scan for IR
              Bot.Left("D1", leftDriveSpeed - 55, rightDriveSpeed - 55);

              if(Scan.Available()){
                IRdata = Scan.Get_IR_Data();     // read the character from the IR sensor
                                              
                Serial.printf("%d\n" , IRdata);
                if ((IRdata == 'U')){            // if the signal is strong, the bot will stop and move to next case
                  Bot.Stop("D1");
                  Serial.println("***IR DETECTED***");                                            
                  driveIndex = 13;
                } 
              }
              break;

            case 13:// go towards IR base
              digitalWrite(TRIG_PIN, LOW);        // ultrasonic sensor sends out and "listens" for signal coming back
              delayMicroseconds(2);
              digitalWrite(TRIG_PIN, HIGH);
              delayMicroseconds(10);
              digitalWrite(TRIG_PIN, LOW);

              duration = pulseIn(ECHO_PIN, HIGH);
              distance = (duration*.0343)/2;      // calculate the distance based on the speed of sound waves
              Serial.print("Distance: ");
              Serial.println(distance);
              pause(100);

              if ((distance >= 2.5 && distance <= 3.5) || (distance > 2300)) { // if the distance is between 2.5 and 3.5 cm, the bot will stop and move to next case
                Bot.Stop("D1");
                Serial.print("***STOP***");
                driveIndex = 14;
              }
              else {
                Bot.Reverse("D1", leftDriveSpeed - 55, rightDriveSpeed - 55); // otherwise the bot will keep reversing backwards
              }
              break;
                

            case 14:  // Open backdoor slowly => drops green stones into collection bin
              Bot.Reverse("D1", leftDriveSpeed-55, rightDriveSpeed-55);
              if (abs(LeftEncoder.lRawEncoderCount) >= 10) {
                backdoor(1); // open the backdoor
                LeftEncoder.clearEncoder();
                RightEncoder.clearEncoder();
                driveIndex = 15;
                robotModeIndex = 0; // set the bot into stop mode
              }
              break;
          }
          break;

          // case not used
          case 17:                //add your code to do something
            robotModeIndex = 0;  //  !!!!!!!  remove if using the case
            break;
        }
        // Update brightness of heartbeat display on SmartLED
        displayTime++;                                             // count milliseconds
        if (displayTime > cDisplayUpdate) {                        // when display update period has passed
          displayTime = 0;                                         // reset display counter
          LEDBrightnessIndex++;                                    // shift to next brightness level
          if (LEDBrightnessIndex > sizeof(LEDBrightnessLevels)) {  // if all defined levels have been used
            LEDBrightnessIndex = 0;                                // reset to starting brightness
          }
          SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]);  // set brightness of heartbeat LED
          Indicator();                                                       // update LED
        }
    }
  }
}
// Set colour of Smart LED daepending on robot mode (and update brightness)
void Indicator() {
  SmartLEDs.setPixelColor(0, modeIndicator[robotModeIndex]);  // set pixel colors to = mode
  SmartLEDs.show();                                           // send the updated pixel colors to the hardware
}

// open/close the backdoor
void backdoor(int index) {
  int i = 0;
  switch (index) {
    case 1: // opens the backdoor
      for (i = 0; i <= 100; i += 1) {
        ledcWrite(cServoChannel, degreesToDutyCycle(i));
        pause(10);
      }

      break;
    case 2: // closes the backdoor
      for (i = 100; i >= 0; i -= 1) {
        ledcWrite(cServoChannel, degreesToDutyCycle(i));
        pause(10);
      }
      break;
  }
}

long degreesToDutyCycle(int deg) {
  const long cMinDutyCycle = 400;   // duty cycle for 0 degrees
  const long cMaxDutyCycle = 2100;  // duty cycle for 180 degrees

  long dutyCycle = map(deg, 0, 180, cMinDutyCycle, cMaxDutyCycle);  // convert to duty cycle

#ifdef OUTPUT_ON
  float percent = dutyCycle * 0.0061039;  // (dutyCycle / 16383) * 100
  Serial.printf("Degrees %d, Duty Cycle Val: %ld = %f%%\n", servoPos, dutyCycle, percent);
#endif

  return dutyCycle;
}

void pause(int msDelay) {
  bool timeUp = false;
  unsigned long timerVal = 0;
  while (!timeUp) {
    currentMicros = micros();
    if ((currentMicros - previousMicros) >= 1000) {
      previousMicros = currentMicros;
      timerVal += 1;
      if (timerVal >= msDelay) {
        timeUp = true;
      }
    }
  }
}

// open/close the sweeper arm
void sweep(int index) {
  int i;
  switch (index) {
    case 1:  // closes the sweeper
      for (i = 0; i <= 90; i += 1) {
        ledcWrite(cServoChannelScoop, degreesToDutyCycle(i));
        pause(5);
      }

      break;
    case 2:  // opens the sweeper
      for (i = 90; i >= 0; i -= 1) {
        ledcWrite(cServoChannelScoop, degreesToDutyCycle(i));
        pause(5);
      }
      break;
  }
}

// used to dump the contents of the collection bucket into the funnel
void dump(int index) {
  int l, r;  // servos at rest position => l = 110, r = 0
  switch (index) {
    case 1:  // bucket goes up
      for (l = 110, r = 0; l >= 0; l -= 2, r += 2) {
        ledcWrite(leftServoChannel, degreesToDutyCycle(l));  // set the desired servo position
        ledcWrite(rightServoChannel, degreesToDutyCycle(r));
        pause(10);
      }
      break;

    case 2:  // bucket goes down
      for (l = 0, r = 110; l <= 110; l += 2, r -= 2) {
        ledcWrite(leftServoChannel, degreesToDutyCycle(l));  // set the desired servo position
        ledcWrite(rightServoChannel, degreesToDutyCycle(r));
        pause(10);
      }
      break;
  }
}
