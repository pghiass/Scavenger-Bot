#include <Arduino.h>
#include <Stepper.h>

// Button structure
struct Button {
  const int pin;                                       // GPIO pin for button
  unsigned int numberPresses;                          // counter for number of button presses
  unsigned int lastPressTime;                          // time of last button press in ms
  bool pressed;                                        // flag for button press event
};

const int stepsPerRevolution = 1313;
Stepper myStepper (stepsPerRevolution, 39, 40);
Button button = {0, 0, 0, false};

void setup() {
  // set the speed at 60 rpm:
  myStepper.setSpeed(80);
  // initialize the serial port:
  Serial.begin(115200);
}

void loop() {
  // step one revolution  in one direction:
  Serial.println("clockwise");
  myStepper.step(stepsPerRevolution);
  delay(500);

  // step one revolution in the other direction:
  Serial.println("counterclockwise");
  myStepper.step(-stepsPerRevolution);
  delay(500);
}