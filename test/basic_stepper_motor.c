 /* Example sketch to control a stepper motor with L298N motor driver, Arduino UNO and Stepper.h library. More info: https://www.makerguides.com */
// Include the Stepper library:
#include <Stepper.h>
#include <AFMotor.h>

// Define number of steps per revolution:
const int stepsPerRevolution = 200;
// Initialize the stepper library on pins 8 through 11:
Stepper myStepper = Stepper(stepsPerRevolution, 6, 7, 8, 9);
Stepper myStepper2 = Stepper(stepsPerRevolution, 2, 3, 4, 5);
void setup() {
  // Set the motor speed (RPMs):
  myStepper.setSpeed(100);
  myStepper2.setSpeed(100);
}
void loop() {
  // Step one revolution in one direction:
  myStepper.step(20);
  delay(2000);
  myStepper.step(-20);
  delay(2000);


   // Step one revolution in one direction:
  myStepper2.step(20);
  delay(2000);
  myStepper2.step(-20);
  delay(2000);

}
