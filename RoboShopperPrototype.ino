#include <AFMotor.h>                              //Library to Control L293D Motor Drive Shield
#include <Servo.h>                                //Library to control Ultrasonic Sensor's Servo Motor

AF_DCMotor rightBack(1);                          //Create an object to control each motor
AF_DCMotor rightFront(2);
AF_DCMotor leftFront(3);
AF_DCMotor leftBack(4);

byte motorSpeed = 65;                             //The maximum motor speed
int motorOffset = 35;                             //Factor to account for one side being more powerful
int turnSpeed = 50;                               //Amount to add to motor speed when turning


void setup()
{
  rightBack.setSpeed(motorSpeed);                 //Set the motors to the motor speed
  rightFront.setSpeed(motorSpeed);
  leftFront.setSpeed(motorSpeed+motorOffset);
  leftBack.setSpeed(motorSpeed+motorOffset);

  rightBack.run(RELEASE);                         //Ensure all motors are stopped
  rightFront.run(RELEASE);
  leftFront.run(RELEASE);
  leftBack.run(RELEASE);
}

void loop()
{
  moveForward();                              //Move Forward
  delay(5000);                                //For 5 seconds

  stopMove();                                 //Stop
  delay(5000);                                //For 5 seconds

  turnLeft (750);                             //Turn Left
  delay(5000);                                //For 5 seconds

  // turnLeft (700);                             //Turn Left 180deg
  // delay(5000);                                //For 5 seconds

  // turnLeft (2600);                             //Turn Left 360deg
  // delay(5000);                                //For 5 seconds

  // turnRight (400);                            //Turn Right
  // delay(5000);                                //For 5 seconds
}

void accelerate()                                 //Function to accelerate the motors from 0 to full speed
{
  for (int i=0; i<motorSpeed; i++)                //Loop from 0 to full speed
  {
    rightBack.setSpeed(i);                        //Set the motors to the current loop speed
    rightFront.setSpeed(i);
    leftFront.setSpeed(i+motorOffset);
    leftBack.setSpeed(i+motorOffset);
    delay(10);
  }
}

void decelerate()                                 //Function to decelerate the motors from full speed to zero
{
  for (int i=motorSpeed; i!=0; i--)               //Loop from full speed to 0
  {
    rightBack.setSpeed(i);                        //Set the motors to the current loop speed
    rightFront.setSpeed(i);
    leftFront.setSpeed(i+motorOffset);
    leftBack.setSpeed(i+motorOffset);
    delay(10);
  }
}

void moveForward()                                //Set all motors to run forward
{
  rightBack.run(FORWARD);
  rightFront.run(FORWARD);
  leftFront.run(FORWARD);
  leftBack.run(FORWARD);
}

void stopMove()                                   //Set all motors to stop
{
  rightBack.run(RELEASE);
  rightFront.run(RELEASE);
  leftFront.run(RELEASE);
  leftBack.run(RELEASE);
}

void turnLeft(int duration)                                 //Set motors to turn left for the specified duration then stop
{
  rightBack.setSpeed(motorSpeed+turnSpeed);                 //Set the motors to the motor speed
  rightFront.setSpeed(motorSpeed+turnSpeed);
  leftFront.setSpeed(motorSpeed+motorOffset+turnSpeed);
  leftBack.setSpeed(motorSpeed+motorOffset+turnSpeed);
  rightBack.run(FORWARD);
  rightFront.run(FORWARD);
  leftFront.run(BACKWARD);
  leftBack.run(BACKWARD);
  delay(duration);
  rightBack.setSpeed(motorSpeed);                           //Set the motors to the motor speed
  rightFront.setSpeed(motorSpeed);
  leftFront.setSpeed(motorSpeed+motorOffset);
  leftBack.setSpeed(motorSpeed+motorOffset);
  rightBack.run(RELEASE);
  rightFront.run(RELEASE);
  leftFront.run(RELEASE);
  leftBack.run(RELEASE);

}

void turnRight(int duration)                                //Set motors to turn right for the specified duration then stop
{
  rightBack.setSpeed(motorSpeed+turnSpeed);                 //Set the motors to the motor speed
  rightFront.setSpeed(motorSpeed+turnSpeed);
  leftFront.setSpeed(motorSpeed+motorOffset+turnSpeed);
  leftBack.setSpeed(motorSpeed+motorOffset+turnSpeed);
  rightBack.run(BACKWARD);
  rightFront.run(BACKWARD);
  leftFront.run(FORWARD);
  leftBack.run(FORWARD);
  delay(duration);
  rightBack.setSpeed(motorSpeed);                           //Set the motors to the motor speed
  rightFront.setSpeed(motorSpeed);
  leftFront.setSpeed(motorSpeed+motorOffset);
  leftBack.setSpeed(motorSpeed+motorOffset);
  rightBack.run(RELEASE);
  rightFront.run(RELEASE);
  leftFront.run(RELEASE);
  leftBack.run(RELEASE);
}
