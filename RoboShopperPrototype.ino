#include <AFMotor.h>                              //Library to Control L293D Motor Drive Shield
#include <Servo.h>                                //Library to Control Ultrasonic Sensor's Servo Motor
#include <IRremote.h>                             //Library to Decode Infrared Remote Contol Signals

//IR Remote Control
byte receiver = A0;                               //Set IR Receiver Pin = A0
IRrecv irrecv(receiver);                          //Create an object to control the IR Sensor
decode_results code;                              //Create a object to store the IR Sensor's results

//DC Motors - Wheels
AF_DCMotor rightBack(1);                          //Create an object to control each motor
AF_DCMotor rightFront(2);
AF_DCMotor leftFront(3);
AF_DCMotor leftBack(4);

byte motorSpeed = 65;                             //The maximum motor speed
int motorOffset = 35;                             //Factor to account for one side being more powerful
int turnSpeed = 50;                               //Amount to add to motor speed when turning

//Ultra Sonic Sensor
Servo servoUltraSonic;                            //Create an object to control the Ultrasonic Sensor's Servo Motor
byte trig = 2;
byte echo = 13;

byte maxDist = 75;                                
byte stopDist = 25;
float timeOut = 2*(maxDist+10)/100/340*1000000;   //Maximum time to wait for a return signal


void setup()
{
  //Arduino Setup
  Serial.begin(9600);                             //Set Baud rateto 9600

  //IR Remote Setup
  irrecv.enableIRIn();                            //Enable the IR Receiver
  irrecv.blink13(true);                           //Enable the IR Receiver's LED

  //Motors Setup
  rightBack.setSpeed(motorSpeed);                 //Set the motors to the motor speed
  rightFront.setSpeed(motorSpeed);
  leftFront.setSpeed(motorSpeed+motorOffset);
  leftBack.setSpeed(motorSpeed+motorOffset);

  rightBack.run(RELEASE);                         //Ensure all motors are stopped
  rightFront.run(RELEASE);
  leftFront.run(RELEASE);
  leftBack.run(RELEASE);

  //UltraSonic Sensor Setup
  servoUltraSonic.attach(10);                   //Assign the Servo Motor to Pin 10
  pinMode(trig,OUTPUT);                         //Assign the Ultrasonic Sensor Pins
  pinMode(echo,INPUT);
}

void loop()
{
  servoUltraSonic.write(90);                  //Set the Servo to Look Straight Ahead (90deg)
  delay(750);                                 //Wait for 750ms for Servo to move

  // if (irrecv.decode(&results)) {
  //   Serial.println(results.value, HEX);
  //   irrecv.resume();
  // }

  int distance = getDistance();               //Check if their are objects infront of the cart
  if (distance >= stopDist) {
    moveForward();
  }
  while (distance >= stopDist) {              //Continue checking the object distance until within minimum stopping distance
    distance = getDistance();
    delay(250);
  }
  stopMove();
  delay(5000);
  turnLeft(550);
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm");
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

int getDistance()                                 //Measure the distance to an object
{
  unsigned long pulse;                            //Pulse Travel Time
  int distance;

  digitalWrite(trig, HIGH);                       //Generate a 10 microseconds pulse
  delayMicroseconds(10);
  digitalWrite(trig,LOW);

  pulse = pulseIn(echo,HIGH,timeOut);             //Measure the time for the pulse to return to the Ultrasonic Sensor
  distance = (float)pulse * 340/2/10000;          //Calculate the object distance based on the pulse

  return distance;
}
