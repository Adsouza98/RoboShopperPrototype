#include <AFMotor.h>                             //Library to Control L293D Motor Drive Shield
#include <Servo.h>                               //Library to Control Ultrasonic Sensor's Servo Motor
#include <MPU6050_tockn.h>                       //Library to Control MPU6050 Sensor
#include <Wire.h>                                //Library to Control I2C connection required to use MPU6050 Sensor

//Timings
long coToBakery_Time = 3400;                     //Checkout -> Bakery Time
long coToProduce_Time = 4900;                    //Checkout -> Produce Time
long bakeryToMeats_Time = 4800;                  //Checkout -> Meats

//MPU6050 Sensor
MPU6050 mpu6050(Wire);                           //Create an object to track MPU6050 data

//DC Motors - Wheels
AF_DCMotor rightBack(1);                         //Create an object to control each motor
AF_DCMotor rightFront(2);
AF_DCMotor leftFront(3);
AF_DCMotor leftBack(4);

byte motorSpeed = 67;                            //The maximum motor speed
int motorOffset = 33;                            //Factor to account for one side being more powerful
int turnSpeed = 50;                              //Amount to add to motor speed when turning

//Ultra Sonic Sensor
Servo servoUltraSonic;                           //Create an object to control the Ultrasonic Sensor's Servo Motor
byte trig = 2;
byte echo = 13;
int straight = 95;
int left = 180;
int right = 12;

byte maxDist = 75;                                
byte stopDist = 25;
float timeOut = 2*(maxDist+10)/100/340*1000000;  //Maximum time to wait for a return signal


void setup()
{
  //Arduino Setup
  Serial.begin(9600);                            //Set Baud rate to 9600

  //MPU6050 Setup
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  //Motors Setup
  rightBack.setSpeed(motorSpeed);                //Set the motors to the motor speed
  rightFront.setSpeed(motorSpeed);
  leftFront.setSpeed(motorSpeed+motorOffset);
  leftBack.setSpeed(motorSpeed+motorOffset);

  rightBack.run(RELEASE);                        //Ensure all motors are stopped
  rightFront.run(RELEASE);
  leftFront.run(RELEASE);
  leftBack.run(RELEASE);

  //UltraSonic Sensor Setup
  servoUltraSonic.attach(10);                    //Assign the Servo Motor to Pin 10
  pinMode(trig,OUTPUT);                          //Assign the Ultrasonic Sensor Pins
  pinMode(echo,INPUT);
}

void loop()
{
  servoUltraSonic.write(straight);                     //Set the Servo to Look Straight Ahead (90deg)
  delay(750);                                          //Wait for 750ms for Servo to move

  // Checkout -> Bakery
  // checkoutToBakery();

  // Checkout -> Produce
  checkoutToProduce();

  // Checkout -> Meats
  // checkoutToMeats();

  // Checkout -> Dairy
  // checkoutToDairy();

  // Produce -> Meats (Southbound)
  // produceToMeats('S');

  // Produce -> Bakery (Southbound)
  // produceToBakery('S');

  // Produce -> Dairy (Westbound)
  // produceToDairy('W');

  // Produce -> Checkout (Westbound)
  // produceToCheckout('W');

  // Meats -> Bakery (Southbound)
  // meatsToBakery('S');

  // Meats -> Bakery (Eastbound)
  // meatsToBakery('E');

  // Bakery -> Dairy
  // bakeryToDairy();

  // Bakery -> Meats
  // bakeryToMeats();

  // Bakery -> Checkout (Westbound)
  // bakeryToCheckout('W');

  // Meats -> Produce (Needs to be verified)
  // meatsToProduce();

  // Dairy -> Bakery
  // dairyToBakery();

  // Dairy -> Meats
  // dairyToMeats();

  // Dairy -> Produce (Northbound)
  // dairyToProduce('N');

  // Dairy -> Produce (Southbound)
  // dairyToProduce('S');

  // Dairy -> Checkout (Northbound)
  // dairyToCheckout('N');

  // Dairy -> Checkout (Southbound)
  //dairyToCheckout('S');



  while(true){}


}

void accelerate()                                //Function to accelerate the motors from 0 to full speed
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

void decelerate()                                //Function to decelerate the motors from full speed to zero
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

void moveForward()                               //Set all motors to run forward
{
  rightBack.run(FORWARD);
  rightFront.run(FORWARD);
  leftFront.run(FORWARD);
  leftBack.run(FORWARD);
}

void stopMove()                                  //Set all motors to stop
{
  rightBack.run(RELEASE);
  rightFront.run(RELEASE);
  leftFront.run(RELEASE);
  leftBack.run(RELEASE);
}

void turnLeft(float currentAngle)                //Set motors to turn left for the specified duration then stop
{
  rightBack.setSpeed(motorSpeed+turnSpeed);                 //Set the motors to the motor speed
  rightFront.setSpeed(motorSpeed+turnSpeed);
  leftFront.setSpeed(motorSpeed+motorOffset+turnSpeed);
  leftBack.setSpeed(motorSpeed+motorOffset+turnSpeed);
  
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  do {
    rightBack.run(FORWARD);
    rightFront.run(FORWARD);
    leftFront.run(BACKWARD);
    leftBack.run(BACKWARD);
    mpu6050.update();
    Serial.print("\tTurning\tangleZ : ");
    Serial.println(mpu6050.getAngleZ());
  } while (mpu6050.getAngleZ() < (currentAngle + 155));
  stopMove();

  rightBack.setSpeed(motorSpeed);                           //Set the motors to the motor speed
  rightFront.setSpeed(motorSpeed);
  leftFront.setSpeed(motorSpeed+motorOffset);
  leftBack.setSpeed(motorSpeed+motorOffset);
}

void turnRight(long currentAngle)                //Set motors to turn right for the specified duration then stop
{
  rightBack.setSpeed(motorSpeed+turnSpeed);                 //Set the motors to the motor speed
  rightFront.setSpeed(motorSpeed+turnSpeed);
  leftFront.setSpeed(motorSpeed+motorOffset+turnSpeed);
  leftBack.setSpeed(motorSpeed+motorOffset+turnSpeed);

  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());

  do {
    rightBack.run(BACKWARD);
    rightFront.run(BACKWARD);
    leftFront.run(FORWARD);
    leftBack.run(FORWARD);
    mpu6050.update();
    Serial.print("\tTurning\tangleZ : ");
    Serial.println(mpu6050.getAngleZ());
  } while(mpu6050.getAngleZ() > currentAngle - 145);
  stopMove();

  rightBack.setSpeed(motorSpeed);                           //Set the motors to the motor speed
  rightFront.setSpeed(motorSpeed);
  leftFront.setSpeed(motorSpeed+motorOffset);
  leftBack.setSpeed(motorSpeed+motorOffset);
}

int getDistance()                                //Measure the distance to an object
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

void lookLeft()
{
  servoUltraSonic.write(left);                    //Set the Servo to Look Left (180deg = 190degs)
  delay(750);
  int distance = getDistance();
  while (distance <= stopDist) {
    distance = getDistance();
    stopMove();
  }
  servoUltraSonic.write(straight);
}

void lookRight()
{
  servoUltraSonic.write(right);                     //Set the Servo to Look Right (0deg)
  delay(750);
  int distance = getDistance();
  while (distance <= stopDist) {
    distance = getDistance();
    stopMove();
  }
  servoUltraSonic.write(straight);
}

void traversal(long travelTime)
{
  bool destination = false;
  unsigned long StartTime = 0;
  unsigned long CurrentTime = 0;
  unsigned long PausedTime = 0;
  long ElaspedTime = 0;
  int distance = getDistance();                  //Check if their are objects infront of the cart
  StartTime = millis();
  while (destination != true) {
    distance = getDistance();                    //Check if their are objects infront of the cart
    delay(250);

    if (distance >= stopDist) {
      moveForward();
      CurrentTime = millis();
      ElaspedTime = CurrentTime - PausedTime - StartTime;
    }

    while ( (distance >= stopDist) && (ElaspedTime < travelTime)) {              //Continue checking the object distance until within minimum stopping distance
      distance = getDistance();
      delay(250);
      CurrentTime = millis();
      ElaspedTime = CurrentTime - PausedTime - StartTime;
    }

    while ( (distance <= stopDist) && (ElaspedTime < travelTime) ) {
      stopMove();
      distance = getDistance();
      delay(250);
      PausedTime = millis() - CurrentTime + 250;
    }

    ElaspedTime = CurrentTime - PausedTime - StartTime;

    if (ElaspedTime >= travelTime) {
      destination = true;
    }

  }
}

char checkoutToBakery()                          //Checkout -> Bakery
{
  traversal(coToBakery_Time);
  stopMove();

  lookLeft();

  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnLeft(mpu6050.getAngleZ());
  return 'E';
}

char checkoutToProduce()                         //Checkout -> Produce
{
  lookLeft();
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnLeft(mpu6050.getAngleZ());

  traversal(coToProduce_Time);

  stopMove();
  lookRight();
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnRight(mpu6050.getAngleZ());
  return 'S';
}

char checkoutToMeats()                           //Checkout -> Meats
{
  checkoutToBakery();
  bakeryToMeats('E');
  return 'N';
}

char checkoutToDairy()                           //Checkout -> Dairy
{
  lookLeft();
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnLeft(mpu6050.getAngleZ());

  traversal(2300);
  stopMove();

  lookRight();

  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnRight(mpu6050.getAngleZ());

  traversal(1600);
  stopMove();
  return 'S';
}

char bakeryToDairy(char dir)                     //Bakery -> Dairy
{
  if (dir == 'N') {
    lookRight();
    mpu6050.update();
    Serial.print("\tangleZ : ");
    Serial.println(mpu6050.getAngleZ());
    turnRight(mpu6050.getAngleZ());
  }
  traversal(2250);
  stopMove();

  lookLeft();
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnLeft(mpu6050.getAngleZ());

  traversal(1700);
  stopMove();
  return 'N';
}

char bakeryToMeats(char dir)                     //Bakery -> Meats
{
  if (dir == 'N') {
    lookRight();
    mpu6050.update();
    Serial.print("\tangleZ : ");
    Serial.println(mpu6050.getAngleZ());
    turnRight(mpu6050.getAngleZ());
  }
  traversal(bakeryToMeats_Time);
  stopMove();

  lookLeft();
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnLeft(mpu6050.getAngleZ());
  return 'N';
}

char bakeryToProduce(char dir)                   //Bakery -> Produce (Not Stable because of meatsToProduce())
{
  if (dir == 'N') {
    bakeryToMeats('N');
  } else {
    bakeryToMeats('E');
  }
  meatsToProduce('N');
  return 'W';
}

void bakeryToCheckout(char dir)                  //Bakery -> Checkout
{
  if (dir == 'E') {
    lookLeft();
    mpu6050.update();
    Serial.print("\tangleZ : ");
    Serial.println(mpu6050.getAngleZ());
    turnLeft(mpu6050.getAngleZ());
  }
  traversal(3000);
  stopMove();
}

char meatsToProduce(char dir)                    //Meats -> Produce (Not Stable)
{
  if (dir == 'W') {
    lookRight();
    mpu6050.update();
    Serial.print("\tangleZ : ");
    Serial.println(mpu6050.getAngleZ());
    turnRight(mpu6050.getAngleZ());
  }
  traversal(2000);
  stopMove();
  lookLeft();
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnLeft(mpu6050.getAngleZ());
  return 'W';
}

char meatsToBakery(char dir)                     //Meats -> Bakery
{
  if (dir == 'N') {
    lookLeft();
    mpu6050.update();
    Serial.print("\tangleZ : ");
    Serial.println(mpu6050.getAngleZ());
    turnLeft(mpu6050.getAngleZ());
  }
  traversal(4800);
  stopMove();
  lookRight();
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnRight(mpu6050.getAngleZ());
  return 'N';
}

char meatsToDairy(char dir)                      //Meats -> Dairy (Need to Test)
{
  if (dir == 'N') {
    lookLeft();
    mpu6050.update();
    Serial.print("\tangleZ : ");
    Serial.println(mpu6050.getAngleZ());
    turnLeft(mpu6050.getAngleZ());
  }

  traversal(2700);
  stopMove();
  lookRight();
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnRight(mpu6050.getAngleZ());
  traversal(1850);
  stopMove();
  return 'N';
}

void meatsToCheckout(char dir)                   //Meats -> Checkout (Need to Test)
{
  if (dir == 'N') {
    meatsToProduce('N');
    produceToCheckout('W');
  } else {
    meatsToBakery('W');
    bakeryToCheckout('N');
  }
}

char dairyToBakery(char dir)                     //Dairy -> Bakery
{
  if (dir == 'N') {
    lookRight();
    mpu6050.update();
    Serial.print("\tangleZ : ");
    Serial.println(mpu6050.getAngleZ());
    turnRight(mpu6050.getAngleZ());
    lookRight();
    mpu6050.update();
    Serial.print("\tangleZ : ");
    Serial.println(mpu6050.getAngleZ());
    turnRight(mpu6050.getAngleZ());
  }
  traversal(1850);
  stopMove();
  lookRight();
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnRight(mpu6050.getAngleZ());

  traversal(2250);
  stopMove();
  lookRight();
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnRight(mpu6050.getAngleZ());
  return 'N';
}

char dairyToMeats(char dir)                      //Dairy -> Meats
{
  if (dir == 'N') {
    lookRight();
    mpu6050.update();
    Serial.print("\tangleZ : ");
    Serial.println(mpu6050.getAngleZ());
    turnRight(mpu6050.getAngleZ());
    lookRight();
    mpu6050.update();
    Serial.print("\tangleZ : ");
    Serial.println(mpu6050.getAngleZ());
    turnRight(mpu6050.getAngleZ());
  }
  traversal(1850);
  stopMove();
  lookLeft();
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnLeft(mpu6050.getAngleZ());

  traversal(2700);
  stopMove();
  lookLeft();
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnLeft(mpu6050.getAngleZ());
  return 'N';
}

char dairyToProduce(char dir)                    //Dairy -> Produce
{
  if (dir == 'S') {
    lookRight();
    mpu6050.update();
    Serial.print("\tangleZ : ");
    Serial.println(mpu6050.getAngleZ());
    turnRight(mpu6050.getAngleZ());
    lookRight();
    mpu6050.update();
    Serial.print("\tangleZ : ");
    Serial.println(mpu6050.getAngleZ());
    turnRight(mpu6050.getAngleZ());
  }
  traversal(1500);
  stopMove();
  lookRight();
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnRight(mpu6050.getAngleZ());

  traversal(2300);
  stopMove();
  lookRight();
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnRight(mpu6050.getAngleZ());
  return 'S';
}

void dairyToCheckout(char dir)                   //Dairy -> Checkout
{
  if (dir == 'S') {
    lookRight();
    mpu6050.update();
    Serial.print("\tangleZ : ");
    Serial.println(mpu6050.getAngleZ());
    turnRight(mpu6050.getAngleZ());
    lookRight();
    mpu6050.update();
    Serial.print("\tangleZ : ");
    Serial.println(mpu6050.getAngleZ());
    turnRight(mpu6050.getAngleZ());
  }
  traversal(1500);
  stopMove();
  lookLeft();
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnLeft(mpu6050.getAngleZ());

  traversal(2300);
  stopMove();
  lookLeft();
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnLeft(mpu6050.getAngleZ());

}

char produceToMeats(char dir)                    //Produce -> Meats
{
  if (dir == 'W') {
    lookLeft();
    mpu6050.update();
    Serial.print("\tangleZ : ");
    Serial.println(mpu6050.getAngleZ());
    turnLeft(mpu6050.getAngleZ());
  }

  traversal(3200);
  stopMove();
  lookRight();
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnRight(mpu6050.getAngleZ());
  return 'W';
}

char produceToBakery(char dir)                   //Produce -> Bakery
{
  if (dir == 'W') {
    produceToMeats('W');
  } else {
    produceToMeats('S');
  }
  meatsToBakery('W');
  return 'N';
}

char produceToDairy(char dir)                    //Produce -> Dairy
{
  if (dir == 'S') {
    lookRight();
    mpu6050.update();
    Serial.print("\tangleZ : ");
    Serial.println(mpu6050.getAngleZ());
    turnRight(mpu6050.getAngleZ());
  }
  traversal(3000);
  stopMove();
  lookLeft();
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnLeft(mpu6050.getAngleZ());
  traversal(1600);
  stopMove();
  return 'S';
}

void produceToCheckout(char dir)                 //Produce -> Checkout
{
  if (dir == 'S') {
    lookRight();
    mpu6050.update();
    Serial.print("\tangleZ : ");
    Serial.println(mpu6050.getAngleZ());
    turnRight(mpu6050.getAngleZ());
  }
  traversal(4900);
  stopMove();
}