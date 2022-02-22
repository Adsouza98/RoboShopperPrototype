#include <AFMotor.h>                             //Library to Control L293D Motor Drive Shield
#include <Servo.h>                               //Library to Control Ultrasonic Sensor's Servo Motor
#include <MPU6050_tockn.h>                       //Library to Control MPU6050 Sensor
#include <Wire.h>                                //Library to Control I2C connection required to use MPU6050 Sensor

//Timings
long coToBakery_Time = 3400;                     //Checkout -> Bakery Time
long coToProduce_Time = 4900;                    //Checkout -> Produce Time
long bakeryToMeats_Time = 4800;                  //Checkout -> Meats

long meatsToProduce_Time = 2000;                 //Meats -> Produce Time

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
  // Serial.println("Checkout->Bakery");
  //checkoutToBakery();

  // Bakery -> Dairy
  // bakeryToDairy();

  // Bakery -> Meats
  // bakeryToMeats();

  // Meats -> Produce (Needs to be verified)
  // meatsToProduce();

  // Checkout -> Produce
  // Serial.println("Checkout->Produce");
  // checkoutToProduce();

  // Checkout -> Meats
  // Serial.println("Checkout->Meats");
  // checkoutToMeats();

  // Checkout -> Dairy
  // Serial.println("Checkout->Diary");
  checkoutToDairy();

  // Dairy -> Bakery
  dairyToBakery();


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

void turnLeft(float currentAngle)                      //Set motors to turn left for the specified duration then stop
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

void turnRight(long currentAngle)                     //Set motors to turn right for the specified duration then stop
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
  } while(mpu6050.getAngleZ() > currentAngle - 155);
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

void checkoutToBakery()                          //Checkout -> Bakery
{
  traversal(coToBakery_Time);
  stopMove();

  lookLeft();

  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnLeft(mpu6050.getAngleZ());
}

void checkoutToProduce()                         //Checkout -> Produce
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
}

void checkoutToMeats()                           //Checkout -> Meats
{
  checkoutToBakery();
  traversal(bakeryToMeats_Time);
  stopMove();
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  lookLeft();
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnLeft(mpu6050.getAngleZ());
}

void checkoutToDairy()                           //Checkout -> Dairy
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
}

void bakeryToDairy()                             //Bakery -> Dairy
{
  traversal(2300);
  stopMove();

  lookLeft();
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnLeft(mpu6050.getAngleZ());

  traversal(1600);
  stopMove();
}

void bakeryToMeats()                             //Bakery -> Meats
{
  traversal(bakeryToMeats_Time);
  stopMove();

  lookLeft();
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnLeft(mpu6050.getAngleZ());
}

void meatsToProduce()                            //Meats -> Produce
{
  traversal(meatsToProduce_Time);
  stopMove();

  lookLeft();
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnLeft(mpu6050.getAngleZ());

}

void dairyToBakery()                             //Dairy -> Bakery
{
  traversal(2000);
  stopMove();
  lookRight();
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnRight(mpu6050.getAngleZ());

  traversal(2200);
  stopMove();
  lookRight();
  mpu6050.update();
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  turnRight(mpu6050.getAngleZ());
}