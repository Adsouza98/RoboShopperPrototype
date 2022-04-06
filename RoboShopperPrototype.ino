#include <AFMotor.h>                             //Library to Control L293D Motor Drive Shield
#include <Servo.h>                               //Library to Control Ultrasonic Sensor's Servo Motor
#include <MPU6050_tockn.h>                       //Library to Control MPU6050 Sensor
#include <Wire.h>                                //Library to Control I2C connection required to use MPU6050 Sensor
#include <Adafruit_Keypad.h>                     //Library to Control 1x4 KeyPad

//KeyPad
int userInputAry[4];
const byte ROWS = 1; //Rows
const byte COLS = 4; //Columns
//define the symbols on the buttons of the keypads
char keys[ROWS][COLS] = {'1','2','3','4'};
byte rowPins[ROWS] = {A4};
byte colPins[COLS] = {A0,A1,A2,A3};

Adafruit_Keypad customKeypad = Adafruit_Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS);

//Timings
long coToBakery_Time = 3400;                     //Checkout -> Bakery Time
long coToProduce_Time = 4900;                    //Checkout -> Produce Time
long bakeryToMeats_Time = 4800;                  //Checkout -> Meats

//Traveling Salesman Algorithm 
//Cost Matrix
float costMatrix[5][5] = { //C0,  C1,  C2,  C3,  C4
                          {0.0, 3.4, 3.9, 4.9, 8.2},   //C0
                          {3.0, 0.0, 3.95, 6.8, 4.8},  //C1
                          {3.8, 4.1, 0.0, 3.8, 4.55},  //C2
                          {4.9, 8.0, 4.6, 0.0, 3.2},   //C3
                          {7.8, 4.8, 4.55, 2.0, 0.0}   //C4
                  };
float completed[10];
float cost = 0;
int costMatrix_Size = 5;

//Refactored Tempoary Array
float refactoredAry[5][5];

//Orientation
char orientation = 'S';
int location = 0;
const int checkoutLocation = 0;
const int bakeryLocation = 1;
const int dairyLocation = 2;
const int produceLocation = 3;
const int meatsLocation = 4;


//Optimized Routing Table Array
int routingTable[10];
int routingTable_Index = 0;

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

  //KeyPad Setup
  customKeypad.begin(); 

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
  //Get User Input (4 inputs)
  Serial.println("Getting User Input");
  keypadInput(userInputAry);

  Serial.print("UserInputAry => ");
  for (int i=0; i<5; i++) {
    Serial.print(userInputAry[i]);
    Serial.print(" ");
  }

  //Refactor the Cost Matrix to Accomodate Only Desrired Destinations
  Serial.println("");
  Serial.println("Getting Refactored Array");
  costTableRefactor((float*)refactoredAry, userInputAry);
  Serial.println("Refactored Array => ");
  printArray((float*)refactoredAry);

  //Find Shortest Path Using the Traveling Salesman Algorithm (Dynamic Programing)
  minCost((float*)refactoredAry, 0);
  Serial.println("\nMinimum Cost is: ");
  Serial.println(cost);
  
  //Optimized Shortest Route
  Serial.println("The Optimized Route2 is: ");
  for (int i=0; i<routingTable_Index; i++) {
    Serial.print(routingTable[i]);
    Serial.print("--->");
  }


  servoUltraSonic.write(straight);                     //Set the Servo to Look Straight Ahead (90deg)
  delay(750);                                          //Wait for 750ms for Servo to move

  for (int i=0; i<routingTable_Index; i++) {
    //Checkout -> X
    if ((routingTable[i] == bakeryLocation) && (location == checkoutLocation)) {
      orientation = checkoutToBakery();
    }
    else if ((routingTable[i] == dairyLocation) && (location == checkoutLocation)) {
      orientation = checkoutToDairy();
    }
    else if ((routingTable[i] == produceLocation) && (location == checkoutLocation)) {
      orientation = checkoutToProduce();
    }
    else if ((routingTable[i] == meatsLocation) && (location == checkoutLocation)) {
      orientation = checkoutToMeats();
    }

    //Bakery -> X
    else if (routingTable[i] == checkoutLocation && (location == bakeryLocation)) {
      bakeryToCheckout(orientation);
    }
    else if ((routingTable[i] == dairyLocation) && (location == bakeryLocation)) {
      orientation = bakeryToDairy(orientation);
    }
    else if ((routingTable[i] == produceLocation) && (location == bakeryLocation)) {
      orientation = bakeryToProduce(orientation);
    }
    else if ((routingTable[i] == meatsLocation) && (location == bakeryLocation)) {
      orientation = bakeryToMeats(orientation);
    }

    //Dairy -> X
    else if ((routingTable[i] == checkoutLocation) && (location == dairyLocation)) {
      dairyToCheckout(orientation);
    }
    else if ((routingTable[i] == bakeryLocation) && (location == dairyLocation)) {
      orientation = dairyToBakery(orientation);
    }
    else if ((routingTable[i] == produceLocation) && (location == dairyLocation)) {
      orientation = dairyToProduce(orientation);
    }
    else if ((routingTable[i] == meatsLocation) && (location == dairyLocation)) {
      orientation = dairyToMeats(orientation);
    }

    //Produce -> X
    else if ((routingTable[i] == checkoutLocation) && (location == produceLocation)) {
      produceToCheckout(orientation);
    }
    else if ((routingTable[i] == bakeryLocation) && (location == produceLocation)) {
      orientation = produceToBakery(orientation);
    }
    else if ((routingTable[i] == dairyLocation) && (location == produceLocation)) {
      orientation = produceToDairy(orientation);
    }
    else if ((routingTable[i] == meatsLocation) && (location == produceLocation)) {
      orientation = produceToMeats(orientation);
    }

    //Meats -> X
    else if ((routingTable[i] == checkoutLocation) && (location == meatsLocation)) {
      meatsToCheckout(orientation);
    }
    else if ((routingTable[i] == bakeryLocation) && (location == meatsLocation)) {
      orientation = meatsToBakery(orientation);
    }
    else if ((routingTable[i] == dairyLocation) && (location == meatsLocation)) {
      orientation = meatsToDairy(orientation);
    }
    else if ((routingTable[i] == produceLocation) && (location == meatsLocation)) {
      orientation = meatsToProduce(orientation);
    }
    keypadContinue();

  }

  while(true){}


}

void keypadInput(int userInput[])
{
  int count = 0;
  int tmp = 0;
  do {
    customKeypad.tick();
    while(customKeypad.available()){
      keypadEvent e = customKeypad.read();

      if (e.bit.EVENT == KEY_JUST_PRESSED) {
        Serial.print((char)e.bit.KEY);
        Serial.println(" pressed");
        tmp = (char)e.bit.KEY - '0';
        userInput[count] = tmp;
        count++;
      }
      if (count == 4) {
        userInput[4] = 0;
        count = 5;
      }
    }
  } while (count < 5);
}

void keypadContinue()
{
  int count = 0;
  do {
    customKeypad.tick();
    while(customKeypad.available()){
      keypadEvent e = customKeypad.read();

      if (e.bit.EVENT == KEY_JUST_PRESSED) {
        Serial.print((char)e.bit.KEY);
        Serial.println(" pressed");
        count++;
      }
    }
  } while (count < 1);
}

//########################### TRAVELING SALESMAN ALGORITHM #########################################
void minCost(float* a, int checkpoint)
{
  int i;
  int nextCheckpoint;
  completed[checkpoint] = 1;

  Serial.print("Adding Checkpoint: ");
  Serial.println(checkpoint);
  routingTable[routingTable_Index] = checkpoint;
  routingTable_Index++;

  nextCheckpoint = least(a, checkpoint);

  if (nextCheckpoint == 999) {
    nextCheckpoint = 0;
    cost += *((a+checkpoint*costMatrix_Size)+nextCheckpoint);
    Serial.print("Adding Checkpoint: ");
    Serial.println(nextCheckpoint);
    routingTable[routingTable_Index] = nextCheckpoint;
    routingTable_Index++;
    Serial.println("Leaving minCost Function");
    return;
  }
  minCost(a, nextCheckpoint);
}

int least(float* a, int checkpoint)
{
  int nextCheckpoint = 999;
  int i;
  float min = 999, kmin;

  for (i=0;i<costMatrix_Size;i++) {
    if ( ( *((a+checkpoint*costMatrix_Size)+i) != 0) && (completed[i] == 0) ) {
      if (*((a+checkpoint*costMatrix_Size)+i) + *((a+i*costMatrix_Size)+checkpoint) < min) {
        min = *((a+i*costMatrix_Size)+0) + *((a+checkpoint*costMatrix_Size)+i);
        kmin = *((a+checkpoint*costMatrix_Size)+i);
        nextCheckpoint = i;
      }
    }
  }
  if (min != 999) {
    cost += kmin;
  }

  return nextCheckpoint;
}
//######################## END OF TRAVELING SALESMAN ALGORITHM #####################################

void costTableRefactor(float* tmp, int userInput[]) //Function to Remove checkpoints that are not on the Customer's shopping destinations list
{
  int a = 1;
  bool a_check = false;
  int b = 2;
  bool b_check = false;
  int c = 3;
  bool c_check = false;
  int d = 4;
  bool d_check = false;
  
  for (int k=0; k<costMatrix_Size; k++) {
    if (userInput[k] == a) {
      Serial.println("a=1 is there");
      a_check = true;
    }
    if (userInput[k] == b) {
      Serial.println("b=2 is there");
      b_check = true;
    }
    if (userInput[k] == c) {
      Serial.println("c=3 is there");
      c_check = true;
    }
    if (userInput[k] == d) {
      Serial.println("d=4 is there");
      d_check = true;
    }
  }

  if (a_check == true) {
    a = -1;
  }
  if (b_check == true) {
    b = -1;
  }
  if (c_check == true) {
    c = -1;
  }
  if (d_check == true) {
    d = -1;
  }

  for (int i=0; i<costMatrix_Size; i++) {
    for (int j=0; j<costMatrix_Size; j++) {
      if (i == a || i == b || i == c || i == d || j == a || j == b || j == c || j == d) {
        *((tmp+i*costMatrix_Size)+j) = 0.0;      // Replace the Columns and Rows of not needed destinations with 0.0 cost
      } else {
        *((tmp+i*costMatrix_Size)+j) = costMatrix[i][j];
      }

    }
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
  location = bakeryLocation;
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

  location = produceLocation;
  return 'S';
}

char checkoutToMeats()                           //Checkout -> Meats
{
  checkoutToBakery();
  bakeryToMeats('E');

  location = meatsLocation;
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
  
  location = dairyLocation;
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

  location = dairyLocation;
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

  location = meatsLocation;
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

  location = produceLocation;
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
  location = checkoutLocation;
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

  location = produceLocation;
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

  location = bakeryLocation;
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

  location = dairyLocation;
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
  location = checkoutLocation;
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

  location = bakeryLocation;
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

  location = meatsLocation;
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

  location = produceLocation;
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

  location = checkoutLocation;
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

  location = meatsLocation;
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

  location = bakeryLocation;
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

  location = dairyLocation;
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

  location = checkoutLocation;
}

//############################# DEBUGGING FUNCTIONS ###################################
void printArray(float* a)                        //Function to Print Cost Matrix Array (For Debugging Purposes)
{
  for (int i=0; i<costMatrix_Size; i++) {
    for (int j=0; j<costMatrix_Size; j++) {
      Serial.print(*((a+i*costMatrix_Size)+j));
      Serial.print(" ");
    }
    Serial.println("");
  }
}

// void accelerate()                                //Function to accelerate the motors from 0 to full speed
// {
//   for (int i=0; i<motorSpeed; i++)                //Loop from 0 to full speed
//   {
//     rightBack.setSpeed(i);                        //Set the motors to the current loop speed
//     rightFront.setSpeed(i);
//     leftFront.setSpeed(i+motorOffset);
//     leftBack.setSpeed(i+motorOffset);
//     delay(10);
//   }
// }

// void decelerate()                                //Function to decelerate the motors from full speed to zero
// {
//   for (int i=motorSpeed; i!=0; i--)               //Loop from full speed to 0
//   {
//     rightBack.setSpeed(i);                        //Set the motors to the current loop speed
//     rightFront.setSpeed(i);
//     leftFront.setSpeed(i+motorOffset);
//     leftBack.setSpeed(i+motorOffset);
//     delay(10);
//   }
// }
