#include "Adafruit_Keypad.h"

const byte ROWS = 1; //Rows
const byte COLS = 4; //Columns
//define the symbols on the buttons of the keypads
char keys[ROWS][COLS] = {'1','2','3','4'};
byte rowPins[ROWS] = {A4};
byte colPins[COLS] = {A0,A1,A2,A3};
Adafruit_Keypad customKeypad = Adafruit_Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS);

void setup() {
  Serial.begin(9600);
  customKeypad.begin();
}

void loop() {

  int routingTable[4];
  keypadInput(routingTable);

  Serial.println("Route is: ");
  for (int i=0; i<5; i++) {
    Serial.println(routingTable[i]);
  }
  while(1){}
}

void keypadInput(int route[])
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
        route[count] = tmp;
        count++;
      }
      if (count == 4) {
        route[4] = 0;
        count = 5;
      }
    }

  } while (count < 5);

  return route;
}


