/*
*  IR REMOTE CONTROL + RGB
*  by Hanie Kiani
*  https://electropeak.com/learn/
*/
#include <IRremote.h>
int RECV_PIN = A3;
IRrecv irrecv(RECV_PIN);
decode_results results;
void setup()
{
 Serial.begin(9600);
 irrecv.enableIRIn(); // Start the receiver
}
void loop() {
 if (irrecv.decode(&results)) {
    int code = results.value;
    switch(code) {
      case 16753245:                          //#1 Button Pressed
        Serial.println("Checkout->Bakery");
        break;
      case 16736925:                          //#2 Button Pressed
        Serial.println("Checkout->Produce");
        break;
      case 16769565:                          //#3 Button Pressed
        Serial.println("Checkout->Meats");
        break;
      case 16720605:                          //#4 Button Pressed
        Serial.println("Checkout->Diary");
        break;
      case 16712445:                          //#5 Button Pressed
        break;
      case 16761405:                          //#6 Button Pressed
        break;
      case 16769055:                          //#7 Button Pressed
        break;
      case 16754775:                          //#8 Button Pressed
        break;
      case 16748655:                          //#9 Button Pressed
        break;
      case 16750695:                          //#0 Button Pressed
        break;
      case 16738455:                          //* Button Pressed
        break;
      case 16756815:                          //# Button Pressed
        break;
      case 16726215:                          //OK Button Pressed
        break;
      case 16718055:                          //UP_ARROW Button Pressed
        break;
      case 16716015:                          //LEFT_ARROW Button Pressed
        break;
      case 16734885:                          //RIGHT_ARROW Button Pressed
        break;
      case 16730805:                          //DOWN_ARROW Button Pressed
        break;
    }
    irrecv.resume();                          //Recieve the next value
  }
}
