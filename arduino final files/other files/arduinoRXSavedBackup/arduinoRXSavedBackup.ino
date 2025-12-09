/*
  Debashish Buragohain
  pose imitator receiver code
  RX pin = 5
  TX pin = 12
*/

#include <Servo.h>
#include <VirtualWire.h>

int count = 0;                           //the number of times the receiving cycle has taken place
const int receive_pin = 5;
byte message[VW_MAX_MESSAGE_LEN];        // a buffer to store the incoming messages
byte messageLength = VW_MAX_MESSAGE_LEN; // the size of the message

//Create the required //servo objects
//Servo //servoHeadHor;
//Servo //servoHeadVer;
//Servo //servorightShoulderY;
//Servo //servoleftShoulderY;
//Servo //servorightShoulderY;
//Servo //servoleftShoulderX;
//Servo //servorightElbow;
//Servo //servoleftElbow;
//Servo //servobase;
//Servo //servorightEar;
//Servo //servoleftEar;

void setup() {
  Serial.begin(115200);
  Serial.println("Device is ready");
  Serial.println(VW_MAX_MESSAGE_LEN);
  // Initialise the IO and ISR
  vw_set_rx_pin(receive_pin);
  vw_setup(2000);      // Bits per sec
  vw_rx_start();       // Start the receiver PLL running
  //servoHeadHor.attach(12);
  //servoHeadVer.attach(11);
  //servorightShoulderY.attach(10);
  //servoleftShoulderY.attach(9);
  //servorightShoulderY.attach(8);
  //servoleftShoulderX.attach(7);
  //servorightElbow.attach(6);
  //servoleftElbow.attach(28);
  //servobase.attach(4);
  //servorightEar.attach(3);
  //servoleftEar.attach(2);
  delay(1000); //wait for some time to charge the capacitors sufficiently
  //servoHeadHor.write(90);
  //servoHeadVer.write(90);
  //servorightShoulderY.write(90);
  //servoleftShoulderY.write(90);
  //servorightShoulderY.write(0);
  //servoleftShoulderX.write(180);
  //servorightElbow.write(90);
  //servoleftElbow.write(90);
  //servobase.write(90);
  //servorightEar.write(90);
  //servoleftEar.write(90);
  delay(500);
}

void loop() {
  if (vw_get_message(message, &messageLength))  // Non-blocking
  {
    count++;
    if (count >= 0) {
      //we are resetting this after every cycle
      count = 0;
      vw_rx_stop();
      delay(40);        //delay added to make things stable
      vw_setup(2000);   // Bits per sec
      delay(40);
      vw_rx_start();
    }
    String inputText;
    for (int i = 0; i < messageLength; i++)    {
      char c = char(message[i]);      //convert the incoming byte to character
      inputText += c;
      delay(40);                      //delay added to make things stable
    }
    inputText.remove(22, 3);
    Serial.println(inputText);
    delay(30);
    Serial.flush();
    delay(30);
    if (inputText.length() !=  26) {
      Serial.println("invalid input length");     //26 characters in the input array
      return;
    }
    else {
      int inputDegrees[13] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};     //error values
      for (int i = 0; i <= 25; i += 2) {        
        int tens = (int(inputText[i]) - 48) * 10;
        int ones = (int(inputText[i + 1]) - 48);
        inputDegrees[i / 2] = (tens + ones) * 10; //get the actual servo degrees
      }
      String displayIt;                           //display the servo degrees over the serial monitor
      for (int i = 0; i < sizeof(inputDegrees) / sizeof(inputDegrees[0]); i++) {
        displayIt += String(inputDegrees[i]) + ' ';
      }
      Serial.println(displayIt);
      //servoHeadHor.write(inputDegrees[0]);
      //servorightShoulderY.write(inputDegrees[2]);
      //servoleftShoulderY.write(inputDegrees[3]);
      //servorightShoulderY.write(inputDegrees[4]);
      //servoleftShoulderX.write(inputDegrees[5]);
      //servorightElbow.write(inputDegrees[6]);
      //servoleftElbow.write(inputDegrees[7]);
      //servobase.write(inputDegrees[8]);
    }
    delay(40);
  }
}
