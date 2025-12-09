//Debashish Buragohain
//pose imitator
//Receiver arduino connected to the servos

#include <Servo.h>
#include <VirtualWire.h>

byte message[VW_MAX_MESSAGE_LEN]; // a buffer to store the incoming messages
byte messageLength = VW_MAX_MESSAGE_LEN; // the size of the message

//Create the required servo objects
Servo servoHeadHor;
Servo servoHeadVer;
Servo rightShoulderY;
Servo leftShoulderY;
Servo rightShoulderX;
Servo leftShoulderX;
Servo rightElbow;
Servo leftElbow;
Servo base;
Servo rightEar;
Servo leftEar;

void setup() {
  Serial.begin(9600);
  // Initialize the IO and ISR
  vw_setup(2000); // Bits per sec
  vw_rx_start(); // Start the receiver
  servoHeadHor.attach(12);
  servoHeadVer.attach(11);
  rightShoulderY.attach(10);
  leftShoulderY.attach(9);
  rightShoulderX.attach(8);
  leftShoulderX.attach(7);
  rightElbow.attach(6);
  leftElbow.attach(28);
  base.attach(4);
  rightEar.attach(3);
  leftEar.attach(2);
  pinMode(LED_BUILTIN, HIGH);
  delay(10000); //wait for some time to charge the capacitors sufficiently
  servoHeadHor.write(90);
  servoHeadVer.write(90);
  rightShoulderY.write(90);
  leftShoulderY.write(90);
  rightShoulderX.write(0);
  leftShoulderX.write(180);
  rightElbow.write(90);
  leftElbow.write(90);
  base.write(90);
  rightEar.write(90);
  leftEar.write(90);
  delay(500);
}

void loop() {
  if (vw_get_message(message, &messageLength))  // Non-blocking
  {
    String inputText;
    int servoDegrees[11];
    memset(GivArr, -1, sizeof(GivArr));         //fill the array with error values of -1
    for (int i = 0; i < messageLength; i++)
    {
      //convert the incoming byte to character
      char c = char(message[i]);
      inputText += c;
      delay(40);      //delay added to make things stable
      
    }
    //continue extracting the servo degrees until we are at the end of the string
    while (inputText.indexOf(' ') != = -1) {
      //the input servo degrees start and end with a space
      int startPosition = inputText.indexOf(' ');
      int endPosition = inputText.indexOf(' ', startPosition + 1);
      String noWithin = inputText.substring(startPosition + 1, endPosition).toInt();
      push(servoDegrees, noWithin);
      inputText.remove(0, endPosition);
    }
    //error handler
    bool anyError = false;
    for (int d = 0; d < sizeof(servoDegrees) / sizeof(servoDegrees[0]); d++) {
      if (servoDegrees[d] == -1) {
        anyError = true;
        break;
      }
    }    
    if (anyError == true) {
      //we display the error message to using the LED
      for (int m = 0; m <= 2; m++) {
        pinMode(LED_BUILTIN, LOW);
        delay(500);
        pinMode(LED_BUILTIN, HIGH);
        delay(500);
      }
      //finally turn off the LED
      pinMode(LED_BUILTIN, LOW)
    }
    //move the servos only when there is no error value in the array
    else {
      pinMode(LED_BUILTIN, HIGH);         //LED on means no error
      servoHeadHor.write(servoDegrees[0]);
      rightShoulderY.write(servoDegrees[2]);
      leftShoulderY.write(servoDegrees[3]);
      rightShoulderX.write(servoDegrees[4]);
      leftShoulderX.write(servoDegrees[5]);
      rightElbow.write(servoDegrees[6]);
      leftElbow.write(servoDegrees[7]);
      base.write(servoDegrees[8]);
    }
    delay(40);
  }
}

//the replica of the push function in JavaScript
void push(int &GivArr[], int data) {
  for (int i = 0; i < sizeof(GivArr) / sizeof(GivArr[0]); i++) {
    if (GivArr[i] == -1) break;
  }
  //clear the array if we have reached the end of it
  if (i == sizeof(GivArr) / sizeof(GivArr[0])) {
    memset(GivArr, -1, sizeof(GivArr));
    i = 0;
  }
  //finally put the data at the desired position
  GivArr[i] = data;
}
