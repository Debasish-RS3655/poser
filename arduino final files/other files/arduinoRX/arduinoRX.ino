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

//Create the required servo objects
Servo servoHeadHor;
Servo servoHeadVer;
Servo servorightShoulderY;
Servo servoleftShoulderY;
Servo servorightShoulderY;
Servo servoleftShoulderX;
Servo servorightElbow;
Servo servoleftElbow;
Servo servobase;
Servo servorightEar;
Servo servoleftEar;

void setup() {
  Serial.begin(115200);
  Serial.println("Device is ready");
  // Initialise the IO and ISR
  vw_set_rx_pin(receive_pin);
  vw_setup(2000);      // Bits per sec
  vw_rx_start();       // Start the receiver PLL running
  servoHeadHor.attach(12);
  servoHeadVer.attach(11);
  servorightShoulderY.attach(10);
  servoleftShoulderY.attach(9);
  servorightShoulderY.attach(8);
  servoleftShoulderX.attach(7);
  servorightElbow.attach(6);
  servoleftElbow.attach(28);
  servobase.attach(4);
  servorightEar.attach(3);
  servoleftEar.attach(2);
  
  pinMode(LED_BUILTIN, HIGH);
  delay(1000); //wait for some time to charge the capacitors sufficiently

  servoHeadHor.write(90);
  servoHeadVer.write(90);
  servorightShoulderY.write(90);
  servoleftShoulderY.write(90);
  servorightShoulderY.write(0);
  servoleftShoulderX.write(180);
  servorightElbow.write(90);
  servoleftElbow.write(90);
  servobase.write(90);
  servorightEar.write(90);
  servoleftEar.write(90);
  delay(500);
}

void loop() {
  if (vw_get_message(message, &messageLength))  // Non-blocking
  {
    count++;
    if (count >= 80) {
      //after every 80 cycles we reset the receiver
      count = 0;
      vw_rx_stop();
      delay(40);    //delay added to make things stable
      vw_setup(2000);   // Bits per sec
      delay(40);
      vw_rx_start();
    }
    //receiving indication
    pinMode(LED_BUILTIN, HIGH);
    delay(100);
    pinMode(LED_BUILTIN, LOW);
    String inputText;
    int inputDegrees[11];
    //fill with error value of 1
    for (int i = 0; i < messageLength; i++)    {
      //convert the incoming byte to character
      char c = char(message[i]);
      inputText += c;
      delay(40);      //delay added to make things stable
    }
    Serial.println(inputText);
    delay(100);
    Serial.flush();
    delay(100);
    //continue extracting the servo degrees until we are at the end of the string
    while (inputText.indexOf('_') != -1) {
      //the input servo degrees start and end with a underscore
      int startPosition = inputText.indexOf('_');
      int endPosition = inputText.indexOf('_', startPosition + 1);
      int i;
      int noWithin = inputText.substring(startPosition + 1, endPosition).toInt();
      //the replica of the push function in Arduino
      for (i = 0; i < sizeof(inputDegrees) / sizeof(inputDegrees[0]); i++) {
        delay(20);
        if (inputDegrees[i] == '\0') break;
      }
      //clear the array if we have reached the end of it
      if (i == sizeof(inputDegrees) / sizeof(inputDegrees[0])) {
        for (int as = 0; as < sizeof(inputDegrees) / sizeof(inputDegrees[0]); as++)inputDegrees[as] = '\0';
        i = 0;
      }
      //finally put the data at the desired position
      inputDegrees[i] = noWithin;
      inputText.remove(0, endPosition);
    }
    //error handler
    bool anyError = false;
    for (int d = 0; d < sizeof(inputDegrees) / sizeof(inputDegrees[0]); d++) {
      if (inputDegrees[d] == '\0') {
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
      pinMode(LED_BUILTIN, LOW);
    }
    //move the servos only when there is no error value in the array
    else {
      pinMode(LED_BUILTIN, HIGH);         //LED on means no error
      Serial.print(" head horizontal servo: " + inputDegrees[0]);
      Serial.print(" right shoulder vertical servo: " + inputDegrees[2]);
      Serial.print(" left shoulder vertical servo: " + inputDegrees[3]);
      Serial.print(" right shoulder horizontal servo: " + inputDegrees[4]);
      Serial.print(" left shoulder horizontal servo: " + inputDegrees[5]);
      Serial.print(" right elbow servo: " + inputDegrees[6]);
      Serial.print(" left elbow servo: " + inputDegrees[7]);
      Serial.print(" base servo: " + inputDegrees[8]);
      servoHeadHor.write(inputDegrees[0]);
      servorightShoulderY.write(inputDegrees[2]);
      servoleftShoulderY.write(inputDegrees[3]);
      servorightShoulderY.write(inputDegrees[4]);
      servoleftShoulderX.write(inputDegrees[5]);
      servorightElbow.write(inputDegrees[6]);
      servoleftElbow.write(inputDegrees[7]);
      servobase.write(inputDegrees[8]);
    }
    delay(40);
  }
}
