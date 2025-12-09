//Arduino Pro Mini RX code
//Debasish Buragohain

#include <VirtualWire.h>
byte message[VW_MAX_MESSAGE_LEN];        // a buffer to store the incoming messages
byte messageLength = VW_MAX_MESSAGE_LEN; // the size of the message
void setup() {
  digitalWrite(8, HIGH);                 //8 is the static reset pin
  Serial.begin(9600);                    //Serial communication between the two arduinos
  pinMode(8, OUTPUT);
  vw_set_rx_pin(5);                      //5 is the static receive pin
  vw_setup(2000);      // Bits per sec
  vw_rx_start();       // Start the receiver PLL running
}

void loop() {
  if (vw_get_message(message, &messageLength))  // Non-blocking
  {
    String inputText;
    for (int i = 0; i < messageLength; i++)    {
      char c = char(message[i]);      //convert the incoming byte to character
      inputText += c;
    }
    inputText.remove(16, 3);
    Serial.print(inputText);     
    delay(60);                        //wait for 60 ms for the data to be completely transmitted
    digitalWrite(8, LOW);             // we reset the Arduino after every iteration
  }
}
