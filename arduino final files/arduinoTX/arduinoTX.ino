//pose imitator RF transmitter connected to the laptop
//Debashish Buragohain
//RX pin = 5
//TX pin = 12
//test command:  0909090909090909

#include <VirtualWire.h>
const int transmit_pin = 12;
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  vw_set_tx_pin(transmit_pin);
  vw_setup(2000); // Bits per sec
}

void loop()
{
  if (Serial.available()) {
    String inputTextString;                   //we firstly push this here then include it in the character array
    while (Serial.available()) {              //conduct a serial read of the data
      char c = Serial.read();
      delay(10);                              //delay added to make things stable
      inputTextString += c;
    }
    inputTextString.remove(16, 1);
    Serial.print(inputTextString);
    char inputText[16];                                             //one char extra for the non-graphical character
    for (int i = 0; i <= 15; i++) inputText[i] = inputTextString[i];
    String displayIt;                           //display the servo degrees over the serial monitor
    for (int i = 0; i < sizeof(inputText) / sizeof(inputText[0]); i++) {
      displayIt += String(inputText[i]) + ' ';
    }
    send(inputText);
    delay(30);                                                     //delays added to make things stable
  }
}

void send (char *message)
{
  vw_send((uint8_t *)message, strlen(message));
  vw_wait_tx(); // Wait until the whole message is gone
}
