#include <VirtualWire.h>

byte message[VW_MAX_MESSAGE_LEN]; // a buffer to store the incoming messages
byte messageLength = VW_MAX_MESSAGE_LEN; // the size of the message
const int receive_pin = 5;

int count = 0;
void setup()
{
  Serial.begin(9600);
  Serial.println("Device is ready");
  // Initialize the IO and ISR
  vw_set_rx_pin(receive_pin);
  vw_setup(2000); // Bits per sec
  vw_rx_start();  // Start the receiver
}

void loop()
{
  if (vw_get_message(message, &messageLength)) // Non-blocking
  {
    if (count >= 80) {
      //after every 80 cycles we reset the receiver
      count = 0;
      vw_rx_stop();
      delay(40);      //delay added to make things stable
      vw_setup(2000); // Bits per sec
      delay(40);
      vw_rx_start();
    }
    Serial.print("Received: ");
    for (int i = 0; i < messageLength; i++)
    {
      Serial.write(message[i]);
    }
    Serial.println();
  }
}
