#include <VirtualWire.h>

void setup()
{
  // Initialize the IO and ISR
  vw_setup(2000); // Bits per sec
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  //since the arduino is unable to handle we don't send the three static degrees
  char a[22] = {'4', '5', '1', '0', '1', '0', '1', '0', '1', '0', '1', '0', '1', '0', '1', '0', '1', '0', '1', '8', '1', '9'};
  pinMode(LED_BUILTIN, HIGH);
  delay(100);
  pinMode(LED_BUILTIN, LOW);
  delay(100);
  send(a);
  delay(300);
}

void send (char *message)
{
  vw_send((uint8_t *)message, strlen(message));
  vw_wait_tx(); // Wait until the whole message is gone
}
