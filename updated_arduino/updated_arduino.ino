// Pose Imitator Arduino Mega Code
// Debasish Buragohain
#include <Servo.h>

long int lastTimeMoved = 0;
const int gapInEarMovements = 6000;

Servo servoHeadHor;
Servo servoHeadVer;
Servo servorightShoulderY;
Servo servoleftShoulderY;
Servo servorightShoulderX;
Servo servoleftShoulderX;
Servo servorightElbow;
Servo servoleftElbow;
Servo servobase;
Servo servorightEar;
Servo servoleftEar;

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
  Serial.print("Servo controller ready.");
  Serial.println("\n");
  pinMode(LED_BUILTIN, OUTPUT);

  servoleftElbow.attach(48);
  servoleftShoulderY.attach(46);
  servoleftShoulderX.attach(44);
  servorightElbow.attach(42);
  servorightShoulderY.attach(40);
  servorightShoulderX.attach(38);
  servobase.attach(36);
  servoHeadVer.attach(34);
  servoHeadHor.attach(32);
  servoleftEar.attach(52);
  servorightEar.attach(50);

  delay(1000);
  servoHeadHor.write(90);
  servoHeadVer.write(90);
  servorightShoulderY.write(90);
  servoleftShoulderY.write(90);
  servorightShoulderX.write(0);
  servoleftShoulderX.write(180);
  servorightElbow.write(90);
  servoleftElbow.write(90);
  servobase.write(90);
  servorightEar.write(90);
  servoleftEar.write(90);
  delay(500);
}

void loop() {
  if (Serial3.available()) {
    String inputText;
    while (Serial3.available()) {
      char c = Serial3.read();
      delay(10);
      inputText += c;
    }
    
    // --- FIX 1: Remove garbage ONLY after the 18th character (9 servos * 2 digits) ---
    // If your input has newline characters (\r\n), this cleans them up without cutting data.
    if (inputText.length() > 18) {
       inputText.remove(18); 
    }

    Serial.println("");
    Serial.print("Raw Input: ");
    Serial.println(inputText);

    // --- FIX 2: Check for length 18 (9 Servos) ---
    if (inputText.length() != 18) {
      Serial.print("Invalid length: ");
      Serial.print(inputText.length());
      for (int dh = 0; dh < 2; dh++) {
        digitalWrite(LED_BUILTIN, HIGH); delay(100);
        digitalWrite(LED_BUILTIN, LOW); delay(100);
      }
      return;
    }
    else {
      // --- FIX 3: Increase Array Size to 9 ---
      int inputDegrees[9] = { -1, -1, -1, -1, -1, -1, -1, -1, -1};
      
      // --- FIX 4: Increase loop limit to read all 18 chars ---
      for (int i = 0; i <= 17; i += 2) {
        int tens = (int(inputText[i]) - 48) * 10;
        int ones = (int(inputText[i + 1]) - 48);
        inputDegrees[i / 2] = (tens + ones) * 10; 
      }

      String displayIt;
      for (int i = 0; i < 9; i++) {
        if (inputDegrees[i] == -1) {
          Serial.print("Error: input degree array contains an error value.");
          return;
        }
        displayIt += String(inputDegrees[i]) + ' ';
      }
      Serial.print("Parsed Angles: ");
      Serial.println(displayIt);

      if (millis() - lastTimeMoved >= gapInEarMovements) moveEarServos();
      
      // Map the 9 values to your servos
      if (servoHeadHor.read() != inputDegrees[0]) { servoHeadHor.write(inputDegrees[0]); delay(150); }
      // Note: You skipped index 1 in your Node logic previously, ensure this mapping matches your new 9-value logic
      if (servorightShoulderY.read() != inputDegrees[1]) { servorightShoulderY.write(inputDegrees[1]); delay(150); }
      if (servoleftShoulderY.read() != inputDegrees[2]) { servoleftShoulderY.write(inputDegrees[2]); delay(150); }
      if (servorightShoulderX.read() != inputDegrees[3]) { servorightShoulderX.write(inputDegrees[3]); delay(150); }
      if (servoleftShoulderX.read() != inputDegrees[4]) { servoleftShoulderX.write(inputDegrees[4]); delay(150); }
      if (servorightElbow.read() != inputDegrees[5]) { servorightElbow.write(inputDegrees[5]); delay(150); }
      if (servoleftElbow.read() != inputDegrees[6]) { servoleftElbow.write(inputDegrees[6]); delay(150); }
      if (servobase.read() != inputDegrees[7]) { servobase.write(inputDegrees[7]); delay(150); }
      
      // --- FIX 5: Handle the 9th Value (Index 8) ---
      // You need to decide which physical servo 'inputDegrees[8]' controls.
      // For example, if it's the Head Vertical:
      // if (servoHeadVer.read() != inputDegrees[8]) { servoHeadVer.write(inputDegrees[8]); delay(150); }
    }
    delay(40);
  }
}

void moveEarServos() {
  lastTimeMoved = millis();
  int servoTarget;
  switch (servorightEar.read()) {
    case 0: servoTarget = 90; break;
    case 90: int ranNum = round(random(0, 1)); servoTarget = (ranNum == 0) ? 0 : 180; break;
    case 180: servoTarget = 90; break;
  }
  servorightEar.write(servoTarget); delay(300);
  servoleftEar.write(servoTarget); delay(300);
}
