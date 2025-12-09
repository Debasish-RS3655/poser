/*
  Arduino Code for communicating with the matrix and controlling the servo motors
  Robotic Intuition Operator Body OS
  Debashish Buragohain
*/

#include <Servo.h>
//Create the required servo objects
Servo servoHeadHor;
Servo servoHeadVer;
Servo rightShoulderY;
Servo leftShoulderY;
Servo rightShoulderX;
Servo leftShoulderX;
Servo rightWrist;
Servo leftWrist;
Servo base;
Servo rightEar;
Servo leftEar;
SoftwareSerial BT (51, 52);       //RX TX
//RX supported pins on Mega
//10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69
//initiate the Bluetooth and servos
//in the current version of the program, there are two arrays, one containing the start and the other containing the ending positions
int servoDegreesStart[11];
int servoDegreesEnd[11];
int givenDelayTime;
//the time given to wait in the current position
const int tempSensor = A0;    //the pin where the lm35 sensor is connected
const int BTstate = 14;       //the connected state pin of bluetooth
const int LED = 15;           //the status LED
int distLeft;
int distRight;
int prevDistLeft;
int prevDistRight;
boolean LeftDetect = false;
boolean RightDetect = false;
String BTData;  //contains the string from the serial buffer
Ultrasonic ultrasonicRight(22, 27); //connected to the Trigger and Echo pins respectively
Ultrasonic ultrasonicLeft(26, 23);  //connected to the Trigger and Echo pins respectively
int x;
int y;
int prevX;
int prevY;
int currentServoPos = -1;           //set to an undefined value
bool modifiedValues[10] = {false, false, false, false, false, false, false, false, false, false};
int modifiedValuesLength = sizeof(modifiedValues) / sizeof(boolean);
//if we have modified anything in the current iteration
//the default set of values
const int maxDistDiffDef = 20;
const int initialDelayDef = 10000;
const int maxDefinedDistDef = 60;             //the maximum distance allowed for reading so that changes too far away are not responded to
const int minLeftDegReflexDef = 90;
const int minRightDegReflexDef = 90;
const float normalTempDef = 97.0;    //we need to set the right values these are just arbitrary ones
const float slightHotDef = 99.0;
const float mediumHotDef = 101.0;
const float justHotDef = 103.0;
const float veryHotDef = 104.0;
//the main parameters to be set
int maxDistDiff;
int initialDelay;     //the wait for the capacitors to charge up in seconds
int maxDefinedDist;
int minLeftDegReflex;
int minRightDegReflex;
float normalTemp;
float slightHot;
float mediumHot;
float justHot;
float veryHot;

void initModules() {
  delay(initialDelay * 1000);
  //wait for some time to charge the capacitors sufficiently
  servoHeadHor.write(90);
  servoHeadVer.write(90);
  rightShoulderY.write(90);
  leftShoulderY.write(90);
  rightShoulderX.write(0);
  leftShoulderX.write(180);
  rightWrist.write(90);
  leftWrist.write(90);
  base.write(90);
  rightEar.write(90);
  leftEar.write(90);
  delay(500);
  //delay added to make things stable
  distLeft = ultrasonicLeft.Ranging(CM);
  distRight = ultrasonicRight.Ranging(CM);
  prevDistLeft = distLeft;
  prevDistRight = distRight;
}

//function to reset all the servo variables
void resetTheVariables() {
  //reset the variables after applying the data given
  memset(servoDegreesStart, 1001, sizeof(servoDegreesStart));
  memset(servoDegreesEnd, 1001, sizeof(servoDegreesEnd));
  givenDelayTime = -1;
}

bool initParameters() {
  //setting the parameters
  //if any EEPROM address is 0 then the main mode won't be started
  for (int i = 0; i <= 9; i++) {
    if (EEPROM.read(i) == 0) {
      return false;
    }
  }
  int address = 0;
  maxDistDiff = (int) map(EEPROM.read(address), 0, 254, 1, 1000);
  initialDelay = (int) map(EEPROM.read(++address), 0, 254, 1, 1000);
  maxDefinedDist = (int) map(EEPROM.read(++address), 0, 254, 1, 1000);
  minLeftDegReflex = (int) map(EEPROM.read(++address), 0, 254, 0, 180);
  minRightDegReflex = (int) map(EEPROM.read(++address), 0, 254, 0, 180);
  normalTemp = (float) map(EEPROM.read(++address), 0, 254, -40, 220);
  slightHot = (float) map(EEPROM.read(++address), 0, 254, -40, 220);
  mediumHot = (float) map(EEPROM.read(++address), 0, 254, -40, 220);
  justHot = (float) map(EEPROM.read(++address), 0, 254, -40, 220);
  veryHot = (float) map(EEPROM.read(++address), 0, 254, -40, 220);
  return true;
}

void btInterface() {
  //the bluetooth interface of the robot body
  BT.println("Robotic Intuition Operator Body frontend.");
  BT.println("Developed by Debashish Buragohain");
  BT.println("This is the command line for the robot body.");
  bool repeatThis = true;
  while (repeatThis == true) {
    if (BT.available() > 0) {
      while (BT.available()) {
        char c = BT.read();
        BTData += c;
      }
      //the list command lists all the available params
      if (BTData.indexOf("list") != -1) {
        BT.println("maxDistDiff: " + (String) maxDistDiff + "/t (int)the max diff in cm to be assumed reflex.");
        BT.println("inititalDelay: " + (String) initialDelay + "/t (int) delay in sec for the capacitors to charge up.");
        BT.println("maxDefinedDist: " + (String) maxDefinedDist + "/t (int)the max distance upto which SONAR is done.");
        BT.println("minLeftDegReflex: " + (String)  minLeftDegReflex + "/t (int) the min left ear degrees for reflex to work.");
        BT.println("minRightDegReflex: " + (String)  minRightDegReflex + "/t (int) the min right ear degrees for reflex to work.");
        BT.println("normalTemp: " + (String)  normalTemp + "/t (float) the value of the normal temperature.");
        BT.println("slightHot: " + (String)  slightHot + "/t (float) the value of slight hot temperature.");
        BT.println("mediumHot: " + (String)  mediumHot + "/t (float) the value of medium hot temperature.");
        BT.println("justHot: " + (String)  justHot + "/t (float) the value of just hot temperature.");
        BT.println("veryHot: " + (String)  veryHot + "/t (float) the value of very hot temperature.");
      }
      //updation of the variables
      else if (BTData.indexOf("modify") != -1) {
        //in this newer version we need an equal before the value
        if (BTData.indexOf("=") != -1) {
          if (BTData.indexOf("maxDistDiff") != -1) {
            String temp = BTData.substring(BTData.indexOf("=") + 1);
            //if the next char after = is a space, we ignore the space
            int value = isSpace(temp.charAt(0)) ? temp.substring(1).toInt() : temp.toInt();
            if (value > 0 && value <= 1000)
              maxDistDiff = value, BT.println("maxDistDiff updated. New value: " + (String)  maxDistDiff), modifiedValues[0] = true;
            else BT.println("Syntax Error: value exceeded limits 1-1000 cm");
          }
          else if (BTData.indexOf("initialDelay") != -1) {
            String temp = BTData.substring(BTData.indexOf("=") + 1);
            int value = isSpace(temp.charAt(0)) ? temp.substring(1).toInt() : temp.toInt();
            if (value > 0 && value <= 1000)
              initialDelay = value, BT.println("initialDelay updated. New value: " + (String)  initialDelay), modifiedValues[1] = true;
            else BT.println("Syntax Error: value exceeded limits 1-1000 sec");
          }
          else if (BTData.indexOf("maxDefinedDist") != -1) {
            String temp = BTData.substring(BTData.indexOf("=") + 1);
            int value = isSpace(temp.charAt(0)) ? temp.substring(1).toInt() : temp.toInt();
            if (value > 0 && value <= 1000)
              maxDefinedDist = value, BT.println("maxDefinedDist updated. New value: " + (String)  maxDefinedDist), modifiedValues[2] = true;
            else BT.println("Syntax Error: value exceeded limits 1-1000 cm");
          }
          else if (BTData.indexOf("minLeftDegReflex") != -1) {
            String temp = BTData.substring(BTData.indexOf("=") + 1);
            int value = isSpace(temp.charAt(0)) ? temp.substring(1).toInt() : temp.toInt();
            if (value >= 0 && value <= 180)
              minLeftDegReflex = value, BT.println("minLeftDegReflex updated. New value: " + (String)  minLeftDegReflex), modifiedValues[3] = true;
            else BT.println("Syntax Error: value exceeded limits 0-180 degrees");
          }
          else if (BTData.indexOf("minRightDegReflex") != -1) {
            String temp = BTData.substring(BTData.indexOf("=") + 1);
            int value = isSpace(temp.charAt(0)) ? temp.substring(1).toInt() : temp.toInt();
            if (value >= 0 && value <= 180)
              minRightDegReflex = value, BT.println("minRightDegReflex updated. New value: " + (String)  minRightDegReflex), modifiedValues[4] = true;
            else BT.println("Syntax Error: value exceeded limits 0-180 degrees");
          }
          else if (BTData.indexOf("normalTemp") != -1) {
            String temp = BTData.substring(BTData.indexOf("=") + 1);
            float value = isSpace(temp.charAt(0)) ? temp.substring(1).toFloat() : temp.toFloat();
            if (value >= -40 && value <= 220)
              normalTemp = value, BT.println("normalTemp updated. New value: " + (String)  normalTemp), modifiedValues[5] = true;
            else BT.println("Syntax Error: value exceeded limits -40 to 220 degree Fahrenheit.") ;
          }
          else if (BTData.indexOf("slightHot") != -1) {
            String temp = BTData.substring(BTData.indexOf("=") + 1);
            float value = isSpace(temp.charAt(0)) ? temp.substring(1).toFloat() : temp.toFloat();
            if (value >= -40 && value <= 220)
              slightHot = value, BT.println("slightHot updated. New value: " + (String)  slightHot), modifiedValues[6] = true;
            else BT.println("Syntax Error: value exceeded limits -40 to 220 degree Fahrenheit.") ;
          }
          else if (BTData.indexOf("mediumHot") != -1) {
            String temp = BTData.substring(BTData.indexOf("=") + 1);
            float value = isSpace(temp.charAt(0)) ? temp.substring(1).toFloat() : temp.toFloat();
            if (value >= -40 && value <= 220)
              mediumHot = value, BT.println("mediumHot updated. New value: " + (String)  mediumHot), modifiedValues[7] = true;
            else BT.println("Syntax Error: value exceeded limits -40 to 220 degree Fahrenheit.") ;
          }
          else if (BTData.indexOf("justHot") != -1) {
            String temp = BTData.substring(BTData.indexOf("=") + 1);
            float value = isSpace(temp.charAt(0)) ? temp.substring(1).toFloat() : temp.toFloat();
            if (value >= -40 && value <= 220)
              justHot = value, BT.println("justHot updated. New value: " + (String)  justHot), modifiedValues[8] = true;
            else BT.println("Syntax Error: value exceeded limits -40 to 220 degree Fahrenheit.") ;
          }
          else if (BTData.indexOf("veryHot") != -1) {
            String temp = BTData.substring(BTData.indexOf("=") + 1);
            float value = isSpace(temp.charAt(0)) ? temp.substring(1).toFloat() : temp.toFloat();
            if (value >= -40 && value <= 220) {
              veryHot = value, BT.println("veryHot updated. New value: " + (String)  veryHot), modifiedValues[9] = true;
            }
            else BT.println("Syntax Error: value exceeded limits -40 to 220 degree Fahrenheit.") ;
          }
          else BT.println("Syntax Error: No parameter provided for modification.");
        }
        else BT.println("Syntax Error: '=' sign not found before value.");
      }
      //reset the parameters
      else if (BTData.indexOf("reset") != -1) {
        if (BTData.indexOf("maxDistDiff") != -1) {
          maxDistDiff = maxDistDiffDef;
          modifiedValues[0] = false;
          BT.println("maxDistDiff has been reset.");
        }
        else if (BTData.indexOf("initialDelay") != -1) {
          initialDelay = initialDelayDef;
          modifiedValues[1] = false;
          BT.println("initialDelay has been reset.");
        }
        else if (BTData.indexOf("maxDefinedDist") != -1) {
          maxDefinedDist = maxDefinedDistDef;
          modifiedValues[2] = false;
          BT.println("maxDefinedDist has been reset.");
        }
        else if (BTData.indexOf("minLeftDegReflex") != -1) {
          minLeftDegReflex = minLeftDegReflexDef;
          modifiedValues[3] = false;
          BT.println("minLeftDegReflex has been reset.");
        }
        else if (BTData.indexOf("minRightDegReflex") != -1) {
          minRightDegReflex = minRightDegReflexDef;
          modifiedValues[4] = false;
          BT.println("minRightDegReflex has been reset.");
        }
        else if (BTData.indexOf("normalTemp") != -1) {
          normalTemp = normalTempDef;
          modifiedValues[5] = false;
          BT.println("normalTemp has been reset.");
        }
        else if (BTData.indexOf("slightHot") != -1) {
          slightHot = slightHotDef;
          modifiedValues[6] = false;
          BT.println("slightHot has been reset.");
        }
        else if (BTData.indexOf("mediumHot") != -1) {
          mediumHot = mediumHotDef;
          modifiedValues[7] = false;
          BT.println("mediumHot has been reset.");
        }
        else if (BTData.indexOf("justHot") != -1) {
          modifiedValues[8] = false;
          justHot = justHotDef;
          BT.println("justHot has been reset.");
        }
        else if (BTData.indexOf("veryHot") != -1) {
          veryHot = veryHotDef;
          modifiedValues[9] = false;
          BT.println("veryHot has been reset.");
        }
        //if there is no attribute then we reset all the parameters
        else {
          maxDistDiff = maxDistDiffDef;
          initialDelay = initialDelayDef;
          maxDefinedDist = maxDefinedDistDef;
          minLeftDegReflex = minLeftDegReflexDef;
          minRightDegReflex = minRightDegReflexDef;
          normalTemp = normalTempDef;
          slightHot = slightHotDef;
          mediumHot = mediumHotDef;
          justHot = justHotDef;
          veryHot = veryHotDef;
          for (int i = 0; i < modifiedValuesLength; i++) {
            modifiedValues[i] = false;
          }
          BT.println("All parameters have been reset.");
        }
      }
      //finally we save the data to the EEPROM
      else if (BTData.indexOf("save") != -1) {
        BT.println("Warning: Please note that Arduino EEPROM has limited lifetime. So please save the params after final checking.");

        for (int i = 0; i < modifiedValuesLength; i++) {
          if (modifiedValues[i] == true) {
            switch (i) {
              case 0:
                EEPROM.write(0, map(maxDistDiff, 1, 1000, 0, 254)) ;
                BT.println("Successfully saved: maxDistDiff = " + (String) (int) map(EEPROM.read(0), 0, 254, 1, 1000));
                modifiedValues[0] = false;
                break;
              case 1:
                EEPROM.write(1, map(initialDelay, 1, 1000, 0, 254));
                BT.println("Successfully saved: initialDelay = " + (String) (int) map(EEPROM.read(1), 0, 254, 1, 1000));
                modifiedValues[1] = false;
                break;
              case 2:
                EEPROM.write(2, map(maxDefinedDist, 1, 1000, 0, 254));
                BT.println("Successfully saved: maxDefinedDist = " + (String) (int) map(EEPROM.read(2), 0, 254, 1, 1000));
                modifiedValues[2] = false;
                break;
              case 3:
                EEPROM.write(3, map(minLeftDegReflex, 0, 180, 0, 254));
                BT.println("Successfully saved: minLeftDegReflex = " + (String) (int) map(EEPROM.read(3), 0, 254, 0, 180));
                modifiedValues[3] = false;
                break;
              case 4:
                EEPROM.write(4, map(minRightDegReflex, 0, 180, 0, 254));
                BT.println("Successfully saved: minRightDegReflex = " + (String) (int) map(EEPROM.read(4), 0, 254, 0, 180));
                modifiedValues[4] = false;
                break;
              case 5:
                EEPROM.write(5, map(normalTemp, -40, 220, 0, 254));
                BT.println("Successfully saved: normalTemp = " + (String) (float) map(EEPROM.read(5), 0, 254, -40, 220));
                modifiedValues[5] = false;
                break;
              case 6:
                EEPROM.write(6, map(slightHot, -40, 220, 0, 254));
                BT.println("Successfully saved: slightHot = " + (String) (float) map(EEPROM.read(6), 0, 254, -40, 220));
                modifiedValues[6] = false;
                break;
              case 7:
                EEPROM.write(7, map(mediumHot, -40, 220, 0, 254));
                BT.println("Successfully saved: mediumHot = " + (String) (float) map(EEPROM.read(7), 0, 254, -40, 220));
                modifiedValues[7] = false;
                break;
              case 8:
                EEPROM.write(8, map(justHot, -40, 220, 0, 254));
                BT.println("Successfully saved: justHot = " + (String) (float) map(EEPROM.read(8), 0, 254, -40, 220));
                modifiedValues[8] = false;
                break;
              case 9:
                EEPROM.write(9, map(veryHot, -40, 220, 0, 254));
                BT.println("Successfully saved: veryHot = " + (String) (float) map(EEPROM.read(9), 0, 254, -40, 220));
                modifiedValues[9] = false;
                break;
            }
          }
        }
        BT.println("Saved Successfully");
      }
      //reset the servo to their original positions
      else if (BTData.indexOf("resetServo") != -1) {
        servoHeadHor.write(90);
        servoHeadVer.write(90);
        rightShoulderY.write(90);
        leftShoulderY.write(90);
        rightShoulderX.write(0);
        leftShoulderX.write(180);
        rightWrist.write(90);
        leftWrist.write(90);
        base.write(90);
        rightEar.write(90);
        leftEar.write(90);
        BT.println("Servos have been reset to original position.");
      }
      //move a particular servo to the position
      //equal to sign is needed in this particular version
      else if (BTData.indexOf("move") != -1) {
        if (BTData.indexOf("=") != -1) {
          if (BTData.indexOf("servoHeadHor") != -1) {
            String temp = BTData.substring(BTData.indexOf("=") + 1);
            int value = isSpace(temp.charAt(0)) ? temp.substring(1).toInt() : temp.toInt();
            servoHeadHor.write(value);
            BT.println("Servo moved.");
          }
          else if (BTData.indexOf("servoHeadVer") != -1) {
            String temp = BTData.substring(BTData.indexOf("=") + 1);
            int value = isSpace(temp.charAt(0)) ? temp.substring(1).toInt() : temp.toInt();
            servoHeadVer.write(value);
            BT.println("Servo moved.");
          }
          else if (BTData.indexOf("rightShoulderY") != -1) {
            String temp = BTData.substring(BTData.indexOf("=") + 1);
            int value = isSpace(temp.charAt(0)) ? temp.substring(1).toInt() : temp.toInt();
            rightShoulderY.write(value);
            BT.println("Servo moved.");
          }
          else if (BTData.indexOf("leftShoulderY") != -1) {
            String temp = BTData.substring(BTData.indexOf("=") + 1);
            int value = isSpace(temp.charAt(0)) ? temp.substring(1).toInt() : temp.toInt();
            leftShoulderY.write(value);
            BT.println("Servo moved.");
          }
          else if (BTData.indexOf("rightShoulderX") != -1) {
            String temp = BTData.substring(BTData.indexOf("=") + 1);
            int value = isSpace(temp.charAt(0)) ? temp.substring(1).toInt() : temp.toInt();
            rightShoulderX.write(value);
            BT.println("Servo moved.");
          }
          else if (BTData.indexOf("leftShoulderX") != -1) {
            String temp = BTData.substring(BTData.indexOf("=") + 1);
            int value = isSpace(temp.charAt(0)) ? temp.substring(1).toInt() : temp.toInt();
            leftShoulderX.write(value);
            BT.println("Servo moved.");
          }
          else if (BTData.indexOf("rightWrist") != -1) {
            String temp = BTData.substring(BTData.indexOf("=") + 1);
            int value = isSpace(temp.charAt(0)) ? temp.substring(1).toInt() : temp.toInt();
            rightWrist.write(value);
            BT.println("Servo moved.");
          }
          else if (BTData.indexOf("leftWrist") != -1) {
            String temp = BTData.substring(BTData.indexOf("=") + 1);
            int value = isSpace(temp.charAt(0)) ? temp.substring(1).toInt() : temp.toInt();
            leftWrist.write(value);
            BT.println("Servo moved.");
          }
          else if (BTData.indexOf("base") != -1) {
            String temp = BTData.substring(BTData.indexOf("=") + 1);
            int value = isSpace(temp.charAt(0)) ? temp.substring(1).toInt() : temp.toInt();
            base.write(value);
            BT.println("Servo moved.");
          }
          else if (BTData.indexOf("rightEar") != -1) {
            String temp = BTData.substring(BTData.indexOf("=") + 1);
            int value = isSpace(temp.charAt(0)) ? temp.substring(1).toInt() : temp.toInt();
            rightEar.write(value);
            BT.println("Servo moved.");
          }
          else if (BTData.indexOf("leftEar") != -1) {
            String temp = BTData.substring(BTData.indexOf("=") + 1);
            int value = isSpace(temp.charAt(0)) ? temp.substring(1).toInt() : temp.toInt();
            leftEar.write(value);
            BT.println("Servo moved.");
          }
          else BT.println("Syntax Error: No correct name of servo found.");
        }
        else BT.println("Syntax Error: '=' sign not found.");
      }
      //list the SONAR readings
      else if (BTData.indexOf("ultrasonic") != -1) {
        BT.println("Left sensor: " + (String)  ultrasonicLeft.Ranging(CM));
        BT.println("Right sensor: " + (String)  ultrasonicRight.Ranging(CM));
      }
      //displays all the available commands to the user
      else if (BTData.indexOf("help") != -1) {
        BT.println("========================= Commands =========================================");
        BT.println("reset + attribute: resets the attribute param if provided, else all the params");
        BT.println("exit: exits the interface and starts the main mode");
        BT.println("list: lists all the parameters along with their definitions.");
        BT.println("help: launches the help menu.");
        BT.println("modify: modifies the parameters e.g => modify justHot = 97.64");
        BT.println("launchMain: starts the main function of the body.");
        BT.println("ultrasonic: shows the current readings of the two SONAR sensors");
        BT.println("========================= Parameters =======================================");
        BT.println("maxDistDiff: (int) max diff in cm to be assumed reflex.");
        BT.println("inititalDelay: (int) initial delay for the capacitors to charge up.");
        BT.println("maxDefinedDist: (int) max distance upto which SONAR is done.");
        BT.println("minLeftDegReflex: (int) min left ear degrees for reflex to work.");
        BT.println("minRightDegReflex: (int) min right ear degrees for reflex to work.");
        BT.println("normalTemp: (float) value of the normal temperature.");
        BT.println("slightHot: (float) value of slight hot temperature.");
        BT.println("mediumHot: (float) value of medium hot temperature.");
        BT.println("justHot: (float) value of just hot temperature.");
        BT.println("veryHot: (float) value of very hot temperature.");
        BT.println("========================= Servo controls ===================================");
        BT.println("resetServo: resets the servos to their original positions");
        BT.println("move: moves the specified servo at the specified position e.g => move leftWrist 180");
        BT.println("========================= Servo names ======================================");
        BT.println("servoHeadHor servoHeadVer rightShoulderY leftShoulderY rightShoulderX leftShoulderX");
        BT.println("rightWrist leftWrist base rightEar leftEar");
      }
      else if (BTData.indexOf("exit") != -1) {
        BT.println("Bluetooth interface exited");
        repeatThis = false;
        return;
      }
      else BT.println(BTData + " is not recognised as a command. Type help for the details.");
      BTData = "";  //clear the bluetooth data string after every iteration
      blinkLED();   //give a signal in the status LED
    }
    //delays added to make things stable
    delay(100);
  }
}

//================================================ The main part of the program ==================================================

void setup() {
  pinMode(tempSensor, INPUT);           //initialise the temperature sensor
  pinMode(BTstate, INPUT);              //the bluetooth state variable
  pinMode(LED, OUTPUT);                 //LED status
  BT.begin(9600);
  blinkLED();
  //set up bluetooth communication with the matrix
  resetTheVariables();
  servoHeadHor.attach(12);
  servoHeadVer.attach(11);
  rightShoulderY.attach(10);
  leftShoulderY.attach(9);
  rightShoulderX.attach(8);
  leftShoulderX.attach(7);
  rightWrist.attach(6);
  leftWrist.attach(28);
  base.attach(4);
  rightEar.attach(3);
  leftEar.attach(2);
  //wait for the body to be connected to bluetooth
  while (BTstate == LOW) delay(50);
  BT.println("Robotic Intuition Operator body has been connected.");
  BT.println("Developed by Debashish Buragohain");
  BT.println("The robot is in main mode. Type 'terminal' to launch the Bluetooth Terminal.");
  //if we have initialised all parameters
  if (initParameters() == true) {
    BT.println("Parameters have been set.");
    initModules();
  }
  else {
    BT.println("Error: One or more parameters were found to be empty. Starting BT interface.");
    btInterface();
    initModules();
  }
}

void loop() {
  //the main start of the program
  if (BTstate == HIGH) {
    mainStart();
  }
}

void blinkLED() {
  pinMode(LED, HIGH);
  delay(500);
  pinMode(LED, LOW);
  delay(500);
  pinMode(LED, HIGH);
  delay(500);
  pinMode(LED, LOW);
  delay(500);
  pinMode(LED, HIGH);
}

void mainStart() {
  //the temperature reading and sending part
  float tempC = analogRead(tempSensor);
  tempC = (tempC * 500) / 1023;   //temperature in celcius
  float tempF = (tempC * 1.8) + 32;     //temperature in fahrenheit
  delay(40);                            //delays added to make things stable
  //send the emotional quotients corresponding to the temperature of the robot
  if (tempF > normalTemp) {
    BT.print("slightHot::");
  }
  else if (tempF >= slightHot) {
    BT.print("mediumHot::");
  }
  else if (tempF >= mediumHot) {
    BT.print("Hot::");
  }
  else if (tempF >= justHot) {
    BT.print("veryHot::");
  }
  else if (tempF >= veryHot) {
    BT.print("extremeHot::");
  }

  //get the BT data from the core
  if (BT.available() > 0) {
    //firstly read the entire data sent
    while (BT.available()) {
      char c = BT.read();
      BTData += c;
      //the pose detection backend functions
      if (BT.read() == 'P') {
        getServoDegrees();
        moveBody();
      }
      //the face tracking functions
      else if (BT.read() == 'X')  {
        x = BT.parseInt();
        if (BT.read() == 'Y') {
          y = BT.parseInt();
          Pos();
        }
      }
      //Reflex actions
      else if (BT.read() == 'R') {
        if (RightDetect  == true) {
          servoHeadHor.write(0);
          delay(1000);
          RightDetect = false;
        }
        if (LeftDetect == true) {
          servoHeadHor.write(180);
          delay(1000);
          LeftDetect = false;
        }
      }
      //Reverting to original position after a reflex, this will be done when nothing special was detected
      else if (BT.read() == 'Q') {
        if (currentServoPos != -1) {
          servoHeadHor.write(currentServoPos);
          delay(500);
          currentServoPos = -1;   //reset the current servo position
        }
      }
      delay(40);
    }
    //launch the bluetooth terminal if we have been commanded.
    if (BTData.indexOf("terminal") != -1) btInterface();
  }

  //get the distances around
  distLeft = ultrasonicLeft.Ranging(CM);
  distRight = ultrasonicRight.Ranging(CM);
  //read only the distances upto the specified length, so that changes in too far the environment are not noticed
  distLeft = min(distLeft, maxDefinedDist);
  distRight = min(distRight, maxDefinedDist);
  BT.print("dist:" + (String) distLeft + '_' + (String) distRight);
  checkPresence();
  //update the distance variables
  prevDistLeft = distLeft;
  prevDistRight = distRight;
  //clear the text variable after every iteration
  BTData = "";
}

void checkPresence() {
  //the reflex action of the robot
  //in the current version the robot's reflex action is activated only in case of incoming object
  if (distLeft - prevDistLeft > maxDistDiff) {
    if (servoDegreesStart[10] >= minLeftDegReflex) {
      //in the current version, the robot shall do the reflex only upon instruction from backend
      currentServoPos = servoHeadHor.read();
      BT.print("LeftObjectDetected::"); //inform the backend
    }
  }
  if (distRight - prevDistRight > maxDistDiff) {
    if (servoDegreesStart[9] >= minRightDegReflex) {
      currentServoPos = servoHeadHor.read();
      BT.print("RightObjectDetected::"); //inform the backend
    }
  }
}

void Pos() {
  if (prevX != x || prevY != y) {
    int servoX = map(x, 600, 0, 179, 70);
    int servoY = map(y, 450, 0, 95, 179);

    servoX = min(servoX, 179);
    servoX = max(servoX, 70);
    servoY = min(servoY, 179);
    servoY = max(servoY, 95);

    servoHeadHor.write(servoX);
    servoHeadVer.write(servoY);
    delay(200);
  }
}

void getServoDegrees() {
  //get the position of the time operator and extract the data between it as the delay time
  int timeStartPos = BTData.indexOf('T');
  int timeEndPos = BTData.indexOf('P');
  //we extract the time given in the data
  givenDelayTime = BTData.substring(timeStartPos + 1, timeEndPos).toInt();
  //then we remove the extra characters from the given string
  BTData.remove(0, timeEndPos);
  int pos;
  //remove the + characters from the string and then put the data to the array
  int servoDegreesStartLength = sizeof(servoDegreesStart) / sizeof(int);
  for (int i = 0; i < servoDegreesStartLength ; i++) {
    pos = BTData.indexOf('+');
    if (pos != -1) {
      //we create a temporary string for storing the data
      String tempString;
      tempString = BTData.substring(0, pos);
      int separatorPos;
      //the underscore operator separates the two servo degrees
      separatorPos = tempString.indexOf('_');
      //the part before the underscore is the starting position and that after the operator is the ending position
      servoDegreesStart[i] = tempString.substring(0, separatorPos).toInt();
      //check if we have a null value in the given data and if so assign the null value to that element
      if (tempString.indexOf("null") != -1)
        servoDegreesEnd[i] = tempString.substring(separatorPos + 1).toInt();
      else
        servoDegreesEnd[i] = -1;  //-1 here refers to an undefined value
      int num = pos++;
      BTData.remove(0, num);      //remove the extracted part from the bluetooth string
    }
  }
}

void moveBody() {
  //move the body first to the starting position and then to the ending position
  //1001 is an error code that means no incoming data
  //-1 means a null value during extraction of the servo degrees
  if (servoDegreesStart[0] != 1001) {

    //first move the starting positions
    if (servoDegreesStart[0] != -1) servoHeadHor.write(servoDegreesStart[0]);
    if (servoDegreesStart[1] != -1) servoHeadVer.write(servoDegreesStart[1]);
    if (servoDegreesStart[2] != -1) rightShoulderY.write(servoDegreesStart[2]);
    if (servoDegreesStart[3] != -1) leftShoulderY.write(servoDegreesStart[3]);
    if (servoDegreesStart[4] != -1) rightShoulderX.write(servoDegreesStart[4]);
    if (servoDegreesStart[5] != -1) leftShoulderX.write(servoDegreesStart[5]);
    if (servoDegreesStart[6] != -1) rightWrist.write(servoDegreesStart[6]);
    if (servoDegreesStart[7] != -1) leftWrist.write(servoDegreesStart[7]);
    if (servoDegreesStart[8] != -1) base.write(servoDegreesStart[8]);
    if (servoDegreesStart[9] != -1) rightEar.write(servoDegreesStart[9]);
    if (servoDegreesStart[10] != -1) leftEar.write(servoDegreesStart[10]);
    //then wait till the given time
    if (givenDelayTime != -1) delay(givenDelayTime);
    //then move the ending positions
    //the horizontal head servo, and the ear servo have only the current positions
    //servoHeadHor.write(servoDegreesEnd[0]);
    if (servoDegreesEnd[1] != -1) servoHeadVer.write(servoDegreesEnd[1]);
    if (servoDegreesEnd[2] != -1) rightShoulderY.write(servoDegreesEnd[2]);
    if (servoDegreesEnd[3] != -1) leftShoulderY.write(servoDegreesEnd[3]);
    if (servoDegreesEnd[4] != -1) rightShoulderX.write(servoDegreesEnd[4]);
    if (servoDegreesEnd[5] != -1) leftShoulderX.write(servoDegreesEnd[5]);
    if (servoDegreesEnd[6] != -1) rightWrist.write(servoDegreesEnd[6]);
    if (servoDegreesEnd[7] != -1) leftWrist.write(servoDegreesEnd[7]);
    if (servoDegreesEnd[8] != -1) base.write(servoDegreesEnd[8]);
    //current version has no support for the ending ear servo position
    resetTheVariables();
  }
}
