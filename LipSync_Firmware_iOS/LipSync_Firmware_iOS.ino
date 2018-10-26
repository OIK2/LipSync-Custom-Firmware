/*
  ----------------------------------------------------------
  =    //        //            ///               ///       =
  =    ////      //         ///  ///         ///    ///    =
  =    // //     //         ///              ///           =
  =    //  //    //          ///              ///          =
  =    //   //   //            ///              ///        =
  =    //    //  //              ///              ///      =
  =    //     // //               ///              ///     =
  =    //      ////        ///   ///         ///   ///     =
  =    //       ///           ///               ///        =
  ----------------------------------------------------------
*/
//Developed BY : MakersMakingChange
//EDITED BY: Milad Hajihassan
//VERSION: 1.1 iOS (25 October 2018)

#include <EEPROM.h>
#include <math.h>

//***PIN ASSIGNMENTS***//

#define MODE_SELECT 12                            // LipSync Mode Select 
#define PUSH_BUTTON_UP 8                          // Cursor Control Button 1: UP - digital input pin 8 (internally pulled-up)
#define PUSH_BUTTON_DOWN 7                        // Cursor Control Button 2: DOWN - digital input pin 7 (internally pulled-up)
#define LED_1 4                                   // LipSync LED Color1 : GREEN - digital output pin 5
#define LED_2 5                                   // LipSync LED Color2 : RED - digital outputpin 4

#define TRANS_CONTROL A3                          // Bluetooth Transistor Control Pin - digital output pin A3
#define PIO4 A4                                   // Bluetooth PIO4 Command Pin - digital output pin A4

#define PRESSURE_CURSOR A5                        // Sip & Puff Pressure Transducer Pin - analog input pin A5
#define X_DIR_HIGH A0                             // X Direction High (Cartesian positive x : right) - analog input pin A0
#define X_DIR_LOW A1                              // X Direction Low (Cartesian negative x : left) - digital output pin A1
#define Y_DIR_HIGH A2                             // Y Direction High (Cartesian positive y : up) - analog input pin A2
#define Y_DIR_LOW A10                             // Y Direction Low (Cartesian negative y : down) - analog input pin A10

//***VARIABLE DECLARATION***//

int xHigh, yHigh, xLow, yLow;  


int speedCounter = 4;                             //Declare variables for speed functionality 
int fixedDelay = 30;
int switchDelay;

int commMode = 0;                                 // 1 == iOS mode or 0 == tvOS mode
int operationMode = 0;                            // Switch between 2 iOS modes
int configIsDone;                                 // Binary check of completed Bluetooth configuration
bool bluetoothCanConfig = false;                  // Allow to config Bluetooth if the flag is true

float sipThreshold;                               //Declare sip and puff variables 
float puffThreshold;
float switchActivate;
int pollCounter = 0;                              //Switch poll counter
unsigned int puffCount;
unsigned int sipCount;


int puff1, puff2, puff3;

//-----------------------------------------------------------------------------------------------------------------------------------

//***MICROCONTROLLER AND PERIPHERAL MODULES CONFIGURATION***//

void setup() {

  Serial.begin(115200);                           // Set baud rate for serial coms for diagnostic data return from Bluetooth and microcontroller ***MAY REMOVE LATER***
  Serial1.begin(115200);                          // Set baud rate for Bluetooth module

  pinMode(LED_1, OUTPUT);                         //Set the visual feedback #1 LED pin to output mode
  pinMode(LED_2, OUTPUT);                         //Set the visual feedback #2 LED pin to output mode
  pinMode(TRANS_CONTROL, OUTPUT);                 //Set the transistor pin to output mode
  pinMode(PIO4, OUTPUT);                          //Set the unused pin to output mode

  pinMode(PRESSURE_CURSOR, INPUT);                //Set the pressure sensor pin to input mode
  
  pinMode(X_DIR_HIGH, INPUT);                     //Set the FSR pin to input mode
  pinMode(X_DIR_LOW, INPUT);
  pinMode(Y_DIR_HIGH, INPUT);
  pinMode(Y_DIR_LOW, INPUT);

  pinMode(MODE_SELECT, INPUT_PULLUP);             // LOW: iOS Mode HIGH: tvOS mode
  
  pinMode(PUSH_BUTTON_UP, INPUT_PULLUP);          //Set the increase joystick speed pin to input mode with pullup
  pinMode(PUSH_BUTTON_DOWN, INPUT_PULLUP);        //Set the decrease joystick speed pin to input mode with pullup

  pinMode(2, INPUT_PULLUP);                       //Set the unused pins to input mode with pullups
  pinMode(3, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);


  delay(10);
  
  while(!Serial1);
  
  pressureSensorInitialization();               //Initialize the pressure sensor
  delay(10);
  switchSpeedValue();                           // Reads saved switch speed parameter from EEPROM
  delay(10);
  operationModeValue();                         // Read saved operation mode parameter from EEPROM
  delay(10);
  
  int execTime = millis();
  Serial.print("Configuration time: ");
  Serial.println(execTime);

  blink(4, 250, 3);                               // End the initialization visual feedback
  
  
  displayFeatureList();                           //Display the list of features
  
  Serial.println(communicationModeStatus());      // Identify the selected communication mode 
  
  bluetoothConfigure(); 

  switchDelay = pow(1.6,(9-speedCounter))*fixedDelay;   //Set the default speed 

  Serial.print("Speed level: ");
  Serial.println((9-speedCounter));
  delay(5);
}

//-----------------------------------------------------------------------------------------------------------------------------------

//***START OF INFINITE LOOP***//

void loop() {
  
  xHigh = analogRead(X_DIR_HIGH);
  xLow = analogRead(X_DIR_LOW);
  yHigh = analogRead(Y_DIR_LOW);
  yLow = analogRead(Y_DIR_HIGH);

  xHigh = map(xHigh, 0, 1023, 0, 16);                 //Map x and y values from (0 to 1023) bound to (0 to 16) as target bound
  xLow = map(xLow, 0, 1023, 0, 16);                   //The actual input values are approximately in (0 to 800) bound range
  yHigh = map(yHigh, 0, 1023, 0, 16);
  yLow = map(yLow, 0, 1023, 0, 16);

  int xDelta = xHigh - xLow;                          //Calculate the x and y delta values   
  int yDelta = yHigh - yLow;
 
  int xx = (xDelta >= 0)? sq(xDelta):-sq(xDelta);     //Square the magnitude of x and y Delta values
  int yy = (yDelta >= 0)? sq(yDelta):-sq(yDelta);
  
  xx -= (xDelta >= 0)? int(sqrt(yDelta)):-int(sqrt(-yDelta));   //Subtract the square root of y Delta value from x Delta value to make movement smoother 
  yy -= (yDelta >= 0)? int(sqrt(xDelta)):-int(sqrt(-xDelta));   //Subtract the square root of x Delta value from y Delta value to make movement smoother 

  xx = constrain(xx, -128, 128);                      //Put constrain to set x and y range between -128 and 128 as lower and upper bounds 
  yy = constrain(yy, -128, 128);
  
  xx = map(xx, -128, 128, -10, 10);                   //Map back x and y range from (-128 to 128) as current bounds to (0 to 1023) as target bounds
  yy = map(yy, -128, 128, -10, 10);


  if (((abs(xx)) > 0) || ((abs(yy)) > 0)) {
   pollCounter++;
   delay(15);
      if (pollCounter >= 10) {
          if ((xx >= 5) && (-5 < yy < 5) && ((abs(xx)) > (abs(yy)))) {
            if ((operationMode == 0) || (commMode == 0)) {
              //Right arrow key
              keyboardCommand((byte)0x00,byte(0x4F));
            }
            else if (operationMode == 1) {
              //Move keyboard cursor to the right direction (Alt/option Key + Right Arrow Key)
              keyboardCommand((1<<2),byte(0x4F));
            }
          } 
          else if ((xx < -5) && (-5 < yy < 5) && ((abs(xx)) > (abs(yy)))){
            //Serial.println("left"); 
            if ((operationMode == 0) || (commMode == 0)) {
              //Left arrow key
              keyboardCommand((byte)0x00,byte(0x50)); 
            }
            else if (operationMode == 1) {
              //Move keyboard cursor to the left (Alt/option key + Left arrow key)
              keyboardCommand((1<<2),byte(0x50)); 
            }            
          }
          else if ((-5 < xx < 5) && (yy < -5) && ((abs(yy)) > (abs(xx)))){
            //Serial.println("up"); 
            if ((operationMode == 0) || (commMode == 0)) {
              //Up arrow key
              keyboardCommand((byte)0x00,byte(0x52));
            }
            else if (operationMode == 1) {
              //Begin text selection on the right side of keyboard cursor (Shift Key + Right Arrow Key)
              keyboardCommand((1<<1),byte(0x4F));
            }            
          }
          else if ((-5 < xx < 5) && (yy > 5) && ((abs(yy)) > (abs(xx)))){
            //Serial.println("down"); 
            if ((operationMode == 0) || (commMode == 0)) {
              //Down arrow key
              keyboardCommand((byte)0x00,byte(0x51));
            }
            else if (operationMode == 1) {
              //Begin text selection on the left side of keyboard cursor (Shift Key + Left Arrow key)
              keyboardCommand((1<<1),byte(0x50));  
            }            
          }    
        delay(switchDelay);       
        pollCounter = 0;
        }

  }
 
  if (digitalRead(PUSH_BUTTON_UP) == LOW) {
    delay(250);
    if (digitalRead(PUSH_BUTTON_DOWN) == LOW) {
    } else {
      increaseSwitchSpeed();      // Increase switch speed with push button up
    }
  }

  if (digitalRead(PUSH_BUTTON_DOWN) == LOW) {
    delay(250);
    if (digitalRead(PUSH_BUTTON_UP) == LOW) {
    } else {
      decreaseSwitchSpeed();      // Decrease switch speed with push button down
    }
  }

   //Pressure sensor sip and puff functions

  switchActivate = (((float)analogRead(PRESSURE_CURSOR)) / 1023.0) * 5.0;
  //Measure the pressure value and compare the result with puff pressure Thresholds 
  if (switchActivate < puffThreshold) {
    while (switchActivate < puffThreshold) {
      switchActivate = (((float)analogRead(PRESSURE_CURSOR)) / 1023.0) * 5.0;
      puffCount++;         //Threshold counter
      delay(5);
    }
    if (commMode == 1) {                   //iOS mode
      if (puffCount < 150) {
        //Enter or select
        keyboardCommand(byte(0x00),byte(0x28));
      } else if (puffCount > 150 && puffCount < 750) {
        if (operationMode == 0) {
          //Search
          //Cmd or KEY_GUI_LEFT key + Space key
          keyboardCommand((1<<3),byte(0x2C));
        } else if (operationMode == 1) {
          //Find
          //Control key + F key
          keyboardCommand((1<<0),byte(0x09));
        }        
      } 
    } else {                   //tvOS mode
      if (puffCount < 150) {
        //Enter or select Key
        keyboardCommand(byte(0x00),byte(0x28));
      } else if (puffCount > 150 && puffCount < 750) {
        //F3 Key to switch to a different app
        keyboardCommand(byte(0x00),byte(0x3C));
      } else if (puffCount > 750) {
        //F9 Key - Fast Forward 
        keyboardCommand(byte(0x00),byte(0x42));     
      }
    }
      delay(switchDelay);
      puffCount = 0;
  }
  //Measure the pressure value and compare the result with sip pressure Thresholds
  if (switchActivate > sipThreshold) {
    while (switchActivate > sipThreshold) {
      switchActivate = (((float)analogRead(PRESSURE_CURSOR)) / 1023.0) * 5.0;
      sipCount++;         ///Threshold counter
      delay(5);
    }
    if (commMode == 1) {                   //iOS mode    
      if (sipCount < 150) {
         //Space
        keyboardCommand(byte(0x00),byte(0x2C));  
      } else if (sipCount > 150 && sipCount < 750) {
        //Home screen
        //Cmd or KEY_GUI_LEFT key + H key
        keyboardCommand((1<<3),byte(0x0B));   
      } else if (sipCount > 750) {
        changeMode();
      }
    } else {                   //tvOS mode
      if (sipCount < 150) {
        //F4 Key - Go back
        keyboardCommand(byte(0x00),byte(0x3D));    
      } else if (sipCount > 150 && sipCount < 750) {
        //F8 Key - Play or pause
        keyboardCommand(byte(0x00),byte(0x41));     
      } else if (sipCount > 750) {
        //F7 Key - Rewind
        keyboardCommand(byte(0x00),byte(0x40));     
      }
    }
      delay(switchDelay); 
      sipCount = 0;
  }
}

//***CHANGE MODE FUNCTION SELECTION***//
void changeMode(void) {
  if (operationMode == 0) {
    operationMode++;
  } else {
    operationMode=0;
  } 
  blink(operationMode+1, 500, 1);
  EEPROM.put(30, operationMode);
  delay(25);
}

//***END OF INFINITE LOOP***//

//-----------------------------------------------------------------------------------------------------------------------------------

//***DISPLAY FEATURE LIST FUNCTION***//

void displayFeatureList(void) {

  Serial.println(" ");
  Serial.println(" --- ");
  Serial.println("This is the LipSync iOS/tvOS firmware : VERSION: 1.1 (25 October 2018)");
  Serial.println(" --- ");
  Serial.println(" ");

}

//***LED BLINK FUNCTIONS***//

void blink(int numBlinks, int delayBlinks, int ledNumber ) {
  if (numBlinks < 0) numBlinks *= -1;

  switch (ledNumber) {
    case 1: {
        for (int i = 0; i < numBlinks; i++) {
          digitalWrite(LED_1, HIGH);
          delay(delayBlinks);
          digitalWrite(LED_1, LOW);
          delay(delayBlinks);
        }
        break;
      }
    case 2: {
        for (int i = 0; i < numBlinks; i++) {
          digitalWrite(LED_2, HIGH);
          delay(delayBlinks);
          digitalWrite(LED_2, LOW);
          delay(delayBlinks);
        }
        break;
      }
    case 3: {
        for (int i = 0; i < numBlinks; i++) {
          digitalWrite(LED_1, HIGH);
          delay(delayBlinks);
          digitalWrite(LED_1, LOW);
          delay(delayBlinks);
          digitalWrite(LED_2, HIGH);
          delay(delayBlinks);
          digitalWrite(LED_2, LOW);
          delay(delayBlinks);
        }
        break;
      }
  }
}

//***OPERATION MODE NUMBER FUNCTIONS***//

void operationModeValue(void) {
  int var;
  EEPROM.get(30, var);
  delay(5);
  operationMode = ((var == 0) || (var == 1)) ? var : 0;
  delay(5);
  EEPROM.put(30, operationMode);
  delay(5);
}

//***HID KEYBOARD SWITCH SPEED FUNCTIONS***//

void switchSpeedValue(void) {
  int var;
  EEPROM.get(2, var);
  delay(5);
  speedCounter = var;
}

//***INCREASE SWITCH SPEED FUNCTION***//

void increaseSwitchSpeed(void) {
  speedCounter++;

  if (speedCounter == 9) {
    blink(6, 50, 3);
    speedCounter = 8;
  } else {
    blink(speedCounter, 100, 1);
    switchDelay = pow(1.6,(9-speedCounter))*fixedDelay;
    EEPROM.put(2, speedCounter);
    delay(25);
  }
  Serial.print("Speed level: ");
  Serial.println((9-speedCounter));
}

//***DECREASE JOYSTICK SPEED FUNCTION***//

void decreaseSwitchSpeed(void) {
  speedCounter--;

  if (speedCounter == -1) {
    blink(6, 50, 3);     // twelve very fast blinks
    speedCounter = 0;
  } else if (speedCounter == 0) {
    blink(1, 350, 1);
    switchDelay = pow(1.6,(9-speedCounter))*fixedDelay;
    EEPROM.put(2, speedCounter);
    delay(25);
  } else {
    blink(speedCounter, 100, 1);
    switchDelay = pow(1.6,(9-speedCounter))*fixedDelay;
    EEPROM.put(2, speedCounter);
    delay(25);
  }
  Serial.print("Speed level: ");
  Serial.println((9-speedCounter));
}



//***BLUETOOTH HID KEYBOARD IOS FUNCTIONS***//

void keyboardCommand(byte modifier,byte button) {

    byte modifierByte=(byte)0x00;
    byte buttonByte=(byte)0x00;
    byte bluetoothKeyboard[5];

    buttonByte=button;
    modifierByte=modifier;

    bluetoothKeyboard[0] = 0xFE;
    bluetoothKeyboard[1] = 0x3;
    bluetoothKeyboard[2] = modifierByte;
    bluetoothKeyboard[3] = buttonByte;
    bluetoothKeyboard[4] = 0x0;

    Serial1.write(bluetoothKeyboard,5);
    Serial1.flush();
    delay(10);
    keyboardClear();

}

void keyboardClear(void) {

  byte bluetoothKeyboard[2];

  bluetoothKeyboard[0] = 0xFD;
  bluetoothKeyboard[1] = 0x00;
  Serial1.write(bluetoothKeyboard,2);
  Serial1.flush();
  delay(10); 
  
}



//***COMMUNICATION MODE STATUS***//

String communicationModeStatus() {
  String modeName;
  if (digitalRead(MODE_SELECT) == LOW) {
    commMode = 0;                                     // 0 == tvOS mode
    modeName="tvOS Mode";
  } else if (digitalRead(MODE_SELECT) == HIGH) {
    commMode = 1;                                     // 1 == iOS mode
    modeName="iOS Mode";
  }
  delay(10);
  return modeName;
}

//***PRESSURE SENSOR INITIALIZATION FUNCTION***//

void pressureSensorInitialization(void) {
  float nominalSwitchValue = (((float)analogRead(PRESSURE_CURSOR)) / 1024.0) * 5.0; // Initial neutral pressure transducer analog value [0.0V - 5.0V]

  sipThreshold = nominalSwitchValue + 0.5;          //Create sip pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation

  puffThreshold = nominalSwitchValue - 0.5;         //Create puff pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
}


//----------------------RN-42 BLUETOOTH MODULE INITIALIZATION SECTION----------------------//

//***BLUETOOTH CONFIGURATION STATUS FUNCTION***//

void bluetoothConfigureStatus(void) {
  int var;                                      // Local integer variable initialized and defined for use with EEPROM GET function
  EEPROM.get(46, var);                          // Assign value of EEPROM memory at index zero (0) to int variable var
  delay(10);
  configIsDone = (var == 1) ? var : 0;          //Define the configIsDone to 0 if the mose is set for the first time
  //Serial.println(var);                          // Only for diagnostics, may be removed later
  delay(10);
}

//***BLUETOOTH CONFIGURATION FUNCTION***//

void bluetoothConfigure(void) {
    bluetoothConfigureStatus();                      // check if Bluetooth has previously been configured
    delay(10);
    if ((configIsDone == 0) || bluetoothCanConfig) {   // if Bluetooth has not been configured then execute configuration sequence
      bluetoothCommandMode();                               // enter Bluetooth command mode
      bluetoothConfigSequence();                           // send configuarion data to Bluetooth module
      delay(10);
    } else {
      Serial.println("Bluetooth configuration has previously been completed.");
      delay(10);
    }
}

//***CHANGE THE TRANSISTOR CONTROLS ONCE THE PNP IS SWITCHED IN FOR THE NPN***

void bluetoothCommandMode(void) {                 
  digitalWrite(TRANS_CONTROL, HIGH);                  // transistor base pin HIGH to ensure Bluetooth module is off
  digitalWrite(PIO4, HIGH);                           // command pin high
  delay(10);
  digitalWrite(TRANS_CONTROL, LOW);                   // transistor base pin LOW to power on Bluetooth module
  delay(10);

  for (int i = 0; i < 3; i++) {                       // cycle PIO4 pin high-low 3 times with 1 sec delay between each level transition
    digitalWrite(PIO4, HIGH);
    delay(150);
    digitalWrite(PIO4, LOW);
    delay(150);
  }

  digitalWrite(PIO4, LOW);                  // drive PIO4 pin low as per command mode instructions
  delay(10);
  Serial1.print("$$$");                     // enter Bluetooth command mode :: "$$$" CANNOT be Serial.println("$$$") ONLY Serial.print("$$$")
  delay(50);                              // time delay to visual inspect the red LED is flashing at 10Hz which indicates the Bluetooth module is in Command Mode
  Serial.println("Bluetooth Command Mode Activated");
}

void bluetoothConfigSequence(void) {
  Serial1.print("$$$");                     // enter Bluetooth command mode :: "$$$" CANNOT be Serial.println("$$$") ONLY Serial.print("$$$")
  delay(50); 
  Serial1.println("ST,255");                 // turn off the 60 sec timer for command mode
  delay(15);
  Serial1.println("SA,2");                   // ***NEW ADDITION - Authentication Values 2: "any mode" work
  delay(15);
  Serial1.println("SX,0");                   // ***NEW ADDITION - Bonding 0: disabled
  delay(15);
  if (commMode == 0) {
    Serial1.println("SN,LipSyncBT_tvOS");     // change name of BT module to LipSyncBT_tvOS
  }
  else {
    Serial1.println("SN,LipSyncBT_iOS");     // change name of BT module to LipSyncBT_iOS
  }
  delay(15);
  Serial1.println("SM,6");                   // ***NEW ADDITION - Pairing "SM,6": auto-connect mode
  delay(15);
  Serial1.println("SH,0000");                // configure device as HID keyboard
  delay(15);
  Serial1.println("S~,6");                   // activate HID profile
  delay(15);
  Serial1.println("SQ,0");                   // configure for latency NOT throughput -> turn off: "SQ,0"
  delay(15);
  Serial1.println("S?,1");                   // 1:ENABLE role switch -> slave device attempts role switch -> indicates better performance for high speed data
  delay(15);
  Serial1.println("R,1");                    // reboot BT module
  delay(15);

  int val0 = 1;
  int val1 = speedCounter;

  EEPROM.put(0, val0);                        // EEPROM address 0 gets configuration completed value (== 1)
  delay(15);
  EEPROM.put(2, val1);                        // EEPROM address 1 gets default switch speed counter value (== 20) ***SHOULD ONLY OCCUR ONCE UNLESS THE LIPSYNC IS FACTORY RESET??
  delay(15);
  int val3;
  EEPROM.get(0, val3);                        // diagnostics
  delay(15); 
}



