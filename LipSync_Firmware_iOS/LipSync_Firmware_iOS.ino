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

//EDITED BY: Milad Hajihassan
//VERSION: 1.0 iOS (28 August 2018)

#include <EEPROM.h>
#include <math.h>

//***PIN ASSIGNMENTS***//

#define MODE_SELECT 12                            // LipSync Mode Select - USB mode (comm_mode = 0; jumper on) or Bluetooth mode (comm_mode = 1; jumper off) - digital input pin 12 (internally pulled-up)
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

int xh, yh, xl, yl;                               // xh: x-high, yh: y-high, xl: x-low, yl: y-low
int x_right, x_left, y_up, y_down;                // individual neutral starting positions for each FSR

int xh_max, xl_max, yh_max, yl_max;               // may just declare these variables but not initialize them because
// these values will be pulled from the EEPROM

float constant_radius = 30.0;                     // constant radius is initialized to 30.0 but may be changed in joystick initialization
float xh_yh_radius, xh_yl_radius, xl_yl_radius, xl_yh_radius;
float xh_yh, xh_yl, xl_yl, xl_yh;
int box_delta;                                    // the delta value for the boundary range in all 4 directions about the x,y center
int switch_delta;                                 // amount switch will activate in some single direction
int speed_counter = 4;                            // switch speed counter


int comm_mode = 0;                                // 0 == iOS mode or 1 == tvOS mode
int operation_mode = 0;                           // Switch between 2 iOS modes
int config_done;                                  // Binary check of completed Bluetooth configuration
bool bluetooth_can_config = false;                 // allow to config Bluetooth if the flag is true

unsigned int puff_count, sip_count;               // int puff and long sip incremental counter :: changed from unsigned long to unsigned int

int poll_counter = 0;                             // switch poll counter
int init_counter_A = 0;                           // serial port initialization counter
int init_counter_B = 0;                           // serial port initialization counter

int default_switch_speed = 30;
int delta_switch_speed = 5;

int switch_delay;
float switch_factor;
int switch_max_speed;

float yh_comp = 1.0;
float yl_comp = 1.0;
float xh_comp = 1.0;
float xl_comp = 1.0;

float yh_check, yl_check, xh_check, xl_check;
int xhm_check, xlm_check, yhm_check, ylm_check;
float sip_threshold, puff_threshold, switch_activate;

typedef struct {
  int _delay;
  float _factor;
  int _max_speed;
} _switch;

_switch setting1 = {5, -1.1, default_switch_speed - (4 * delta_switch_speed)}; // 5,-1.0,10
_switch setting2 = {5, -1.1, default_switch_speed - (3 * delta_switch_speed)}; // 5,-1.2,10
_switch setting3 = {5, -1.1, default_switch_speed - (2 * delta_switch_speed)};
_switch setting4 = {5, -1.1, default_switch_speed - (delta_switch_speed)};
_switch setting5 = {5, -1.1, default_switch_speed};
_switch setting6 = {5, -1.1, default_switch_speed + (delta_switch_speed)};
_switch setting7 = {5, -1.1, default_switch_speed + (2 * delta_switch_speed)};
_switch setting8 = {5, -1.1, default_switch_speed + (3 * delta_switch_speed)};
_switch setting9 = {5, -1.1, default_switch_speed + (4 * delta_switch_speed)};

_switch switch_params[9] = {setting1, setting2, setting3, setting4, setting5, setting6, setting7, setting8, setting9};

int single = 0;
int puff1, puff2, puff3;

//-----------------------------------------------------------------------------------------------------------------------------------

//***MICROCONTROLLER AND PERIPHERAL MODULES CONFIGURATION***//

void setup() {

  Serial.begin(115200);                           // setting baud rate for serial coms for diagnostic data return from Bluetooth and microcontroller ***MAY REMOVE LATER***
  Serial1.begin(115200);                          // setting baud rate for Bluetooth module

  pinMode(LED_1, OUTPUT);                         // visual feedback #1
  pinMode(LED_2, OUTPUT);                         // visual feedback #2
  pinMode(TRANS_CONTROL, OUTPUT);                 // transistor pin output
  pinMode(PIO4, OUTPUT);                          // command mode pin output

  pinMode(PRESSURE_CURSOR, INPUT);                // pressure sensor pin input
  pinMode(X_DIR_HIGH, INPUT);                     // redefine the pins when all has been finalized
  pinMode(X_DIR_LOW, INPUT);                      // ditto above
  pinMode(Y_DIR_HIGH, INPUT);                     // ditto above
  pinMode(Y_DIR_LOW, INPUT);                      // ditto above

  pinMode(MODE_SELECT, INPUT_PULLUP);             // LOW: iOS Mode HIGH: tvOS mode
  pinMode(PUSH_BUTTON_UP, INPUT_PULLUP);          // increase switch speed button
  pinMode(PUSH_BUTTON_DOWN, INPUT_PULLUP);        // decrease switch speed button

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);

  while(!Serial1);
  
  Joystick_Initialization();                      // home joystick and generate movement threshold boundaries
  delay(10);
  Pressure_Sensor_Initialization();
  delay(10);
  Set_Default();                                  // should only occur once per initialization of a new microcontroller
  delay(10);
  Communication_Mode_Status();                  // identify the selected communication mode
  delay(10);
                                  // conditionally configure the Bluetooth module [WHAT IF A NEW BT MODULE IS INSTALLED?]
  delay(10);
  Switch_Speed_Value();                           // reads saved switch speed parameter from EEPROM
  delay(10);
  Operation_Mode_Value();                         // read saved operation mode parameter from EEPROM
  delay(10);
  
  int exec_time = millis();
  Serial.print("Configuration time: ");
  Serial.println(exec_time);

  blink(4, 250, 3);                               // end initialization visual feedback
  BT_Configure(); 
  Display_Feature_List();  

  switch_delay = switch_params[speed_counter]._delay;
  switch_factor = switch_params[speed_counter]._factor;
  switch_max_speed = switch_params[speed_counter]._max_speed;

  // functions below are for diagnostic feedback only
   
  delay(10);
  Serial.print("config_done: ");
  Serial.println(EEPROM.get(0, puff1));
  delay(5);
  Serial.print("speed_counter: ");
  Serial.println(EEPROM.get(2, puff2));
  delay(5);
  Serial.print("operation_mode: ");
  Serial.println(EEPROM.get(30, puff3));
  delay(5);
  Serial.print("switch_delay: ");
  Serial.println(switch_params[puff2]._delay);
  delay(5);
  Serial.print("switch_factor: ");
  Serial.println(switch_params[puff2]._factor);
  delay(5);
  Serial.print("switch_max_speed: ");
  Serial.println(switch_params[puff2]._max_speed);
  delay(5);
  Serial.print("yh_comp factor: ");
  Serial.println(EEPROM.get(6, yh_check));
  delay(5);
  Serial.print("yl_comp factor: ");
  Serial.println(EEPROM.get(10, yl_check));
  delay(5);
  Serial.print("xh_comp factor: ");
  Serial.println(EEPROM.get(14, xh_check));
  delay(5);
  Serial.print("xl_comp factor: ");
  Serial.println(EEPROM.get(18, xl_check));
  delay(5);
  Serial.print("xh_max: ");
  Serial.println(EEPROM.get(22, xhm_check));
  delay(5);
  Serial.print("xl_max: ");
  Serial.println(EEPROM.get(24, xlm_check));
  delay(5);
  Serial.print("yh_max: ");
  Serial.println(EEPROM.get(26, yhm_check));
  delay(5);
  Serial.print("yl_max: ");
  Serial.println(EEPROM.get(28, ylm_check));
  delay(5);
}

//-----------------------------------------------------------------------------------------------------------------------------------

//***START OF INFINITE LOOP***//

void loop() {
  xh = analogRead(X_DIR_HIGH);                    // A0 :: NOT CORRECT MAPPINGS
  xl = analogRead(X_DIR_LOW);                     // A1
  yh = analogRead(Y_DIR_HIGH);                    // A2
  yl = analogRead(Y_DIR_LOW);                     // A10
///*
  xh_yh = sqrt(sq(((xh - x_right) > 0) ? (float)(xh - x_right) : 0.0) + sq(((yh - y_up) > 0) ? (float)(yh - y_up) : 0.0));     // sq() function raises input to power of 2, returning the same data type int->int ...
  xh_yl = sqrt(sq(((xh - x_right) > 0) ? (float)(xh - x_right) : 0.0) + sq(((yl - y_down) > 0) ? (float)(yl - y_down) : 0.0));   // the sqrt() function raises input to power 1/2, returning a float type
  xl_yh = sqrt(sq(((xl - x_left) > 0) ? (float)(xl - x_left) : 0.0) + sq(((yh - y_up) > 0) ? (float)(yh - y_up) : 0.0));      // These are the vector magnitudes of each quadrant 1-4. Since the FSRs all register
  xl_yl = sqrt(sq(((xl - x_left) > 0) ? (float)(xl - x_left) : 0.0) + sq(((yl - y_down) > 0) ? (float)(yl - y_down) : 0.0));    // a larger digital value with a positive application force, a large negative difference

  if ((xh_yh > xh_yh_radius) || (xh_yl > xh_yl_radius) || (xl_yl > xl_yl_radius) || (xl_yh > xl_yh_radius)) {
   poll_counter++;
   delay(30);    // originally 15 ms
    if (poll_counter >= 10) {
      if (comm_mode == 1) {                   //iOS mode
        if ((xh_yh >= xh_yl) && (xh_yh >= xl_yh) && (xh_yh >= xl_yl)) {
          //Serial.println("quad1");
          if (operation_mode == 0) {
            //Right arrow key
            (abs(x_left-xl)<(x_left)) ? Keyboard_Clear() : Keyboard_Command((byte)0x00,byte(0x4F));
            //Up arrow key
            (abs(y_down-yl)<(y_down)) ? Keyboard_Clear() : Keyboard_Command((byte)0x00,byte(0x52));
            //Keyboard_Command((byte)0x00,byte(0x4F));
          } else if (operation_mode == 1) {
            //Move keyboard cursor to the right direction (Alt/option Key + Right Arrow Key)
            (abs(x_left-xl)<(x_left)) ? Keyboard_Clear() : Keyboard_Command((1<<2),byte(0x4F));
            //Begin text selection on the right side of keyboard cursor (Shift Key + Right Arrow Key)
            (abs(y_down-yl)<(y_down)) ? Keyboard_Clear() : Keyboard_Command((1<<1),byte(0x4F));
            //Keyboard_Command((1<<2),byte(0x4F));
          }        
        } else if ((xh_yl > xh_yh) && (xh_yl > xl_yl) && (xh_yl > xl_yh)) {
          //Serial.println("quad4");
          if (operation_mode == 0) {
            //Right arrow key
            (abs(x_left-xl)<(x_left)) ? Keyboard_Clear() : Keyboard_Command((byte)0x00,byte(0x4F));
            //Down arrow key
            (abs(y_up-yh)<(y_up)) ? Keyboard_Clear() : Keyboard_Command((byte)0x00,byte(0x51));            
            //Keyboard_Command((byte)0x00,byte(0x51));
          } else if (operation_mode == 1) {
            //Move keyboard cursor to the right direction (Alt/option Key + Right Arrow Key)
            (abs(x_left-xl)<(x_left)) ? Keyboard_Clear() : Keyboard_Command((1<<2),byte(0x4F));
            //Begin text selection on the left side of keyboard cursor (Shift Key + Left Arrow key)
            (abs(y_up-yh)<(y_up)) ? Keyboard_Clear() : Keyboard_Command((1<<1),byte(0x50));  
            //Keyboard_Command((1<<1),byte(0x50));
          }        
        } else if ((xl_yl >= xh_yh) && (xl_yl >= xh_yl) && (xl_yl >= xl_yh)) {
          //Serial.println("quad3");
          if (operation_mode == 0) {
            //Left arrow key
            (abs(xh-x_right)<(x_right)) ? Keyboard_Clear() : Keyboard_Command((byte)0x00,byte(0x50));            
            //Down arrow key
            (abs(y_up-yh)<(y_up)) ? Keyboard_Clear() : Keyboard_Command((byte)0x00,byte(0x51));               
            //Keyboard_Command((byte)0x00,byte(0x50));
          } else if (operation_mode == 1) {
            //Move cursor left (Alt/option key + Left arrow key)
            (abs(xh-x_right)<(x_right)) ? Keyboard_Clear() : Keyboard_Command((1<<2),byte(0x50));                
            //Begin text selection on the left side of keyboard cursor (Shift Key + Left Arrow key)
            (abs(y_up-yh)<(y_up)) ? Keyboard_Clear() : Keyboard_Command((1<<1),byte(0x50));              
            //Keyboard_Command((1<<2),byte(0x50));
          }        
        } else if ((xl_yh > xh_yh) && (xl_yh >= xh_yl) && (xl_yh >= xl_yl)) {
          //Serial.println("quad2");
          if (operation_mode == 0) {
            //Left arrow key
            (abs(xh-x_right)<(x_right)) ? Keyboard_Clear() : Keyboard_Command((byte)0x00,byte(0x50));              
            //Up arrow key
            (abs(y_down-yl)<(y_down)) ? Keyboard_Clear() : Keyboard_Command((byte)0x00,byte(0x52));            
            //Keyboard_Command((byte)0x00,byte(0x52));
          } else if (operation_mode == 1) {
            //Move cursor left (Alt/option key + Left arrow key)
            (abs(xh-x_right)<(x_right)) ? Keyboard_Clear() : Keyboard_Command((1<<2),byte(0x50));               
            //Begin text selection on the right side of keyboard cursor (Shift Key + Right Arrow Key)
            (abs(y_down-yl)<(y_down)) ? Keyboard_Clear() : Keyboard_Command((1<<1),byte(0x4F));
            //Keyboard_Command((1<<1),byte(0x4F));
          }        
        }
        delay(switch_delay);       
        poll_counter = 0;
      } else {                   //tvOS mode
        if ((xh_yh >= xh_yl) && (xh_yh >= xl_yh) && (xh_yh >= xl_yl)) {
          //Serial.println("quad1");
          //Right arrow key
          (abs(x_left-xl)<(x_left)) ? Keyboard_Clear() : Keyboard_Command((byte)0x00,byte(0x4F));
          //Up arrow key
          (abs(y_down-yl)<(y_down)) ? Keyboard_Clear() : Keyboard_Command((byte)0x00,byte(0x52));          
          //Keyboard_Command((byte)0x00,byte(0x4F));
        } else if ((xh_yl > xh_yh) && (xh_yl > xl_yl) && (xh_yl > xl_yh)) {
          //Serial.println("quad4");
          //Right arrow key
          (abs(x_left-xl)<(x_left)) ? Keyboard_Clear() : Keyboard_Command((byte)0x00,byte(0x4F));
          //Down arrow key
          (abs(y_up-yh)<(y_up)) ? Keyboard_Clear() : Keyboard_Command((byte)0x00,byte(0x51));              
          //Keyboard_Command((byte)0x00,byte(0x51));
        } else if ((xl_yl >= xh_yh) && (xl_yl >= xh_yl) && (xl_yl >= xl_yh)) {
          //Serial.println("quad3");
          //Left arrow key
          (abs(xh-x_right)<(x_right)) ? Keyboard_Clear() : Keyboard_Command((byte)0x00,byte(0x50));            
          //Down arrow key
          (abs(y_up-yh)<(y_up)) ? Keyboard_Clear() : Keyboard_Command((byte)0x00,byte(0x51));  
          //Keyboard_Command((byte)0x00,byte(0x50));
        } else if ((xl_yh > xh_yh) && (xl_yh >= xh_yl) && (xl_yh >= xl_yl)) {
          //Serial.println("quad2");
          //Left arrow key
          (abs(xh-x_right)<(x_right)) ? Keyboard_Clear() : Keyboard_Command((byte)0x00,byte(0x50));              
          //Up arrow key
          (abs(y_down-yl)<(y_down)) ? Keyboard_Clear() : Keyboard_Command((byte)0x00,byte(0x52));   
          //Keyboard_Command((byte)0x00,byte(0x52));
        }
        delay(switch_delay);
        poll_counter = 0;
      }
    }
  }

  //switch speed control push button functions below

  if (digitalRead(PUSH_BUTTON_UP) == LOW) {
    delay(250);
    if (digitalRead(PUSH_BUTTON_DOWN) == LOW) {
      Joystick_Calibration();
    } else {
      Increase_Switch_Speed();      // increase switch speed with push button up
    }
  }

  if (digitalRead(PUSH_BUTTON_DOWN) == LOW) {
    delay(250);
    if (digitalRead(PUSH_BUTTON_UP) == LOW) {
      Joystick_Calibration();
    } else {
      Decrease_Switch_Speed();      // decrease switch speed with push button down
    }
  }

  //pressure sensor sip and puff functions below

  switch_activate = (((float)analogRead(PRESSURE_CURSOR)) / 1023.0) * 5.0;

  if (switch_activate < puff_threshold) {
    while (switch_activate < puff_threshold) {
      switch_activate = (((float)analogRead(PRESSURE_CURSOR)) / 1023.0) * 5.0;
      puff_count++;         // NEED TO FIGURE OUT ROUGHLY HOW LONG ONE CYCLE OF THIS WHILE LOOP IS -> COUNT THRESHOLD
      delay(5);
    }
    Serial.println(puff_count);             //***REMOVE

    if (comm_mode == 1) {                   //iOS mode
      if (puff_count < 150) {
        //Enter or select
        Keyboard_Command(byte(0x00),byte(0x28));
        delay(100);
      } else if (puff_count > 150 && puff_count < 750) {
        if (operation_mode == 0) {
          //Search
          //Cmd or KEY_GUI_LEFT key + Space key
          Keyboard_Command((1<<3),byte(0x2C));
        } else if (operation_mode == 1) {
          //Find
          //Control key + F key
          Keyboard_Command((1<<0),byte(0x09));
        }        
        delay(100);
      } else if (puff_count > 750) {
        blink(4, 350, 3);     // visual prompt for user to release joystick for automatic calibration of home position
        Manual_Joystick_Home_Calibration();
      }   
    } else {                   //tvOS mode
      if (puff_count < 150) {
        //Enter or select Key
        Keyboard_Command(byte(0x00),byte(0x28));
        delay(100);
      } else if (puff_count > 150 && puff_count < 750) {
        //F3 Key to switch to a different app
        Keyboard_Command(byte(0x00),byte(0x3C));
        delay(100);
      } else if (puff_count > 750) {
        blink(4, 350, 3);     // visual prompt for user to release joystick for automatic calibration of home position
        Manual_Joystick_Home_Calibration();
      } 
    }
    puff_count = 0;
  }

  if (switch_activate > sip_threshold) {
    while (switch_activate > sip_threshold) {
      switch_activate = (((float)analogRead(PRESSURE_CURSOR)) / 1023.0) * 5.0;
      sip_count++;         // NEED TO FIGURE OUT ROUGHLY HOW LONG ONE CYCLE OF THIS WHILE LOOP IS -> COUNT THRESHOLD
      delay(5);
    }
    Serial.println(sip_count);             //***REMOVE

    if (comm_mode == 1) {                   //iOS mode
      if (sip_count < 150) {
         //Space
        Keyboard_Command(byte(0x00),byte(0x2C));     
        delay(100);
      } else if (sip_count > 150 && sip_count < 750) {
        if (operation_mode == 0) {
          //Home screen
          //Cmd or KEY_GUI_LEFT key + H key
          Keyboard_Command((1<<3),byte(0x0B));
        } else if (operation_mode == 1) {
          //Home screen
          //Cmd or KEY_GUI_LEFT key + H key
          Keyboard_Command((1<<3),byte(0x0B));
        }        
        delay(100);
      } else if (sip_count > 750) {
        Change_Mode();
        delay(5);
      }   
    } else {                   //tvOS mode
      if (sip_count < 150) {
        //F4 Key - Go back
        Keyboard_Command(byte(0x00),byte(0x29));        
        delay(100);
      } else if (sip_count > 150 && sip_count < 750) {
        //F8 Key - Play or pause
        Keyboard_Command(byte(0x00),byte(0x41));       
        delay(100);
      }
    }
    sip_count = 0;
  }
}

//***CHANGE MODE FUNCTION SELECTION***//
void Change_Mode(void) {
  if (operation_mode == 0) {
    operation_mode++;
  } else {
    operation_mode=0;
  } 
  blink(operation_mode+1, 500, 1);
  EEPROM.put(30, operation_mode);
  delay(25);
}

//***END OF INFINITE LOOP***//

//-----------------------------------------------------------------------------------------------------------------------------------


void Display_Feature_List(void) {

  Serial.println(" ");
  Serial.println(" --- ");
  Serial.println("This is the LipSync iOS firmware : VERSION: 1.0 (28 August 2018)");
  Serial.println(" ");
  Serial.println("Enhanced functions:");
  Serial.println(" ");
  Serial.println("tvOS and iOS modes");
  Serial.println(" --- ");
  Serial.println(" ");

}

//***LED BLINK FUNCTIONS***//

void blink(int num_blinks, int delay_blinks, int led_number ) {
  if (num_blinks < 0) num_blinks *= -1;

  switch (led_number) {
    case 1: {
        for (int i = 0; i < num_blinks; i++) {
          digitalWrite(LED_1, HIGH);
          delay(delay_blinks);
          digitalWrite(LED_1, LOW);
          delay(delay_blinks);
        }
        break;
      }
    case 2: {
        for (int i = 0; i < num_blinks; i++) {
          digitalWrite(LED_2, HIGH);
          delay(delay_blinks);
          digitalWrite(LED_2, LOW);
          delay(delay_blinks);
        }
        break;
      }
    case 3: {
        for (int i = 0; i < num_blinks; i++) {
          digitalWrite(LED_1, HIGH);
          delay(delay_blinks);
          digitalWrite(LED_1, LOW);
          delay(delay_blinks);
          digitalWrite(LED_2, HIGH);
          delay(delay_blinks);
          digitalWrite(LED_2, LOW);
          delay(delay_blinks);
        }
        break;
      }
  }
}

//***OPERATION MODE NUMBER FUNCTIONS***//

void Operation_Mode_Value(void) {
  int var;
  EEPROM.get(30, var);
  delay(5);
  operation_mode = ((var == 0) || (var == 1)) ? var : 0;
  delay(5);
  EEPROM.put(30, operation_mode);
  delay(5);
}

//***HID KEYBOARD SWITCH SPEED FUNCTIONS***//

void Switch_Speed_Value(void) {
  int var;
  EEPROM.get(2, var);
  delay(5);
  speed_counter = var;
}

void Increase_Switch_Speed(void) {
  speed_counter++;

  if (speed_counter == 9) {
    blink(6, 50, 3);     // twelve very fast blinks
    speed_counter = 8;
  } else {
    blink(speed_counter, 100, 1);

    switch_delay = switch_params[speed_counter]._delay;
    switch_factor = switch_params[speed_counter]._factor;
    switch_max_speed = switch_params[speed_counter]._max_speed;

    EEPROM.put(2, speed_counter);
    delay(25);
    Serial.println("+");
  }
}

void Decrease_Switch_Speed(void) {
  speed_counter--;

  if (speed_counter == -1) {
    blink(6, 50, 3);     // twelve very fast blinks
    speed_counter = 0;
  } else if (speed_counter == 0) {
    blink(1, 350, 1);

    switch_delay = switch_params[speed_counter]._delay;
    switch_factor = switch_params[speed_counter]._factor;
    switch_max_speed = switch_params[speed_counter]._max_speed;

    EEPROM.put(2, speed_counter);
    delay(25);
    Serial.println("-");
  } else {
    blink(speed_counter, 100, 1);

    switch_delay = switch_params[speed_counter]._delay;
    switch_factor = switch_params[speed_counter]._factor;
    switch_max_speed = switch_params[speed_counter]._max_speed;

    EEPROM.put(2, speed_counter);
    delay(25);
    Serial.println("-");
  }
}



//***BLUETOOTH HID KEYBOARD IOS FUNCTIONS***//

void Keyboard_Command(byte modifier,byte button) {

    byte modifierByte=(byte)0x00;
    byte buttonByte=(byte)0x00;
    byte BTkeyboard[5];

    buttonByte=button;
    modifierByte=modifier;

    BTkeyboard[0] = 0xFE;
    BTkeyboard[1] = 0x3;
    BTkeyboard[2] = modifierByte;
    BTkeyboard[3] = buttonByte;
    BTkeyboard[4] = 0x0;

    Serial1.write(BTkeyboard,5);
    Serial1.flush();
    delay(10);
    Keyboard_Clear();

}

void Keyboard_Clear(void) {

  byte BTkeyboard[2];

  BTkeyboard[0] = 0xFD;
  BTkeyboard[1] = 0x00;
  Serial1.write(BTkeyboard,2);
  Serial1.flush();
  delay(10); 
  
}



//***COMMUNICATION MODE STATUS***//

void Communication_Mode_Status(void) {
  if (digitalRead(MODE_SELECT) == LOW) {
    comm_mode = 0;                                // 0 == tvOS mode
    delay(10);
    Serial.println("comm_mode = 0");
  } else if (digitalRead(MODE_SELECT) == HIGH) {
    comm_mode = 1;                                // 1 == iOS mode
    delay(10);
    Serial.println("comm_mode = 1");
  }
}

//***JOYSTICK INITIALIZATION FUNCTION***//

void Joystick_Initialization(void) {
  xh = analogRead(X_DIR_HIGH);            // Initial neutral x-high value of joystick
  delay(10);

  xl = analogRead(X_DIR_LOW);             // Initial neutral x-low value of joystick
  delay(10);

  yh = analogRead(Y_DIR_HIGH);            // Initial neutral y-high value of joystick
  delay(10);

  yl = analogRead(Y_DIR_LOW);             // Initial neutral y-low value of joystick
  delay(10);

  x_right = xh;
  x_left = xl;
  y_up = yh;
  y_down = yl;

  EEPROM.get(6, yh_comp);
  delay(10);
  EEPROM.get(10, yl_comp);
  delay(10);
  EEPROM.get(14, xh_comp);
  delay(10);
  EEPROM.get(18, xl_comp);
  delay(10);
  EEPROM.get(22, xh_max);
  delay(10);
  EEPROM.get(24, xl_max);
  delay(10);
  EEPROM.get(26, yh_max);
  delay(10);
  EEPROM.get(28, yl_max);
  delay(10);

  constant_radius = 30.0;                       //40.0 works well for a constant radius

  xh_yh_radius = constant_radius;
  xh_yl_radius = constant_radius;
  xl_yl_radius = constant_radius;
  xl_yh_radius = constant_radius;

}

//***PRESSURE SENSOR INITIALIZATION FUNCTION***//

void Pressure_Sensor_Initialization(void) {
  float nominal_switch_value = (((float)analogRead(PRESSURE_CURSOR)) / 1024.0) * 5.0; // Initial neutral pressure transducer analog value [0.0V - 5.0V]

  sip_threshold = nominal_switch_value + 0.5;    //Create sip pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation

  puff_threshold = nominal_switch_value - 0.5;   //Create puff pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
}


//----------------------RN-42 BLUETOOTH MODULE INITIALIZATION SECTION----------------------//

//***BLUETOOTH CONFIGURATION STATUS FUNCTION***//

void BT_Config_Status(void) {
  int BT_EEPROM;                                // Local integer variable initialized and defined for use with EEPROM GET function
  EEPROM.get(46, BT_EEPROM);                        // Assign value of EEPROM memory at index zero (0) to int variable BT_EEPROM
  delay(10);
  config_done = (BT_EEPROM == 1) ? BT_EEPROM : 0;  //Define the config_done to 0 if the mose is set for the first time
  Serial.println(BT_EEPROM);                       // Only for diagnostics, may be removed later
  delay(10);
}

//***BLUETOOTH CONFIGURATION FUNCTION***//

void BT_Configure(void) {
    BT_Config_Status();                    // check if Bluetooth has previously been configured
    delay(10);
    if ((config_done == 0) || bluetooth_can_config) {   // if Bluetooth has not been configured then execute configuration sequence
      BT_Command_Mode();                               // enter Bluetooth command mode
      BT_Config_Sequence();                           // send configuarion data to Bluetooth module
      delay(10);
    } else {
      Serial.println("Bluetooth configuration has previously been completed.");
      delay(10);
    }
}

void BT_Command_Mode(void) {                 //***CHANGE THE TRANSISTOR CONTROLS ONCE THE PNP IS SWITCHED IN FOR THE NPN***
  digitalWrite(TRANS_CONTROL, HIGH);         // transistor base pin HIGH to ensure Bluetooth module is off
  digitalWrite(PIO4, HIGH);                 // command pin high
  delay(10);
  digitalWrite(TRANS_CONTROL, LOW);        // transistor base pin LOW to power on Bluetooth module
  delay(10);

  for (int i = 0; i < 3; i++) {             // cycle PIO4 pin high-low 3 times with 1 sec delay between each level transition
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

void BT_Config_Sequence(void) {
    Serial1.print("$$$");                     // enter Bluetooth command mode :: "$$$" CANNOT be Serial.println("$$$") ONLY Serial.print("$$$")
  delay(50); 
  Serial1.println("ST,255");                 // turn off the 60 sec timer for command mode
  delay(15);
  Serial1.println("SA,2");                   // ***NEW ADDITION - Authentication Values 2: "any mode" work
  delay(15);
  Serial1.println("SX,0");                   // ***NEW ADDITION - Bonding 0: disabled
  delay(15);
  if (comm_mode == 0) {
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
  int val1 = speed_counter;

  EEPROM.put(0, val0);                        // EEPROM address 0 gets configuration completed value (== 1)
  delay(15);
  EEPROM.put(2, val1);                        // EEPROM address 1 gets default switch speed counter value (== 20) ***SHOULD ONLY OCCUR ONCE UNLESS THE LIPSYNC IS FACTORY RESET??
  delay(15);
  int val3;
  EEPROM.get(0, val3);  // diagnostics
  delay(15);            // diagnostics
  //Serial.println(val3); diagnostics
}

void BT_Low_Power_Mode(void) {
  BT_Command_Mode();                          // enter BT command mode
  Serial1.println('Z');                       // enter deep sleep mode (<2mA) when not connected
  delay(10);
  BT_configAOK();
  Serial.println("Bluetooth Deep Sleep Mode Activated");
  delay(10);
}

void BT_Connected_Status(void) {
  while (1) {
    Serial1.println("GK");
    delay(100);
    if (Serial1.available() > 0) {
      if ((char)Serial1.read() == '1') {
        Serial.println("BT is now connected!");
        delay(10);
        break;
      }
    }
  }
}

void BT_configAOK(void) {                    // diagnostic feedback from Bluetooth configuration
  while (Serial1.available() > 0) {
    Serial.print((char)Serial1.read());
  }
}

//***JOYSTICK SPEED CALIBRATION***//

void Joystick_Calibration(void) {

  Serial.println("Prepare for joystick calibration!");
  Serial.println(" ");
  blink(4, 300, 3);

  Serial.println("Move mouthpiece to the furthest vertical up position and hold it there until the LED turns SOLID RED, then release the mouthpiece.");
  blink(6, 500, 1);
  yh_max = analogRead(Y_DIR_HIGH);
  blink(1, 1000, 2);
  Serial.println(yh_max);

  Serial.println("Move mouthpiece to the furthest horizontal right position and hold it there until the LED turns SOLID RED, then release the mouthpiece.");
  blink(6, 500, 1);
  xh_max = analogRead(X_DIR_HIGH);
  blink(1, 1000, 2);
  Serial.println(xh_max);

  Serial.println("Move mouthpiece to the furthest vertical down position and hold it there until the LED turns SOLID RED, then release the mouthpiece.");
  blink(6, 500, 1);
  yl_max = analogRead(Y_DIR_LOW);
  blink(1, 1000, 2);
  Serial.println(yl_max);

  Serial.println("Move mouthpiece to the furthest horizontal left position and hold it there until the LED turns SOLID RED, then release the mouthpiece.");
  blink(6, 500, 1);
  xl_max = analogRead(X_DIR_LOW);
  blink(1, 1000, 2);
  Serial.println(xl_max);

  int max1 = (xh_max > xl_max) ? xh_max : xl_max;
  int max2 = (yh_max > yl_max) ? yh_max : yl_max;
  float max_final = (max1 > max2) ? (float)max1 : (float)max2;

  //int delta_max_total = (yh_max - y_up) + (yl_max - y_down) + (xh_max - x_right) + (xl_max - x_left);

  Serial.print("max_final: ");
  Serial.println(max_final);

  yh_comp = (max_final - y_up) / (yh_max - y_up);
  yl_comp = (max_final - y_down) / (yl_max - y_down);
  xh_comp = (max_final - x_right) / (xh_max - x_right);
  xl_comp = (max_final - x_left) / (xl_max - x_left);

  EEPROM.put(6, yh_comp);
  delay(10);
  EEPROM.put(10, yl_comp);
  delay(10);
  EEPROM.put(14, xh_comp);
  delay(10);
  EEPROM.put(18, xl_comp);
  delay(10);
  EEPROM.put(22, xh_max);
  delay(10);
  EEPROM.put(24, xl_max);
  delay(10);
  EEPROM.put(26, yh_max);
  delay(10);
  EEPROM.put(28, yl_max);
  delay(10);

  blink(5, 250, 3);

  Serial.println(" ");
  Serial.println("Joystick speed calibration procedure is complete.");
}



//***MANUAL JOYSTICK POSITION CALIBRATION***///
void Manual_Joystick_Home_Calibration(void) {

  xh = analogRead(X_DIR_HIGH);            // Initial neutral x-high value of joystick
  delay(10);
  Serial.println(xh);                     // Recommend keeping in for diagnostic purposes

  xl = analogRead(X_DIR_LOW);             // Initial neutral x-low value of joystick
  delay(10);
  Serial.println(xl);                     // Recommend keeping in for diagnostic purposes

  yh = analogRead(Y_DIR_HIGH);            // Initial neutral y-high value of joystick
  delay(10);
  Serial.println(yh);                     // Recommend keeping in for diagnostic purposes

  yl = analogRead(Y_DIR_LOW);             // Initial neutral y-low value of joystick
  delay(10);
  Serial.println(yl);                     // Recommend keeping in for diagnostic purposes

  x_right = xh;
  x_left = xl;
  y_up = yh;
  y_down = yl;

  int max1 = (xh_max > xl_max) ? xh_max : xl_max;
  int max2 = (yh_max > yl_max) ? yh_max : yl_max;
  float max_final = (max1 > max2) ? (float)max1 : (float)max2;

  Serial.print("max_final: ");
  Serial.println(max_final);

  yh_comp = (max_final - y_up) / (yh_max - y_up);
  yl_comp = (max_final - y_down) / (yl_max - y_down);
  xh_comp = (max_final - x_right) / (xh_max - x_right);
  xl_comp = (max_final - x_left) / (xl_max - x_left);

  EEPROM.put(6, yh_comp);
  delay(10);
  EEPROM.put(10, yl_comp);
  delay(10);
  EEPROM.put(14, xh_comp);
  delay(10);
  EEPROM.put(18, xl_comp);
  delay(10);

  Serial.println("Arrow position calibration complete.");

}
void Set_Default(void) {

  int default_config_setup;
  int default_switch_setting;
  int set_default;
  float default_comp_factor = 1.0;

  EEPROM.get(4, set_default);
  delay(10);

  if (set_default != 1) {

    default_config_setup = 0;
    EEPROM.put(0, default_config_setup);
    delay(10);

    default_switch_setting = 4;
    EEPROM.put(2, default_switch_setting);
    delay(10);

    EEPROM.put(6, default_comp_factor);
    delay(10);

    EEPROM.put(10, default_comp_factor);
    delay(10);

    EEPROM.put(14, default_comp_factor);
    delay(10);

    EEPROM.put(18, default_comp_factor);
    delay(10);

    set_default = 1;
    EEPROM.put(4, set_default);
    delay(10);

  }
}
