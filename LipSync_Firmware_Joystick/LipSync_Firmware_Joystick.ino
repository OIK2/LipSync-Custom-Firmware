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
//VERSION: 1.0 (9 May 2018)


#include <EEPROM.h>
#include "Joystick.h"
#include <math.h>

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, 
  JOYSTICK_TYPE_JOYSTICK, 4, 0,
  true, true, false, 
  false, false, false,
  false, false, 
  false, false, false);

int button1 = 0;
int button2 = 1;
int button3 = 2;
int button4 = 3;



//***PIN ASSIGNMENTS***//

#define MODE_SELECT 12                            // LipSync Mode Select - USB mode (comm_mode = 0; jumper on) or Bluetooth mode (comm_mode = 1; jumper off) - digital input pin 12 (internally pulled-up)
#define PUSH_BUTTON_UP 8                          // Joystick Control Button 1: UP - digital input pin 8 (internally pulled-up)
#define PUSH_BUTTON_DOWN 7                        // Joystick Control Button 2: DOWN - digital input pin 7 (internally pulled-up)
#define LED_1 4                                   // LipSync LED Color1 : GREEN - digital output pin 5
#define LED_2 5                                   // LipSync LED Color2 : RED - digital outputpin 4

#define TRANS_CONTROL A3                          // Bluetooth Transistor Control Pin - digital output pin A3
#define PIO4 A4                                   // Bluetooth PIO4 Command Pin - digital output pin A4

#define PRESSURE_JOYSTICK A5                        // Sip & Puff Pressure Transducer Pin - analog input pin A5
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
int joystick_delta;                                 // amount joystick moves in some single or combined direction
int speed_counter = 4;                            // joystick speed counter
int joystick_click_status = 0;                      // value indicator for click status, ie. tap, back and drag
int comm_mode = 0;                                // 0 == USB Communications or 1 == Bluetooth Communications
int config_done;                                  // Binary check of completed Bluetooth configuration
unsigned int puff_count, sip_count;               // int puff and long sip incremental counter :: changed from unsigned long to unsigned int

int poll_counter = 0;                             // joystick poll counter
int init_counter_A = 0;                           // serial port initialization counter
int init_counter_B = 0;                           // serial port initialization counter

int default_joystick_speed = 30;
int delta_joystick_speed = 5;

int joystick_delay;
float joystick_factor;
int joystick_max_speed;

float yh_comp = 1.0;
float yl_comp = 1.0;
float xh_comp = 1.0;
float xl_comp = 1.0;

float yh_check, yl_check, xh_check, xl_check;
int xhm_check, xlm_check, yhm_check, ylm_check;
float sip_threshold, puff_threshold, joystick_click, joystick_back;

typedef struct {
  int _delay;
  float _factor;
  int _max_speed;
} _joystick;

_joystick setting1 = {5, -1.1, default_joystick_speed - (4 * delta_joystick_speed)}; // 5,-1.0,10
_joystick setting2 = {5, -1.1, default_joystick_speed - (3 * delta_joystick_speed)}; // 5,-1.2,10
_joystick setting3 = {5, -1.1, default_joystick_speed - (2 * delta_joystick_speed)};
_joystick setting4 = {5, -1.1, default_joystick_speed - (delta_joystick_speed)};
_joystick setting5 = {5, -1.1, default_joystick_speed};
_joystick setting6 = {5, -1.1, default_joystick_speed + (delta_joystick_speed)};
_joystick setting7 = {5, -1.1, default_joystick_speed + (2 * delta_joystick_speed)};
_joystick setting8 = {5, -1.1, default_joystick_speed + (3 * delta_joystick_speed)};
_joystick setting9 = {5, -1.1, default_joystick_speed + (4 * delta_joystick_speed)};

_joystick joystick_params[9] = {setting1, setting2, setting3, setting4, setting5, setting6, setting7, setting8, setting9};

int single = 0;
int puff1, puff2;


int lastButtonState[4];       // Last state of the button
int deadZoneDivisor = 4;      // a constant to calculate deadzone joystick area for each axis

//-----------------------------------------------------------------------------------------------------------------------------------

//***MICROCONTROLLER AND PERIPHERAL MODULES CONFIGURATION***//

void setup() {

  Serial.begin(115200);                           // setting baud rate for serial coms for diagnostic data return from Bluetooth and microcontroller ***MAY REMOVE LATER***
  Serial1.begin(115200);                          // setting baud rate for Bluetooth module

  pinMode(LED_1, OUTPUT);                         // visual feedback #1
  pinMode(LED_2, OUTPUT);                         // visual feedback #2
  pinMode(TRANS_CONTROL, OUTPUT);                 // transistor pin output
  pinMode(PIO4, OUTPUT);                          // command mode pin output

  pinMode(PRESSURE_JOYSTICK, INPUT);                // pressure sensor pin input
  pinMode(X_DIR_HIGH, INPUT);                     // redefine the pins when all has been finalized
  pinMode(X_DIR_LOW, INPUT);                      // ditto above
  pinMode(Y_DIR_HIGH, INPUT);                     // ditto above
  pinMode(Y_DIR_LOW, INPUT);                      // ditto above

  pinMode(MODE_SELECT, INPUT_PULLUP);             // LOW: USB (default with jumper in) HIGH: Bluetooth (jumper removed)
  pinMode(PUSH_BUTTON_UP, INPUT_PULLUP);          // increase joystick speed button
  pinMode(PUSH_BUTTON_DOWN, INPUT_PULLUP);        // decrease joystick speed button

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);

  //delay(2000);                                    // DO NOT REMOVE DELAY!!!

  //while(!Serial);

  while(!Serial1);
  
  // Initialize Joystick Library
  Joystick.begin();
  delay(10);
  Joystick_Initialization();                      // home joystick and generate movement threshold boundaries
  delay(10);
  Pressure_Sensor_Initialization();
  delay(10);
  Set_Default();                                  // should only occur once per initialization of a new microcontroller
  delay(10);                             // conditionally configure the Bluetooth module [WHAT IF A NEW BT MODULE IS INSTALLED?]

  lastButtonState[0] = 0;
  lastButtonState[1] = 0;
  lastButtonState[2] = 0;
  lastButtonState[3] = 0;

  joystick_speed_value();                           // reads saved joystick speed parameter from EEPROM
  delay(10);
  int exec_time = millis();
  Serial.print("Configuration time: ");
  Serial.println(exec_time);

  blink(4, 250, 3);                               // end initialization visual feedback

  //Display_Feature_List();  

  joystick_delay = joystick_params[speed_counter]._delay;
  joystick_factor = joystick_params[speed_counter]._factor;
  joystick_max_speed = joystick_params[speed_counter]._max_speed;

  // functions below are for diagnostic feedback only

  Serial.print("config_done: ");
  Serial.println(EEPROM.get(0, puff1));
  delay(5);
  Serial.print("speed_counter: ");
  Serial.println(EEPROM.get(2, puff2));
  delay(5);
  Serial.print("joystick_delay: ");
  Serial.println(joystick_params[puff2]._delay);
  delay(5);
  Serial.print("joystick_factor: ");
  Serial.println(joystick_params[puff2]._factor);
  delay(5);
  Serial.print("joystick_max_speed: ");
  Serial.println(joystick_params[puff2]._max_speed);
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
  Joystick.setXAxisRange(-(x_right),(x_left));
  Joystick.setYAxisRange(-(y_up),(y_down));
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

    delay(20);    // originally 15 ms
    if (poll_counter >= 3) {
      
        if ((xh_yh >= xh_yl) && (xh_yh >= xl_yh) && (xh_yh >= xl_yl)) {
          //Serial.println("quad1");
          (abs(x_left-xl)<(x_left/deadZoneDivisor)) ? Joystick.setXAxis(0) : Joystick.setXAxis((x_left-xl));
          (abs(y_down-yl)<(y_down/deadZoneDivisor)) ? Joystick.setYAxis(0) : Joystick.setYAxis(-(y_down-yl));
          //Joystick.setXAxis(x_left-xl);
          //Joystick.setYAxis(-(y_down-yl));
        } else if ((xh_yl > xh_yh) && (xh_yl > xl_yl) && (xh_yl > xl_yh)) {
          //Serial.println("quad4");
          (abs(x_left-xl)<(x_left/deadZoneDivisor)) ? Joystick.setXAxis(0) : Joystick.setXAxis((x_left-xl));
          (abs(y_up-yh)<(y_up/deadZoneDivisor)) ? Joystick.setYAxis(0) : Joystick.setYAxis((y_up-yh));
          //Joystick.setXAxis(x_left-xl);
          //Joystick.setYAxis((y_up-yh));
        } else if ((xl_yl >= xh_yh) && (xl_yl >= xh_yl) && (xl_yl >= xl_yh)) {
          //Serial.println("quad3");
          (abs(xh-x_right)<(x_right/deadZoneDivisor)) ? Joystick.setXAxis(0) : Joystick.setXAxis((xh-x_right));
          (abs(y_up-yh)<(y_up/deadZoneDivisor)) ? Joystick.setYAxis(0) : Joystick.setYAxis((y_up-yh));
          //Joystick.setXAxis((xh-x_right));
          //Joystick.setYAxis((y_up-yh));
        } else if ((xl_yh > xh_yh) && (xl_yh >= xh_yl) && (xl_yh >= xl_yl)) {
          //Serial.println("quad2: ");
          (abs(xh-x_right)<(x_right/deadZoneDivisor)) ? Joystick.setXAxis(0) : Joystick.setXAxis((xh-x_right));
          (abs(y_down-yl)<(y_down/deadZoneDivisor)) ? Joystick.setYAxis(0) : Joystick.setYAxis(-(y_down-yl));
          //Joystick.setXAxis((xh-x_right));
          //Joystick.setYAxis(-(y_down-yl));
        }
         delay(joystick_delay);
         poll_counter = 0;
    }
  } else {
    Joystick.setXAxis(0);
    Joystick.setYAxis(0);
  }
  

  //joystick speed control push button functions below

  if (digitalRead(PUSH_BUTTON_UP) == LOW) {
    delay(250);
    if (digitalRead(PUSH_BUTTON_DOWN) == LOW) {
      Joystick_Calibration();
    } else {
      increase_joystick_speed();      // increase joystick speed with push button up
    }
  }

  if (digitalRead(PUSH_BUTTON_DOWN) == LOW) {
    delay(250);
    if (digitalRead(PUSH_BUTTON_UP) == LOW) {
      Joystick_Calibration();
    } else {
      decrease_joystick_speed();      // decrease joystick speed with push button down
    }
  }

  //pressure sensor sip and puff functions below

  joystick_click = (((float)analogRead(PRESSURE_JOYSTICK)) / 1023.0) * 5.0;

  if (joystick_click < puff_threshold) {
    while (joystick_click < puff_threshold) {
      joystick_click = (((float)analogRead(PRESSURE_JOYSTICK)) / 1023.0) * 5.0;
      puff_count++;         // NEED TO FIGURE OUT ROUGHLY HOW LONG ONE CYCLE OF THIS WHILE LOOP IS -> COUNT THRESHOLD
      delay(5);
    }
    Serial.println(puff_count);             //***REMOVE
      if (puff_count < 150) {
        Serial.println(puff_count); 
        if (!lastButtonState[0]) {
          Joystick.pressButton(button1);
          delay(250);
          Joystick.releaseButton(button1);
          delay(50);
          lastButtonState[0] = 0;
        }
      } else if (puff_count > 150 && puff_count < 500) {
        if (!lastButtonState[2]) {
          Joystick.pressButton(button3);
          delay(250);
          Joystick.releaseButton(button3);
          delay(50);
          lastButtonState[2] = 0;
        } 
      } else if (puff_count > 750) {
        blink(4, 350, 3);   // visual prompt for user to release joystick for automatic calibration of home position
        Manual_Joystick_Home_Calibration();
      }

    puff_count = 0;
  }

  if (joystick_click > sip_threshold) {
    while (joystick_click > sip_threshold) {
      joystick_click = (((float)analogRead(PRESSURE_JOYSTICK)) / 1023.0) * 5.0;
      sip_count++;         // NEED TO FIGURE OUT ROUGHLY HOW LONG ONE CYCLE OF THIS WHILE LOOP IS -> COUNT THRESHOLD
      delay(5);
    }
    Serial.println(sip_count);             //***REMOVE
      if (sip_count < 150) {
        if (!lastButtonState[1]) {
          Joystick.pressButton(button2);
          delay(250);
          Joystick.releaseButton(button2);
          delay(50);
          lastButtonState[1] = 0;
        } 
      } else if (sip_count > 150 && sip_count < 500) {
         if (!lastButtonState[3]) {
          Joystick.pressButton(button4);
          delay(250);
          Joystick.releaseButton(button4);
          delay(50);
          lastButtonState[3] = 0;
        } 
      } else {
        //sip_secondary();
        delay(5);
      }
    sip_count = 0;
  }
  
}

//***END OF INFINITE LOOP***//

//-----------------------------------------------------------------------------------------------------------------------------------


void Display_Feature_List(void) {

  Serial.println(" ");
  Serial.println(" --- ");
  Serial.println("This is the LipSync Joystick firmware : VERSION: 1.0 (9 May 2018)");
  Serial.println(" ");
  Serial.println("Enhanced functions:");
  Serial.println(" ");
  Serial.println("Joystick functionality");
  Serial.println(" --- ");
  Serial.println(" ");

}

//***: SIP SECONDARY FUNCTION SELECTION***//
void sip_secondary(void) {
  while (1) {

    xh = analogRead(X_DIR_HIGH);                    // A0 :: NOT CORRECT MAPPINGS
    xl = analogRead(X_DIR_LOW);                     // A1
    yh = analogRead(Y_DIR_HIGH);                    // A2
    yl = analogRead(Y_DIR_LOW);                     // A10

    digitalWrite(LED_2, HIGH);

  }
  digitalWrite(LED_2, LOW);
}




//***LED BLINK FUNCTIONS***//

void blink(int num_Blinks, int delay_Blinks, int LED_number ) {
  if (num_Blinks < 0) num_Blinks *= -1;

  switch (LED_number) {
    case 1: {
        for (int i = 0; i < num_Blinks; i++) {
          digitalWrite(LED_1, HIGH);
          delay(delay_Blinks);
          digitalWrite(LED_1, LOW);
          delay(delay_Blinks);
        }
        break;
      }
    case 2: {
        for (int i = 0; i < num_Blinks; i++) {
          digitalWrite(LED_2, HIGH);
          delay(delay_Blinks);
          digitalWrite(LED_2, LOW);
          delay(delay_Blinks);
        }
        break;
      }
    case 3: {
        for (int i = 0; i < num_Blinks; i++) {
          digitalWrite(LED_1, HIGH);
          delay(delay_Blinks);
          digitalWrite(LED_1, LOW);
          delay(delay_Blinks);
          digitalWrite(LED_2, HIGH);
          delay(delay_Blinks);
          digitalWrite(LED_2, LOW);
          delay(delay_Blinks);
        }
        break;
      }
  }
}

//***HID JOYSTICK SPEED FUNCTIONS***//

void joystick_speed_value(void) {
  int var;
  EEPROM.get(2, var);
  delay(5);
  speed_counter = var;
}

void increase_joystick_speed(void) {
  speed_counter++;

  if (speed_counter == 9) {
    blink(6, 50, 3);     // twelve very fast blinks
    speed_counter = 8;
  } else {
    blink(speed_counter, 100, 1);

    joystick_delay = joystick_params[speed_counter]._delay;
    joystick_factor = joystick_params[speed_counter]._factor;
    joystick_max_speed = joystick_params[speed_counter]._max_speed;

    EEPROM.put(2, speed_counter);
    delay(25);
    Serial.println("+");
  }
}

void decrease_joystick_speed(void) {
  speed_counter--;

  if (speed_counter == -1) {
    blink(6, 50, 3);     // twelve very fast blinks
    speed_counter = 0;
  } else if (speed_counter == 0) {
    blink(1, 350, 1);

    joystick_delay = joystick_params[speed_counter]._delay;
    joystick_factor = joystick_params[speed_counter]._factor;
    joystick_max_speed = joystick_params[speed_counter]._max_speed;

    EEPROM.put(2, speed_counter);
    delay(25);
    Serial.println("-");
  } else {
    blink(speed_counter, 100, 1);

    joystick_delay = joystick_params[speed_counter]._delay;
    joystick_factor = joystick_params[speed_counter]._factor;
    joystick_max_speed = joystick_params[speed_counter]._max_speed;

    EEPROM.put(2, speed_counter);
    delay(25);
    Serial.println("-");
  }
}

//***HID JOYSTICK MOVEMENT FUNCTIONS***//

int y_joystick_high(int j) {

  if (j > y_up) {

    float y_up_factor = 1.25 * (yh_comp * (((float)(j - y_up)) / (yh_max - y_up)));

    int k = (int)(round(-1.0 * pow(joystick_max_speed, y_up_factor)) - 1.0);

    if (k <= (-1 * joystick_max_speed) ) {
      k = -1 * joystick_max_speed;
      return k;
    } else if ( (k < 0) && (k > (-1 * joystick_max_speed))) {
      return k;
    } else {
      k = 0;
      return k;
    }
  } else {
    return 0;
  }
}

int y_joystick_low(int j) {

  if (j > y_down) {

    float y_down_factor = 1.25 * (yl_comp * (((float)(j - y_down)) / (yl_max - y_down)));

    int k = (int)(round(1.0 * pow(joystick_max_speed, y_down_factor)) - 1.0);

    if (k >= joystick_max_speed) {

      k = joystick_max_speed;
      return k;
    } else if ((k > 0) && (k < joystick_max_speed)) {
      return k;
    } else {
      k = 0;
      return k;
    }
  } else {
    return 0;
  }
}

int x_joystick_high(int j) {

  if (j > x_right) {

    float x_right_factor = 1.25 * (xh_comp * (((float)(j - x_right)) / (xh_max - x_right)));

    int k = (int)(round(1.0 * pow(joystick_max_speed, x_right_factor)) - 1.0);

    if (k >= joystick_max_speed) {

      k = joystick_max_speed;
      return k;
    } else if ((k > 0) && (k < joystick_max_speed)) {
      return k;
    } else {
      k = 0;
      return k;
    }
  } else {
    return 0;
  }
}

int x_joystick_low(int j) {

  if (j > x_left) {

    float x_left_factor = 1.25 * (xl_comp * (((float)(j - x_left)) / (xl_max - x_left)));

    int k = (int)(round(-1.0 * pow(joystick_max_speed, x_left_factor)) - 1.0);

    if ( k <= (-1 * joystick_max_speed) ) {
      k = -1 * joystick_max_speed;
      return k;
    } else if ( (k < 0) && (k > -1 * joystick_max_speed)) {
      return k;
    } else {
      k = 0;
      return k;
    }
  } else {
    return 0;
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
  float nominal_joystick_value = (((float)analogRead(PRESSURE_JOYSTICK)) / 1024.0) * 5.0; // Initial neutral pressure transducer analog value [0.0V - 5.0V]

  sip_threshold = nominal_joystick_value + 0.5;    //Create sip pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation

  puff_threshold = nominal_joystick_value - 0.5;   //Create puff pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
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

  //float avg_delta_max = ((float)(delta_max_total)) / 4;

  //Serial.print("avg_delta_max: ");
  //Serial.println(avg_delta_max);

  yh_comp = (max_final - y_up) / (yh_max - y_up);
  yl_comp = (max_final - y_down) / (yl_max - y_down);
  xh_comp = (max_final - x_right) / (xh_max - x_right);
  xl_comp = (max_final - x_left) / (xl_max - x_left);


  Joystick.setXAxisRange((xl_max-x_left),-(xh_max-x_right));
  Joystick.setYAxisRange((yl_max-y_down),-(yh_max-y_up));

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

  //int delta_max_total = (yh_max - y_up) + (yl_max - y_down) + (xh_max - x_right) + (xl_max - x_left);

  Serial.print("max_final: ");
  Serial.println(max_final);

  //float avg_delta_max = ((float)(delta_max_total)) / 4;

  //Serial.print("avg_delta_max: ");
  //Serial.println(avg_delta_max);

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

  Joystick.setXAxisRange(-(xlm_check-x_left),(xhm_check-x_right));
  Joystick.setYAxisRange(-(ylm_check-y_down),(yhm_check-y_up));

  Serial.println("Home position calibration complete.");

}

//***SPECIAL INITIALIZATION OPERATIONS***//

void Serial_Initialization(void) {
  while (!Serial1) {
    while (!Serial) {
      if (init_counter_A < 100) {
        delay(5);
        init_counter_A++;
      } else {
        break;
      }
    }
    if (init_counter_B < 100) {
      delay(5);
      init_counter_B++;
    } else {
      break;
    }
  }
  delay(10);
  Serial.println(init_counter_A);
  Serial.println(init_counter_B);
  Serial.println("Serial and Serial1 are good!");
}

void Set_Default(void) {

  int default_config_setup;
  int default_joystick_setting;
  int set_default;
  float default_comp_factor = 1.0;

  EEPROM.get(4, set_default);
  delay(10);

  if (set_default != 1) {

    default_config_setup = 0;
    EEPROM.put(0, default_config_setup);
    delay(10);

    default_joystick_setting = 4;
    EEPROM.put(2, default_joystick_setting);
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
