#include <Arduino.h>

#include <Stepper.h>

//#include <Servo.h>


//****************************************************
//testovacie funkcie na nastavenie motorov
//****************************************************
//#define TESTOVANIE true
  #ifdef TESTOVANIE
    #define TEST1 true
    //#define TEST2 true
    //#define TEST3 true
  #endif

//otoci osou o jednu otacku, pocka 1000ms a otoci osou naspat
void test_1_x();
void test_1_y();
//presunie vozik z jednej strany na druhu, potom naspat
void test_2_x();
void test_2_y();
//nakresli obdlznik
void test_3();
//******************************************************

//deklaracie funkcii na zaciatok 
void penUp();
void penDown();

//konstanty na priradenie pinov ku driverom
const uint8_t  COIL_A1_X = 16;//PIN_PH0;
const uint8_t  COIL_A2_X = 17;//PIN_PH1;
const uint8_t  COIL_B1_X = 15;//PIN_PJ0;
const uint8_t  COIL_B2_X = 14;//PIN_PJ1;

const uint8_t  COIL_A1_Y = 0;//PIN_PE0;
const uint8_t  COIL_A2_Y = 1;//PIN_PE1;
const uint8_t  COIL_B1_Y = 2;//PIN_PE4;
const uint8_t  COIL_B2_Y = 3;//PIN_PE5;

const uint8_t SERVO_PIN = 6;//PIN_PB4;

// Servo position for Up and Down 
const int PEN_UP = LOW;
const int PEN_DOWN = HIGH;


//###################################
//kalibracne konstanty stroja 
//##################################
//	SM15DD CD-ROM laser spindle motor
//	Bipolar stepper motor
//	5 V
//	10 ohm per winding

//	18 degrees per step, otocenie rotora na jeden STEP
const float DEGREES_PER_STEP_X = 18.0;
const float DEGREES_PER_STEP_Y = 18.0;

// revolutions per minute, otacky za minutu
const float REVOLUTION_PER_MINUTE_X = 100.0;
const float REVOLUTION_PER_MINUTE_Y = 100.0;

// STEPS_PER_REVOLUTION = 360 / angle_per_step
// pre SM15DD platí STEPS_PER_REVOLUTION = 360 /18 = 20

const float STEPS_PER_REVOLUTION_X = 360 / DEGREES_PER_STEP_X;
const float STEPS_PER_REVOLUTION_Y = 360 / DEGREES_PER_STEP_Y;
// step_per_second = rotations_per_minute * steps_per_revolution / 60
const float STEPS_PER_SECOND_X = REVOLUTION_PER_MINUTE_X * STEPS_PER_REVOLUTION_X / 60;
const float STEPS_PER_SECOND_Y = REVOLUTION_PER_MINUTE_Y * STEPS_PER_REVOLUTION_Y / 60;
// Motor steps to go 1 millimeter.
// Use test sketch to go 100 steps. Measure the length of line. 
// posun osi o milimetre na otáčku 
const float MM_PER_REVOLUTION_X = 3.0;
const float MM_PER_REVOLUTION_Y = 3.0;
// Calculate steps per mm. Enter here.
const float STEPS_PER_MILIMETER_X = STEPS_PER_REVOLUTION_X / MM_PER_REVOLUTION_X;
const float STEPS_PER_MILIMETER_Y = STEPS_PER_REVOLUTION_Y / MM_PER_REVOLUTION_Y;

// dlzka zaciatku osi v mimlimetroch
const float DLZKA_MIN_X = 0.0;
const float DLZKA_MIN_Y = 0.0;

// dlzka konca osi v mimlimetroch
const float DLZKA_MAX_X = 40.0;
const float DLZKA_MAX_Y = 40.0;
//############################################################
// koniec kalibracných konštánt stroja
//######################################################

// create servo object to control a servo 
//Servo penServo;  

// Initialize steppers for X- and Y-axis using this Arduino pins for the L293D H-bridge
Stepper myStepperX(STEPS_PER_REVOLUTION_X, COIL_A1_X, COIL_A2_X, COIL_B1_X, COIL_B2_X);
Stepper myStepperY(STEPS_PER_REVOLUTION_Y, COIL_A1_Y, COIL_A2_Y, COIL_B1_Y, COIL_B2_Y);

// Set to true to get debug output.
boolean verbose = false;

//  Needs to interpret 
//  G1 for moving
//  G4 P300 (wait 150ms)
//  M300 S30 (pen down)
//  M300 S50 (pen up)
//  Discard anything with a (
//  Discard any other command!


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
#define VERSION        (1)  // firmware version
#define BAUD           (57600)  // How fast is the Arduino talking?
#define MAX_BUF        (64)  // What is the longest message Arduino can store?
#define STEPS_PER_TURN (400)  // depends on your stepper motor.  most are 200.
#define MIN_STEP_DELAY (50.0)
#define MAX_FEEDRATE   (1000000.0/MIN_STEP_DELAY)
#define MIN_FEEDRATE   (0.01)

// for arc directions
#define ARC_CW          (1)
#define ARC_CCW         (-1)
// Arcs are split into many line segments.  How long are the segments?
#define MM_PER_SEGMENT  (10)

char  buffer[MAX_BUF];  // where we store the message until we get a newline

int   sofar;            // how much is in the buffer
float px, py;      // location

// speeds
float fr =     0;  // human version
long  step_delay;  // machine version

// settings
char mode_abs=1;   // absolute mode?

/**
 * delay for the appropriate number of microseconds
 * @input ms how many milliseconds to wait
 */
void pause(long ms) {
  delay(ms/1000);
  delayMicroseconds(ms%1000);  // delayMicroseconds doesn't work for values > ~16k.
}

/**
 * Set the feedrate (speed motors will move)
 * @input nfr the new speed in steps/second
 */
void feedrate(float nfr) {
  if(fr==nfr) return;  // same as last time?  quit now.

  if(nfr>MAX_FEEDRATE || nfr<MIN_FEEDRATE) {  // don't allow crazy feed rates
    Serial.print(F("New feedrate must be greater than "));
    Serial.print(MIN_FEEDRATE);
    Serial.print(F("steps/s and less than "));
    Serial.print(MAX_FEEDRATE);
    Serial.println(F("steps/s."));
    return;
  }
  step_delay = 1000000.0/nfr;
  fr = nfr;
}


/**
 * Set the logical position
 * @input npx new position x
 * @input npy new position y
 */
void position(float npx,float npy) {
  // here is a good place to add sanity tests
  px=npx;
  py=npy;
}


/**
 * Uses bresenham's line algorithm to move both motors
 * @input newx the destination x position
 * @input newy the destination y position
 **/
void line(float newx,float newy) {
  long i;
  long over= 0;
  
  long dx  = newx-px;
  long dy  = newy-py;
  int dirx = dx>0?1:-1;
  int diry = dy>0?-1:1;  // because the motors are mounted in opposite directions
  dx = abs(dx);
  dy = abs(dy);

  if(dx>dy) {
    over = dx/2;
    for(i=0; i<dx; ++i) {
      myStepperX.step(dirx);
      over += dy;
      if(over>=dx) {
        over -= dx;
        myStepperY.step(diry);
      }
      pause(step_delay);
    }
  } else {
    over = dy/2;
    for(i=0; i<dy; ++i) {
      myStepperY.step(diry);
      over += dx;
      if(over >= dy) {
        over -= dy;
        myStepperX.step(dirx);
      }
      pause(step_delay);
    }
  }

  px = newx;
  py = newy;
}


// returns angle of dy/dx as a value from 0...2PI
float atan3(float dy,float dx) {
  float a = atan2(dy,dx);
  if(a<0) a = (PI*2.0)+a;
  return a;
}


// This method assumes the limits have already been checked.
// This method assumes the start and end radius match.
// This method assumes arcs are not >180 degrees (PI radians)
// cx/cy - center of circle
// x/y - end position
// dir - ARC_CW or ARC_CCW to control direction of arc
void arc(float cx,float cy,float x,float y,float dir) {
  // get radius
  float dx = px - cx;
  float dy = py - cy;
  float radius=sqrt(dx*dx+dy*dy);

  // find angle of arc (sweep)
  float angle1=atan3(dy,dx);
  float angle2=atan3(y-cy,x-cx);
  float theta=angle2-angle1;
  
  if(dir>0 && theta<0) angle2+=2*PI;
  else if(dir<0 && theta>0) angle1+=2*PI;
  
  theta=angle2-angle1;
  
  // get length of arc
  // float circ=PI*2.0*radius;
  // float len=theta*circ/(PI*2.0);
  // simplifies to
  float len = abs(theta) * radius;

  int i, segments = ceil( len * MM_PER_SEGMENT );
 
  float nx, ny, angle3, scale;

  for(i=0;i<segments;++i) {
    // interpolate around the arc
    scale = ((float)i)/((float)segments);
    
    angle3 = ( theta * scale ) + angle1;
    nx = cx + cos(angle3) * radius;
    ny = cy + sin(angle3) * radius;
    // send it to the planner
    line(nx,ny);
  }
  
  line(x,y);
}

/**
 * Look for character /code/ in the buffer and read the float that immediately follows it.
 * @return the value found.  If nothing is found, /val/ is returned.
 * @input code the character to look for.
 * @input val the return value if /code/ is not found.
 **/
float parseNumber(char code,float val) {
  char *ptr=buffer;  // start at the beginning of buffer
  while((long)ptr > 1 && (*ptr) && (long)ptr < (long)buffer+sofar) {  // walk to the end
    if(*ptr==code) {  // if you find code on your walk,
      return atof(ptr+1);  // convert the digits that follow into a float and return it
    }
    ptr=strchr(ptr,' ')+1;  // take a step from here to the letter after the next space
  }
  return val;  // end reached, nothing found, return default val.
}

/**
 * write a string followed by a float to the serial line.  Convenient for debugging.
 * @input code the string.
 * @input val the float.
 */
void output(const char *code,float val) {
  Serial.print(code);
  Serial.println(val);
}


/**
 * print the current position, feedrate, and absolute mode.
 */
void where() {
  output("X",px);
  output("Y",py);
  output("F",fr);
  Serial.println(mode_abs?"ABS":"REL");
} /**
 * display helpful information
 */

void help() {
  Serial.print(F("GcodeCNCDemo2AxisV1 "));
  Serial.println(VERSION);
  Serial.println(F("Commands:"));
  Serial.println(F("G00 [X(steps)] [Y(steps)] [F(feedrate)]; - line"));
  Serial.println(F("G01 [X(steps)] [Y(steps)] [F(feedrate)]; - line"));
  Serial.println(F("G02 [X(steps)] [Y(steps)] [I(steps)] [J(steps)] [F(feedrate)]; - clockwise arc"));
  Serial.println(F("G03 [X(steps)] [Y(steps)] [I(steps)] [J(steps)] [F(feedrate)]; - counter-clockwise arc"));
  Serial.println(F("G04 P[seconds]; - delay"));
  Serial.println(F("G90; - absolute mode"));
  Serial.println(F("G91; - relative mode"));
  Serial.println(F("G92 [X(steps)] [Y(steps)]; - change logical position"));
  Serial.println(F("M18; - disable motors"));
  Serial.println(F("M100; - this help message"));
  Serial.println(F("M114; - report position and feedrate"));
  Serial.println(F("All commands must end with a newline."));
}

/**
 * Read the input buffer and find any recognized commands.  One G or M command per line.
 */
void processCommand() {
  //nastavenie pera
  int pen = 0;

  int cmd = parseNumber('G',-1);
  switch(cmd) {
  case  0:
  case  1: { // line
    feedrate(parseNumber('F',fr));
    line( parseNumber('X',(mode_abs?px:0)) + (mode_abs?0:px),
          parseNumber('Y',(mode_abs?py:0)) + (mode_abs?0:py) );

// ovladanie pera - laseru
    pen = parseNumber('S',-1);
    switch(pen) {
      case 1000:
        penUp();      
      case 0:
      default:
        penDown();
        break;
    }
// ovladanie pera - laseru    

    break;
    }
  case 2:
  case 3: {  // arc
      feedrate(parseNumber('F',fr));
      arc(parseNumber('I',(mode_abs?px:0)) + (mode_abs?0:px),
          parseNumber('J',(mode_abs?py:0)) + (mode_abs?0:py),
          parseNumber('X',(mode_abs?px:0)) + (mode_abs?0:px),
          parseNumber('Y',(mode_abs?py:0)) + (mode_abs?0:py),
          (cmd==2) ? -1 : 1);
      break;
    }
  case  4:  pause(parseNumber('P',0)*1000);  break;  // dwell
  case 90:  mode_abs=1;  break;  // absolute mode
  case 91:  mode_abs=0;  break;  // relative mode
  case 92:  // set logical position
    position( parseNumber('X',0),
              parseNumber('Y',0) );
    break;
  default:  break;
  }

  cmd = parseNumber('M',-1);
  switch(cmd) {
  case 18:  // disable motors
  //musi sa opravit
  //  disable();
    break;
  case 100:  help();  break;
  case 114:  where();  break;
  default:  break;
  }
}

/**
 * prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
 */
void ready() {
  sofar=0;  // clear input buffer
  Serial.println(F("ok"));  // signal ready to receive input
}

//  Raises pen
void penUp() { 
  digitalWrite(SERVO_PIN,PEN_UP);

  //musi sa opravit
 // delay(penDelay); 
  
//  digitalWrite(15, LOW);
//    digitalWrite(16, HIGH);
  if (verbose) { 
    Serial.println("Pen up!"); 
    
  } 
}
//  Lowers pen
void penDown() { 
  digitalWrite(SERVO_PIN,PEN_DOWN);
  //musi sa opravit
  //delay(penDelay);  
  
//  digitalWrite(15, HIGH);
//    digitalWrite(16, LOW);
  if (verbose) { 
    Serial.println("Pen down.");   
  } 
}

/**
 * First thing this machine does on startup.  Runs only once.
 */
/**********************
 * void setup() - Initialisations
 ***********************/
void setup() {
  //  Setup
  Serial.begin( 9600 );
  
  pinMode(SERVO_PIN, OUTPUT);

  delay(100);  
  
  // Decrease if necessary
  myStepperX.setSpeed(REVOLUTION_PER_MINUTE_X);
  myStepperY.setSpeed(REVOLUTION_PER_MINUTE_Y);  
  //
  penUp();

  //  Set & move to initial default position
  // TBD

  //  Notifications!!!
  Serial.println("Mini CNC Plotter alive and kicking!");
  Serial.print("X range is from "); 
  Serial.print(DLZKA_MIN_X); 
  Serial.print(" to "); 
  Serial.print(DLZKA_MAX_X); 
  Serial.println(" mm."); 
  Serial.print("Y range is from "); 
  Serial.print(DLZKA_MIN_Y); 
  Serial.print(" to "); 
  Serial.print(DLZKA_MAX_Y); 
  Serial.println(" mm."); 
  ready();
}

/**
 * After setup() this machine will repeat loop() forever.
 */
void loop() {
#ifdef TESTOVANIE
  //****************************************************
  //zaciatok testovania
  //****************************************************
    while (1) {
  #ifdef TEST1
      test_1_x();
      test_1_y();
  #endif

  #ifdef TEST2
      test_2_x();
      test_2_y();
  #endif  

  #ifdef TEST3
      test_3();
  #endif  
    }
  //****************************************************
  //koniec testovania
  //****************************************************    
#endif

  // listen for serial commands
  while(Serial.available() > 0) {  // if something is available
    char c=Serial.read();  // get it
    Serial.print(c);  // repeat it back so I know you got the message
    if(sofar<MAX_BUF-1) buffer[sofar++]=c;  // store it
    if((c=='\n') || (c == '\r')) {
      // entire message received
      buffer[sofar]=0;  // end the buffer so string functions work right
      Serial.print(F("\r\n"));  // echo a return character for humans
/*
Serial.println("-->> nacital som : ");
Serial.println(buffer);
Serial.println("\r\n");
*/
      processCommand();  // do something with the command
      ready();
    }
  }
}


//************************************************
//testovanie
//************************************************
//malo by otocit motor o jednu otacku (360 stupnou)
void test_1_x() {
  myStepperX.step(STEPS_PER_REVOLUTION_X);
  delay(1000);
  myStepperX.step(-STEPS_PER_REVOLUTION_X);
}

//malo by otocit motor o jednu otacku (360 stupnou)
void test_1_y() {
  myStepperY.step(STEPS_PER_REVOLUTION_Y);
  delay(1000);
  myStepperY.step(-STEPS_PER_REVOLUTION_Y);
}

//malo by posuvat os x po dlzke DLZKA_MAX_X
void test_2_x() {
  myStepperX.step((int)(STEPS_PER_MILIMETER_X * DLZKA_MAX_X));
  delay(1000);
  myStepperX.step(-(int)(STEPS_PER_MILIMETER_X * DLZKA_MAX_X));
}

//malo by posuvat os x po dlzke DLZKA_MAX_Y
void test_2_y() {
  myStepperY.step((int)(STEPS_PER_MILIMETER_Y * DLZKA_MAX_Y));
  delay(1000);
  myStepperY.step(-(int)(STEPS_PER_MILIMETER_Y * DLZKA_MAX_Y));
}


//malo by nakreslit obdlznik o dlzke stran DLZKA_MAX_X a DLZKA_MAX_Y
void test_3() {
  myStepperX.step((int)(STEPS_PER_MILIMETER_X * DLZKA_MAX_X));
  delay(1000);
  myStepperY.step((int)(STEPS_PER_MILIMETER_Y * DLZKA_MAX_Y));
  delay(1000);
  myStepperX.step(-(int)(STEPS_PER_MILIMETER_X * DLZKA_MAX_X));
  delay(1000);
  myStepperY.step(-(int)(STEPS_PER_MILIMETER_Y * DLZKA_MAX_Y));
  delay(1000);
}
