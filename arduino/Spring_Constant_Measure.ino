// SPRING CONSTANT MEASUREMENT SOFTWARE
// BY JACK FEATHERSTONE
// Makes use of several parts of Aaron's Linear_Actuator_Control code:
// /eno/abholt2/Linear_Actuator_Control.ino

/*
 * Measures the force require to extend a linear actuator against a 
 * certain material, which can be used to confirm that it is linearly
 * elastic, and if so, measure the spring constant.
 * 
 * Adapted from https://github.com/Jfeatherstone/SpaceJam/blob/master/Arduino/SpaceJam.ino
 * May have vestigial pieces as a result.
 */
 
// May require  sudo chmod 666 /dev/ttyACM0 to get permissions
// to write to Arduino (on my personal laptop, though it is a mess, to be fair)

// Arduino IDE also requires Java >8, so make sure this is correct, otherwise the Serial Monitor may not work

// This allows us to use the stepper motor
// Installed from IDE Library interface
// Can also be found here: http://www.airspayce.com/mikem/arduino/AccelStepper/index.html
// (the link http://www.airspayce.com/mikem/arduino/AccelStepper/AccelStepper-1.61.zip downloads it instantly)
#include <AccelStepper.h>

// This allows us to write to the lcd screen
// Installed from: https://github.com/DFRobot/DFRobot_RGBLCD
#include <LCD4Bit_mod.h> 

// This allows us to use the strain gauge cell
// Installed from: https://www.robotshop.com/en/strain-gauge-load-cell-amplifier-shield-2ch.html
#include <WheatstoneBridge.h>

// This allows us to use the clock on the RST board (where the SD card goes)
// Installed from IDE Library interface
#include <RTClib.h>

// This allows us to write to the SD card on the RST board
// Installed from IDE Library interface
#include <SD.h>

// This allows us to trigger the camera to take a picture
// Installed from: https://github.com/dharmapurikar/Arduino
// Note that since this library is so old, you have to modify
// any references to "WProgram.h" to "Arduino.h" in the source code for it to work
#include <multiCameraIrControl.h>

//////////////////////////
// VARIABLE DEFINITIONS //
//////////////////////////
// For pins, see PIN SETUP section

// General program attributes

// The version of the program
// This should never be more than 6 characters to properly fit on the screen
const String version = "1.0";


// LCD Screen
// The brightness of the text can be adjusted using the potentiometer
// next to the reset button btw
LCD4Bit_mod lcd = LCD4Bit_mod(2); 

// SD Card
RTC_PCF8523 rtc; // Define the real time clock variable
const int LOG_INTERVAL = 1000;
const int ECHO_TO_SERIAL = 0;
const int WAIT_TO_START = 0;
// This is the actual file object that we will use to write
File logFile;
// We'll define this later, since we have to make sure we're not overwriting another log
// This is determined when a trial is started
String logFileName;
bool foundLogFile = false; // To keep track of whether we have somewhere to put the data

const char dateFormat[] = "YY-MM-DD-hh:mm:ss";

// Initialize the load cells
// This means we don't need to use the `linearCalibration` method in setup()
// Strain gauge cell initial calibration values
// I am not sure what the units on any of these values are, but I believe they are taken
// from the examples available for the Wheatstone bridge library
const int CST_STRAIN_IN_MIN = 350;       // Raw value calibration lower point
const int CST_STRAIN_IN_MAX = 650;       // Raw value calibration upper point
const int CST_STRAIN_OUT_MIN = 0;        // Weight calibration lower point
const int CST_STRAIN_OUT_MAX = 1000;     // Weight calibration upper point

WheatstoneBridge lwsb(A14, CST_STRAIN_IN_MIN, CST_STRAIN_IN_MAX, CST_STRAIN_OUT_MIN, CST_STRAIN_OUT_MAX);
WheatstoneBridge rwsb(A13, CST_STRAIN_IN_MIN, CST_STRAIN_IN_MAX, CST_STRAIN_OUT_MIN, CST_STRAIN_OUT_MAX);

WheatstoneBridge wsbArr[2] = {rwsb, lwsb};

const int MAX_LOAD_CELL_READING = 500;

// Setup the actuators through the H-bridges
// We use the wiring setup (colors of wires are in parenthesis):
// Right actuator:
// H-Bridge      |     Arduino
//    IN1             31 (Blue)
//    IN2             33 (Red)
//    IN3             35 (Green)
//    IN4             37 (Yellow)
// Left actuator:
// H-Bridge      |     Arduino
//    IN1             41 (Blue)
//    IN2             43 (White)
//    IN3             45 (Green)
//    IN4             47 (Yellow)
AccelStepper rStepper(AccelStepper::FULL4WIRE, 31, 33, 35, 37);
AccelStepper lStepper(AccelStepper::FULL4WIRE, 41, 43, 45, 47);
AccelStepper steppers[2] = {rStepper, lStepper};

const float DISTANCE_PER_STEP = 1;
const int MAX_STEPS_PER_EXPERIMENT = 4000;

// Set the initial value of the stepper to be 0, but this will be calibrated later
int stepperSpeed = 100; // Default speed is 5 mm/s

// Parameters used when the positions of the steppers are reset
int stepperResetSpeed = 150;
int stepperResetPosition = 800;

// The amount to change the speed each button press
const int speedChangeIncrement = 10;

//////////////////////////
//      PIN SETUP       //
//////////////////////////

// This is the pin to look for the SD card on
const int CHIP_SELECT = 10;
// Button that stops the reset routine
const int RIGHT_RESET_BUTTON_GND = 22;
const int RIGHT_RESET_BUTTON = 23;
const int LEFT_RESET_BUTTON_GND = 24;
const int LEFT_RESET_BUTTON = 25;
const int RESET_BUTTONS[2] = {RIGHT_RESET_BUTTON, LEFT_RESET_BUTTON};

// If this pin is connected to ground, the reset routine
// will be skipped.
const int SKIP_ACTUATOR_RESET_SHORT = 32;

// Where the LCD screen buttons can be read in;
// see getLCDKey()
const int LCD_ANALOG_BUTTON = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Start");

  // SD Card
  pinMode(CHIP_SELECT, OUTPUT);
  // LCD Buttons
  pinMode(LCD_ANALOG_BUTTON, INPUT);

  // Setup our ability to check when the actuator hits the wall
  pinMode(RIGHT_RESET_BUTTON_GND, OUTPUT);
  pinMode(LEFT_RESET_BUTTON_GND, OUTPUT);
  //digitalWrite(RIGHT_RESET_BUTTON_GND, LOW);
  //digitalWrite(LEFT_RESET_BUTTON_GND, LOW);
  pinMode(RIGHT_RESET_BUTTON, INPUT_PULLUP);
  pinMode(LEFT_RESET_BUTTON, INPUT_PULLUP);

  pinMode(SKIP_ACTUATOR_RESET_SHORT, INPUT_PULLUP);
  
  // Setup the LCD screen
  lcd.init();
  lcd.clear();
  lcd.cursorTo(0, 0); // (Column, Row), (x, y)
  lcd.printIn("Startup...");

  if (!rtc.begin()) {
    Serial.println("Error initializing RTC");
    delay(500);
    abort();
  }

  // You may need to manually set the time of the clock since it may
  // already be considered "running".
  // I would recommend using the university's official time:
  // https://www.time.ncsu.edu/
  //rtc.adjust(DateTime(2021, 11, 11, 11, 54, 0));
  // Another option is to compile the code on the computer that will control the
  // camera, and uncomment the following line
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // The important part is that both the arduino and the camera
  // have the same timestamps
  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Now start up the SD Card reader/writer
  if (!SD.begin(CHIP_SELECT)) {
    Serial.println("SD Card initialization failed, check CHIP_SELECT value, as well as SD Card formatting (must be FAT32/FAT16)");
    // We can run trials without the SD card, but we should warn the user
    // that no force data will be recorded    
    lcd.clear();
    lcd.cursorTo(0, 0);
    lcd.printIn("Check SD card");
    delay(1500);
  }

  // Setup the motor
  // First we set the current motor position to be zero
  steppers[0].setCurrentPosition(0);
  steppers[0].setMaxSpeed(stepperSpeed);
  steppers[0].setSpeed(stepperSpeed);
  steppers[1].setCurrentPosition(0);
  steppers[1].setMaxSpeed(stepperSpeed);
  steppers[1].setSpeed(stepperSpeed);

  // We can skip this process by connecting pin SKIP_ACTUATOR_RESET_SHORT
  // to ground
  if (digitalRead(SKIP_ACTUATOR_RESET_SHORT)) {
  
    String labels[2] = {"Right", "Left"};
    for (int i = 0; i < 2; i++) {
      lcd.clear();
      lcd.cursorTo(1, 0);
      lcd.printIn("Actuator reset:");
      lcd.cursorTo(2, 0);
      lcdPrintStr(labels[i] + "...");
  
      bool skipReset = false;
      
      // Move the actuators to start position
      while (digitalRead(RESET_BUTTONS[i]) && !skipReset) {
  //      if (get_key(analogRead(LCD_ANALOG_BUTTON) == 5))
  //        skipReset = true;
          
        steppers[i].setSpeed(-stepperResetSpeed);
        steppers[i].runSpeed();
      }
      steppers[i].setCurrentPosition(0);
  
      // Move the actuator back a little bit
      steppers[i].moveTo(stepperResetPosition);
      while (steppers[i].distanceToGo() != 0 && !skipReset) {
  //      if (get_key(analogRead(LCD_ANALOG_BUTTON) == 5))
  //        skipReset = true;
        steppers[i].setSpeed(stepperResetSpeed);
        steppers[i].runSpeed();
      }
    
      lcd.clear();
      lcd.cursorTo(1, 0);
      lcd.printIn("Actuator reset:");  delay(5000);
      lcd.cursorTo(2, 0);
      lcdPrintStr(labels[i] + "...done!");
      delay(1000);
  
    }

  }
  
  lcd.clear();

  Serial.println("Setup finished");
}


void loop() {

  lcd.cursorTo(1, 0);
  lcd.printIn("Press left or");
  lcd.cursorTo(2, 0);
  lcd.printIn("right to begin");

  int button = get_key(analogRead(LCD_ANALOG_BUTTON));

  bool runExperiment = false;
  int stepperToUse = -1;

  // Select button
  //if (button == 5) {
  //  
  //}
  
  // Up button
  //if (button == 2) {
  //      
  //} else
  // Down button
  //if (button == 3) {
  //      
  //} else
  
  // Left button
  if (button == 4) {
    // When pressing left, we push the particles to the left, which actually means using the right stepper
    runExperiment = true;
    stepperToUse = 0;

  } else
  // Right button
  if (button == 1) {
    // When pressing right, we push the particles to the right, which actually means using the left stepper
    runExperiment = true;
    stepperToUse = 1;
  }

  if (runExperiment) {

    // Defined as a global var so it can be used later to check we are okay to write
    foundLogFile = false;
    
    if (SD.begin(CHIP_SELECT)) {
      // Only goes up to 100 since have 100 log files is already way too many
      for (int i = 0; i < 100; i++) {
        String logNum = String(i);
        // Format the number properly
        if (logNum.length() == 1)
          logNum = "0" + logNum;

        // Put it all together
        // VERY IMPORTANT NOTE: this data shield has very strict rules about
        // file names when writing to an SD card: all characters will appear
        // in caps, and the name cannot be longer than 12 total characters
        // (so 8 characters before the extension). The library/code will not
        // give you an error if you violate these rules, but the file will just
        // not be created.
        logFileName = "SPR_" + logNum + ".csv";

        // Now check if this file already exists
        if (!SD.exists(logFileName)) {
          Serial.println(logFileName);
          logFile = SD.open(logFileName, FILE_WRITE);
          foundLogFile = true;
          break;
        }
      }
    }

    if (!foundLogFile) {
      Serial.println("SD card error; force data will not be recorded!");
      lcd.clear();
      lcd.cursorTo(1, 0);
      lcd.printIn("Warning:");
      lcd.cursorTo(2, 0);
      lcd.printIn("SD card error");
      delay(1000);
    } else
      Serial.println("Found log file");
    
    lcd.clear();
    lcd.cursorTo(1, 0);
    lcd.printIn("Running...");
    lcd.cursorTo(2, 0);
    lcd.printIn("Extending");

    // Tell the stepper to move the max number of steps, but
    // we will likely stop it before it reaches this point.
    // This is because the stepper.run() method requires that a step
    // is due for it to move, so this makes sure it is pretty much
    // always due to move.
    steppers[stepperToUse].move(MAX_STEPS_PER_EXPERIMENT);
    steppers[stepperToUse].setSpeed(stepperSpeed);
    
    int stepCount = 0;
    // Record the initial force reading
    float initialForceReading = wsbArr[stepperToUse].measureForce();
    
    while (steppers[stepperToUse].distanceToGo() != 0) {
      // This will take either 1 or 0 steps. If it takes 1, it will return true
      bool stepped = steppers[stepperToUse].runSpeed();
      
      // Measure the current reading from the appropriate load cell
      // and subtract off the initial value, since it might be centered
      // somewhere weird.
      float currForce = wsbArr[stepperToUse].measureForce() - initialForceReading;

      if (stepped) {

        if (foundLogFile) {
          logFile.println(String(stepCount * DISTANCE_PER_STEP) + ", " + String(currForce));
        }
        Serial.println(currForce);
 
        stepCount++;
      }
      
      // We don't want to break the load cell/stepper, so if the
      // force gets too high, we stop the experiment.
      if (currForce >= MAX_LOAD_CELL_READING){
        steppers[stepperToUse].stop();
        break;
      }
    }

    if (foundLogFile) {
      logFile.close();
      Serial.println("Log file written and closed!");
    }
    
    lcd.cursorTo(2, 0);
    lcd.printIn("Retracting");
    
    // Then move back
    steppers[stepperToUse].move(-stepCount);
    while (steppers[stepperToUse].distanceToGo() != 0) {
      steppers[stepperToUse].setSpeed(-stepperResetSpeed);
      steppers[stepperToUse].runSpeed();
    }

  }

}

// This function is used to take the average of the force sensor readings, since measuring
// the force every step would be a little too many points
long average(long arr[], int l) {
  long sum = 0;
  for (int i = 0; i < l; i++) {
    sum += arr[i];
  }
  return sum / long(l);
}


// This function converts the analog input value
// from the LCD shield into which (if any) of the
// buttons are currently pressed. Mostly taken from
// LCD4Bit_modExample
/*
 * None      - 0
 * Right     - 1
 * Up        - 2
 * Down      - 3
 * Left      - 4
 * Select    - 5
 */
int adc_key_val[5] ={30, 150, 360, 535, 760 };
int NUM_KEYS = 5;
int get_key(unsigned int input) {
  int k;
    
  for (k = 0; k < NUM_KEYS; k++) {
    if (input < adc_key_val[k]) {       
      return k+1;
    }
  }
    
  if (k >= NUM_KEYS)
    k = 0;     // No valid key pressed
    
  return k;
}

// This LCD library is really annoying in that it requires
// all messages to be character arrays (not Strings), but returning
// an array is a whole ordeal in c++, so it's better just to put it into
// its own method
void lcdPrintStr(String msg) {
  // I really love memory managed languages...
  char* arr = (char*) malloc(16);
  msg.toCharArray(arr, msg.length()+1);
  lcd.printIn(arr);
  free(arr);
  return;
}
