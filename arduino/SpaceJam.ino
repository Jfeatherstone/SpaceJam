// SINGLE PARTICLE FAILURE SOFTWARE
// BY JACK FEATHERSTONE
// Makes use of several parts of Aaron's Linear_Actuator_Control code:
// /eno/abholt2/Linear_Actuator_Control.ino

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
// The load cells that are flipped have a negative reading in the direction they are
// normally strained
// Top boundary particle (flipped)
WheatstoneBridge wsb0(A12, CST_STRAIN_IN_MIN, CST_STRAIN_IN_MAX, CST_STRAIN_OUT_MIN, CST_STRAIN_OUT_MAX);
// Right actuator
WheatstoneBridge wsb1(A13, CST_STRAIN_IN_MIN, CST_STRAIN_IN_MAX, CST_STRAIN_OUT_MIN, CST_STRAIN_OUT_MAX);
// Left actuator (flipped)
WheatstoneBridge wsb2(A14, CST_STRAIN_IN_MIN, CST_STRAIN_IN_MAX, CST_STRAIN_OUT_MIN, CST_STRAIN_OUT_MAX);
// Bottom boundary particle
WheatstoneBridge wsb3(A15, CST_STRAIN_IN_MIN, CST_STRAIN_IN_MAX, CST_STRAIN_OUT_MIN, CST_STRAIN_OUT_MAX);

// Order: top, bottom, right, left
WheatstoneBridge wsbArr[4] = {wsb0, wsb3, wsb1, wsb2};
float loadCellSigns[4] = { -1, 1, 1, -1}; // See note about 'flipped' above

// This is the number of samples we average for the force measurement, since it
// would be too much to take a measurement on each step (would look very noisy)
int forceMeasurementAverageResolution = 100;


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
// Set the initial value of the stepper to be 0, but this will be calibrated later
int stepperPosition[2] = {0, 0};
int stepperSpeed = 10; // Default speed is 5 mm/s
int stepperMaxSpeed = 150;

// Parameters used when the positions of the steppers are reset
int stepperResetSpeed = 150;
int stepperResetPosition = 800;

// The amount to change the speed each button press
const int speedChangeIncrement = 10;
// The number of steps to move for each trial
const int numStepsPerTrial = 4000;

// Camera triggering
const int CAMERA_TRIGGER = 15;
Nikon D90(CAMERA_TRIGGER);
// Take a picture with D90.shotNow()

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
  steppers[0].setCurrentPosition(stepperPosition);
  steppers[0].setMaxSpeed(stepperMaxSpeed);
  steppers[0].setSpeed(stepperSpeed);
  steppers[1].setCurrentPosition(stepperPosition);
  steppers[1].setMaxSpeed(stepperMaxSpeed);
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
  DateTime now = rtc.now();
  char format[] = "YY-MM-DD-hh:mm:ss";
  String strDt = now.toString(format);
  //Serial.println(strDt);

  
  lcd.cursorTo(1, 0);
  lcd.printIn("Speed select:");
  lcd.cursorTo(2, 0);
  // See method below about printing a string to LCD
  lcdPrintStr("  ^  " + String(stepperSpeed) + " mm/s  v");

  int button = get_key(analogRead(LCD_ANALOG_BUTTON));

  bool runExperiment = false;
  int stepperToUse = -1;

  // Select button
  if (button == 5) {
    D90.shotNow();
  }
  
  // Up button
  if (button == 2) {
    
    stepperSpeed += speedChangeIncrement;
    
  } else
  // Down button
  if (button == 3) {
    
    stepperSpeed -= speedChangeIncrement;
    
  } else
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
        logFileName = "LC_" + logNum + ".csv";

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

    steppers[stepperToUse].move(numStepsPerTrial);

    int iterCount = 0;
    long averageForceTempArr[4][forceMeasurementAverageResolution];
    // Keep track of when the start time was
    long startTime = millis();
    long lastAverageTime = 0;
    
    while (steppers[stepperToUse].distanceToGo() != 0) {

        // We don't want to take a force measurement every step, so we downsample a bit
        averageForceTempArr[0][iterCount % forceMeasurementAverageResolution] = wsb0.measureForce();
        averageForceTempArr[1][iterCount % forceMeasurementAverageResolution] = wsb1.measureForce();
        averageForceTempArr[2][iterCount % forceMeasurementAverageResolution] = wsb2.measureForce();
        averageForceTempArr[3][iterCount % forceMeasurementAverageResolution] = wsb3.measureForce();
        
        if (iterCount % forceMeasurementAverageResolution == 0 && iterCount > 0) {
        
          // Calculate the average time
          long currentAverageTime =  (millis() - startTime) - ((millis() - startTime) - lastAverageTime) / 2;
          long averages[4] = {average(averageForceTempArr[0], forceMeasurementAverageResolution), average(averageForceTempArr[1], forceMeasurementAverageResolution), average(averageForceTempArr[2], forceMeasurementAverageResolution), average(averageForceTempArr[3], forceMeasurementAverageResolution)};
          // Write the time,force
          Serial.println(String(float(currentAverageTime) / 1000.f) + ", " + String(averages[0]) + ", " + String(averages[1]) + ", " + String(averages[2]) + ", " + String(averages[3]));
          if (foundLogFile) {
            logFile.println(String(float(currentAverageTime) / 1000.f) + ", " + String(averages[0]) + ", " + String(averages[1]) + ", " + String(averages[2]) + ", " + String(averages[3]));
          }
          lastAverageTime = currentAverageTime;
        }

      // TODO: record force data + other things

      steppers[stepperToUse].setSpeed(stepperSpeed);
      steppers[stepperToUse].runSpeed();
      
      iterCount++;
    }

    if (foundLogFile) {
      logFile.close();
      Serial.println("Log file written and closed!");
    }
    
    lcd.cursorTo(2, 0);
    lcd.printIn("Retracting");
    
    // Then move back
    steppers[stepperToUse].move(-numStepsPerTrial);
    while (steppers[stepperToUse].distanceToGo() != 0) {
      steppers[stepperToUse].setSpeed(-stepperResetSpeed);
      steppers[stepperToUse].runSpeed();
    }


  }
  // Various debug statements
  // Load cells
  //Serial.println(String(wsb0.measureForce()) + "," + String(wsb1.measureForce()) + "," + String(wsb2.measureForce()) + "," + String(wsb3.measureForce()));
  // Reset and LCD buttons
  //Serial.println(String(button) + ", " + String(digitalRead(RIGHT_RESET_BUTTON)) + ", " + String(digitalRead(LEFT_RESET_BUTTON)));


//  // Check if any buttons are pressed:
//  // We use the up and down buttons to start the stepper moving up or down
//  // and the left and right to select the speed.
//  // The select button is used to run the reset routine
//  if (buttons) {
//
//    // Run the reset routine
//    if (buttons & BUTTON_UP) {
//      // First we raise the cart until it clicks the button at the top
//      int stopButtonState = 0;
//      stepper.setMaxSpeed(resetRoutineStepperSpeed);
//
//      lcd.clear();
//      lcd.setCursor(0, 0);
//      lcd.print("Reset routine:");
//      lcd.setCursor(0, 1);
//      lcd.print("Finding top");
//
//      while (stopButtonState != HIGH) {
//        stopButtonState = digitalRead(RESET_ROUTINE_BUTTON);
//
//        // Move up a little
//        stepper.move(1);
//        stepper.runToPosition();
//      }
//
//      // Reset the position
//      stepperPosition = 0;
//      stepper.setCurrentPosition(0);
//
//      lcd.clear();
//      lcd.setCursor(0, 0);
//      lcd.print("Reset routine:");
//      lcd.setCursor(0, 1);
//      lcd.print("Descending");
//
//      // Now move back down
//      stepper.runToNewPosition(resetRoutineDescendPosition);
//      stepperPosition = resetRoutineDescendPosition;
//
//      // Put the speed back to what it was
//      stepper.setMaxSpeed(int(stepperSpeed / stepToSpeedConversion));
//
//      // Show a little completion message
//      lcd.clear();
//      lcd.setCursor(0, 0);
//      lcd.print("Reset complete");
//      delay(1000);
//      lcd.clear();
//    }
//
//    // Make the probe descend
//    if (buttons & BUTTON_DOWN) {
//
//      // First make sure that the SD card is inserted and all set to go
//      // If it isn't inserted, we still allow the probe to descend, just
//      // warn the user that force data won't be recording
//
//      // Setup the log file we will be writing the force data to
//      // Has the format <speed>mms-<number>.CSV where number will start at 0 and move up
//
//      // Defined as a global var so it can be used later to check we are okay to write
//      foundLogFile = false;
//
//      if (SD.begin(CHIP_SELECT)) {
//        // Only goes up to 100 since have 100 log files is already way too many
//        for (int i = 0; i < 100; i++) {
//          String logNum = String(i);
//          // Format the number properly
//          if (logNum.length() == 1)
//            logNum = "0" + logNum;
//
//          // Put it all together
//          logFileName = String(stepperSpeed) + "mms-" + logNum + ".CSV";
//
//          // Now check if this file already exists
//          if (!SD.exists(logFileName)) {
//            logFile = SD.open(logFileName, FILE_WRITE);
//            foundLogFile = true;
//            break;
//          }
//        }
//      }
//
//      if (!foundLogFile) {
//        Serial.println("SD Card error; force data will not be recorded!");
//        lcd.clear();
//        lcd.setCursor(0, 0);
//        lcd.print("Warning:");
//        lcd.setCursor(0, 1);
//        lcd.print("SD card error");
//        delay(1000);
//      }
//
//
//
//      // Set the target position
//      stepperPosition -= numStepsToDescend;
//      stepper.moveTo(stepperPosition);
//
//      // Display a message on the LCD
//      lcd.clear();
//      lcd.setCursor(0, 0);
//      lcd.print("Descending...");
//    }
//
//    // Decrease the speed
//    if (buttons & BUTTON_LEFT) {
//      stepperSpeed -= speedChangeIncrement;
//      stepper.setMaxSpeed(int(stepperSpeed / stepToSpeedConversion));
//      lcd.clear();
//    }
//
//    // Increase the speed
//    if (buttons & BUTTON_RIGHT) {
//      stepperSpeed += speedChangeIncrement;
//      stepper.setMaxSpeed(int(stepperSpeed / stepToSpeedConversion));
//      lcd.clear();
//    }
//
//    // Show the help menu
//    if (buttons & BUTTON_SELECT) {
//
//      // Show version
//      lcd.clear();
//      lcd.setCursor(0, 0);
//      lcd.print("EMPANADA");
//      lcd.setCursor(0, 1);
//      lcd.print("Firmware");
//      lcd.setCursor(15 - version.length(), 1); // 15 since the display has 16 pixels - 1 for the "v"
//      lcd.print("v" + version);
//
//      delay(5000);
//
//      // Show controls
//      lcd.clear();
//      lcd.setCursor(0, 0);
//      lcd.print("UP-Reset routine");
//      lcd.setCursor(0, 1);
//      lcd.print("DOWN-Descend");
//
//      delay(5000);
//
//      lcd.clear();
//      lcd.setCursor(0, 0);
//      lcd.print("LEFT-speed -");
//      lcd.setCursor(0, 1);
//      lcd.print("RIGHT-speed +");
//
//      delay(5000);
//    }
//  }
//
//  if (stepper.targetPosition() != stepper.currentPosition()) {
//    // Keep track of what step we are on, and define an array that we
//    // can average over to find the force
//    uint8_t stepCount = 0;
//    long averageForceTempArr[forceMeasurementAverageResolution];
//    // Keep track of when the start time was
//    long startTime = millis();
//
//    // Since we are averaging, we want the time we record to be at the
//    // center of the averaging samples, so we need to keep track of
//    // when the last average was
//    long lastAverageTime = 0;
//
//    // Loop until we reach the target destination
//    while (stepper.targetPosition() != stepper.currentPosition()) {
//      // Have the motor take a step at the constant speed we have defined before
//      stepper.run();
//
//      if (foundLogFile) {
//        // Read the force from the wheatstone bridge
//        averageForceTempArr[stepCount % forceMeasurementAverageResolution] = wsb.measureForce();
//
//        // We don't want to take a force measurement every step, so we downsample a bit
//        if (stepCount % forceMeasurementAverageResolution == 0 && stepCount > 0) {
//          // Calculate the average time
//          long currentAverageTime =  (millis() - startTime) - ((millis() - startTime) - lastAverageTime) / 2;
//          // Write the time,force
//          logFile.println(String(float(currentAverageTime) / 1000.f) + ", " + String(average(averageForceTempArr, forceMeasurementAverageResolution)));
//          lastAverageTime = currentAverageTime;
//          //Serial.println(average(averageForceTempArr, forceMeasurementAverageResolution));
//          //Serial.println(stepper.speed());
//
//        }
//      }
//      // Increment the step count
//      stepCount++;
//    }
//
//    // Close the log file, which should flush it as well
//    if (foundLogFile) {
//      logFile.close();
//    }
//  }

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
