#include "QuickPID.h"
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <ESP_FlexyStepper.h> //https://github.com/pkerspe/ESP-FlexyStepper


// Motor setup
const int MOTOR_STEP_PIN = 12;
const int MOTOR_DIRECTION_PIN = 14;
float motorPosition = 0;

// Thermocouple
const int PIN_TC_DO = 26;
const int PIN_TC_CS = 15;
const int PIN_TC_CLK = 27;
float tempC = 0;
Adafruit_MAX31855 TC(PIN_TC_CLK, PIN_TC_CS, PIN_TC_DO);
const long TCreadSpeed = 1 * pow(10, 6);      // Time in microseconds between reading TC
unsigned long lastTCread;

// Heater PWM setup
const int PWMFreq = 5; //
const int PWMChannel = 0;
const int PWMResolution = 10;
const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) - 1);

// Serial communication
const byte numChars = 32;
char receivedChars[numChars];   // Stores incoming characters from serial
char controlString[numChars];   // Stores string instructions from python
char tempChars[numChars];        // temporary array for use when parsing
float controlFloat;             // Stores float values
bool newData = false;        // Stores when new data is read from serial
bool sendData = false;       // Used as flag to request current status be sent asap
bool moveStarted = false;     // Flag to know if a move has been started
const long serialPrintSpeed  = 2 * pow(10, 6); // Time in microseconds between printing to serial
unsigned long lastSerialSent = 0;
const long autoModeSerialPrintSpeed  = 0.5 * pow(10, 6); // Time in microseconds between printing to serial
unsigned long lastAutoModeSerialSent = 0;
bool debug = false;
bool autoMode = false;

// FANUC robot I/O Communication
const int PINS_ROBO_INPUT[6] = {23, 22, 21, 19, 18, 5};

// Temp controller
float tSetPoint, Output;
const int PIN_OUTPUT = 13;

float Kp = 5; // 125  // oldest 25
float Ki = 2.0; //3        // oldes: 10, old: 0.3
float Kd = 0.1;        // oldes 0.5

const int MAX_TEMP  = 250;   // Max temp allowable
const int MIN_TEMP = 0;     // Min temp allowable

//Specify PID
QuickPID myPID(&tempC, &Output, &tSetPoint);

// Stepper setup
ESP_FlexyStepper stepper;
#define STEPS_PER_MM 393
long acceleration = 100;        // mm/s^2
long stepperPosition = 0;
float motorSpeed = 1;          // mm/s
char movingStatus[10];        // Char array holding "true" or "false"

// ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** *****
void setup()
{

  Serial.begin(115200);

  //apply PID gains
  myPID.SetTunings(Kp, Ki, Kd);

  //turn the PID on
  myPID.SetMode(myPID.Control::automatic);

  // Set output limits
  myPID.SetOutputLimits(0, MAX_DUTY_CYCLE);

  // Setup temp control PWM output
  ledcSetup(PWMChannel, PWMFreq, PWMResolution);
  ledcAttachPin(PIN_OUTPUT, PWMChannel);

  // connect and configure the stepper motor to its IO PIns
  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
  stepper.setStepsPerMillimeter(STEPS_PER_MM);
  stepper.setSpeedInMillimetersPerSecond(motorSpeed);
  stepper.setAccelerationInMillimetersPerSecondPerSecond(acceleration);
  stepper.setDecelerationInMillimetersPerSecondPerSecond(acceleration);

  // Setup Robot I/O Pins
  for (int i = 0; i < 6; i++) {
    pinMode(PINS_ROBO_INPUT[i], INPUT_PULLUP);
  }

  // Not start the stepper instance as a service in the "background" as a separate task
  // and the OS of the ESP will take care of invoking the processMovement() task regularily so you can do whatever you want in the loop function
  stepper.startAsService();
}

// ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** *****
void loop()
{

  // Read input from serial
  recvWithStartEndMarkers();

  // if new data, parse it
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() used in parseData() replaces the commas with \0
    parseData();
  }

  // Toggle debug
  if ((strcmp(controlString, "DEBUG") == 0) && (newData == true)) {
    if (controlFloat == 0) {
      debug = false;
      Serial.println("Debug Disabled");
    }
    if (controlFloat == 1) {
      debug = true;
      Serial.println("Debug Enabled");
    }
  }

  // Print new data
  if ((newData == true) && (debug == true)) {
    Serial.println("***** New characters received *****");
    Serial.print("Full message: ");
    Serial.println(receivedChars);
    Serial.print("Control string: ");
    Serial.println(controlString);
    Serial.print("Control float: ");
    Serial.println(controlFloat);
  }

  // Update tSetPoint if requested
  if ((strcmp(controlString, "SETTEMP") == 0) && (newData == true)) {

    if ((controlFloat > MAX_TEMP) || (controlFloat < MIN_TEMP)) {
      Serial.print("Requested set temp: "); Serial.print(controlFloat); Serial.println(" is out of allowable range");
    }
    else {
      tSetPoint = controlFloat;
      Serial.print("New set point: "); Serial.print(controlFloat); Serial.println("");
      sendData = true;
    }
  }

  // Update Kp if requested
  if ((strcmp(controlString, "KP") == 0) && (newData == true)) {
    Kp = controlFloat;
    Serial.print("New Kp set: "); Serial.print(controlFloat); Serial.println("");
    myPID.SetTunings(Kp, Ki, Kd);
  }

  // Update Ki if requested
  if ((strcmp(controlString, "KI") == 0) && (newData == true)) {
    Ki = controlFloat;
    Serial.print("New Ki set: "); Serial.print(controlFloat); Serial.println("");
    myPID.SetTunings(Kp, Ki, Kd);
  }

  // Update Kd if requested
  if ((strcmp(controlString, "KD") == 0) && (newData == true)) {
    Kd = controlFloat;
    Serial.print("New Kd set: "); Serial.print(controlFloat); Serial.println("");
    myPID.SetTunings(Kp, Ki, Kd);
  }

  // Prints current PID settings
  if ((strcmp(controlString, "TUNINGS") == 0) && (newData == true)) {
    Serial.print("PID Tunings");
    Serial.print(", Kp, ");  Serial.print(Kp);
    Serial.print(", Ki, "); Serial.print(Ki);
    Serial.print(", Kd, "); Serial.print(Kd);
    Serial.println("");
  }

  // Update motorSpeed
  if ((strcmp(controlString, "SETSPEED") == 0) && (newData == true)) {
    motorSpeed = controlFloat;
    stepper.setSpeedInMillimetersPerSecond(motorSpeed);
    Serial.print("New motor speed: "); Serial.print(controlFloat); Serial.println("");
  }

  // Update Acceleration
  if ((strcmp(controlString, "SETACCEL") == 0) && (newData == true)) {
    acceleration = controlFloat;
    stepper.setAccelerationInStepsPerSecondPerSecond(acceleration);
    stepper.setDecelerationInStepsPerSecondPerSecond(acceleration);
    Serial.print("New acceleration set: "); Serial.print(controlFloat); Serial.println("");
  }

  // Increment move motor
  if ((strcmp(controlString, "MOVE") == 0) && (newData == true)) {
    stepperPosition += controlFloat;
    Serial.print("Moving "); Serial.print(controlFloat); Serial.print(" mm to position: "); Serial.print(stepperPosition); Serial.print(" mm"); Serial.println("");
    stepper.setTargetPositionRelativeInMillimeters(controlFloat);
    sendData = true; // Assures new info will be sent next time
    moveStarted = true;
  }

  // Absolute move motor
  if ((strcmp(controlString, "SETTARGET") == 0) && (newData == true)) {
    stepperPosition = controlFloat;
    Serial.print("Moving to position "); Serial.print(controlFloat); Serial.print(" [mm]"); Serial.println("");
    stepper.setTargetPositionInMillimeters(controlFloat);
    sendData = true; // Assures new info will be sent next time
    moveStarted = true;
  }

  // Change home location
  if ((strcmp(controlString, "SETHOME") == 0) && (newData == true)) {
    Serial.print("Current location set as home"); Serial.println("");
    stepper.setCurrentPositionAsHomeAndStop();
  }

  // Check status of movement
  if (stepper.motionComplete() == 0) {
    strncpy(movingStatus, "true", 10);
  }
  else {
    strncpy(movingStatus, "false", 10);

    // If a movement had been started and is now complete
    if (moveStarted) {
      Serial.println("Target position reached");
      sendData = true;
      moveStarted = false;
    }
  }

  // Stop motor
  if ((strcmp(controlString, "STOP") == 0) && (newData == true)) {
    Serial.print("Stopping!"); Serial.println("");
    stepper.setTargetPositionToStop();
  }


  // Turn on auto mode
  if ((strcmp(controlString, "AUTO") == 0) && (newData == true)) {

    if (controlFloat == 0) {
      autoMode = false;
      Serial.print("Auto Mode Disabled"); Serial.println("");
    }

    if (controlFloat == 1) {
      autoMode = true;
      Serial.print("Auto Mode Enabled"); Serial.println("");
    }

  }

  // Executes commands for running the stepper motor during auto mode
  if (autoMode == true) {

    executeAutoMode();

  }

  // Read temp sensor every set amount of time
  if ((micros() - lastTCread) > TCreadSpeed) {
    float TCread = TC.readCelsius();

    // Only saves TC value if valid reading taken
    if (!isnan(TCread)) {
      tempC = TCread;
    }
    lastTCread = micros();
  }

  // Update serial with info every set amount of time
  if (((micros() - lastSerialSent) > serialPrintSpeed) || (sendData)) {
    Serial.print("SetT, "); Serial.print(tSetPoint); Serial.print(", [C],");
    Serial.print(" ActT, "); Serial.print(tempC); Serial.print(", [C],");
    //    Serial.print(" Voltage, "); Serial.print(inputVoltage); Serial.print(", [V],");
    Serial.print(" Heater, "); Serial.print(float(Output / MAX_DUTY_CYCLE * 100)); Serial.print(", [%],");
    Serial.print(" Pos, "); Serial.print(stepper.getCurrentPositionInMillimeters()); Serial.print(", [mm],");
    Serial.print(" Speed, "); Serial.print(motorSpeed); Serial.print(", [mm/s],");
    Serial.print(" Accel, "); Serial.print(acceleration); Serial.print(", [mm/s/s],");
    Serial.print(" target dist, "); Serial.print((stepper.getDistanceToTargetSigned() / float(STEPS_PER_MM))); Serial.print(", [mm],");
    Serial.println("");
    lastSerialSent = micros();
    sendData = false;
  }

  // compute temperature PID
  myPID.Compute();

  // Write to temp output channel
  ledcWrite(PWMChannel, Output);

  delay(50);
  newData = false;
}


// ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** *****
// Function to execute auto mode of reading FANUC robot input, and setting stepper speed based on input
void executeAutoMode() {

  bool sendSerial = false;
  float mappedSpeed = 0;
  int robotInputs[6] = {0, 0, 0, 0, 0, 0};
  int robotInput = 0;

  // Check if should send serial info
  if ((micros() - lastAutoModeSerialSent) >= autoModeSerialPrintSpeed) {
    sendSerial = true;
    lastAutoModeSerialSent = micros();
  }

  if (sendSerial) {
    Serial.print("Input Pin State: ");
  }

  // Read Robot input pins
  for (int i = 0; i < 6; i++) {
    int temp = digitalRead(PINS_ROBO_INPUT[i]);
    if (sendSerial) {
      Serial.print(temp);
    }
    robotInputs[i] = temp;
    robotInput += temp * pow(2, i);
  }

  if (sendSerial) {
    Serial.print(", Input (dec): "); Serial.print(robotInput);
  }

  // Set extruder rate based on read value
  // If zero, set zero
  if (robotInput == 0) {
    stepper.setTargetPositionToStop();
    if (sendSerial) {
      Serial.print(", [auto] Stopping Extrusion"); Serial.println("");
    }
    return;
  }

  // If low range, map to range
  else if (robotInput < 5) {
    mappedSpeed = mapf(robotInput, 1, 4, -15, -1);
  }

  // If main range, map to range
  else if (robotInput < 55) {
    mappedSpeed = mapf(robotInput, 5, 54, -1, 10);
  }

  // If upper range, map to range
  else if (robotInput <= 63) {
    mappedSpeed = mapf(robotInput, 55, 63, 10, 30);
  }

  motorSpeed = mappedSpeed;

  // Execute motion by setting speed, and incrementing relative movement is close to finished

  // Set speed
  stepper.setSpeedInMillimetersPerSecond(abs(motorSpeed));

  // Set target location based on offset in correct direciton (dir) from current location
  if (tempC > 180) {
    motorPosition = stepper.getCurrentPositionInMillimeters();
    stepper.setTargetPositionInMillimeters(motorPosition + 10 * motorSpeed / abs(motorSpeed));
  }
  else {
    if (sendSerial) {
      Serial.print(", Temperature too low to auto extrude");
    }
  }
  if (sendSerial) {
    Serial.print(", Extrusion speed: "); Serial.print(motorSpeed); Serial.print(", Current pos: "); Serial.print(motorPosition); Serial.print(", Target pos: "); Serial.print(motorPosition + 10 * motorSpeed / abs(motorSpeed));
    Serial.println("");
  }

}

// ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** *****
// Copied from: https://forum.arduino.cc/index.php?topic=396450
void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

// ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** *****
// Copied from: https://forum.arduino.cc/index.php?topic=396450
void parseData() {      // split the data into its parts

  char * strtokIndx; // this is used by strtok() as an index

  // Checks for delimter "," if pointer returns null, no control float sent
  char * delimiterCheck;
  int delimiter = ',';
  delimiterCheck = strchr(tempChars, delimiter);

  strtokIndx = strtok(tempChars, ",");    // get the first part - the string
  strcpy(controlString, strtokIndx);      // copy it to messageFromPC

  // Converts controlstring to uppercase https://forum.arduino.cc/t/changing-case-of-char-array/147763/7
  for (int i = 0; i < numChars; i++ )  {
    if ( controlString[i] == 0 ) break;
    controlString[i] = controlString[i] & 0b11011111;
  }

  // Only convert control float if the delimiter "," existed
  if (delimiterCheck) {

    strtokIndx = strtok(NULL, ",");

    // Checks for null pointer (such as sending "move," by accident)
    if (strtokIndx == NULL) {
      Serial.println("Incompatible control float sent");
      newData = false;
    }

    else {
      controlFloat = atof(strtokIndx);
    }
  }


  // Otherwise, set control float to zero
  else {
    controlFloat = 0;
  }
}

// ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** ***** *****
// Custom map function to allow mapping with floats, taken from : https://forum.arduino.cc/index.php?topic=361046.0
double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
