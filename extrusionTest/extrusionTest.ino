/********************************************************
   PID Basic Example
   Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include "QuickPID.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <ESP_FlexyStepper.h>

Adafruit_ADS1115 ads1115; // Construct an ads1115 
const float multiplier = 0.1875F;
//const float fiveTo3Mofier = 5/3.3F;
float inputVoltage = 0;
float tempC = 0;
int16_t adc0;

// Motor I/O Pin assignments
const int MOTOR_STEP_PIN = 26;
const int MOTOR_DIRECTION_PIN = 33;
const int LED_PIN = 13;

// Stepper settings
long acceleration = 1000000;
long stepperPosition = 0;
long motorSpeed = 25000;

// Heater PWM setup
const int PWMFreq = 5; // 
const int PWMChannel = 0;
const int PWMResolution = 10;
const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) - 1);

// Communication
const byte numChars = 32;
char receivedChars[numChars];   // Stores incoming characters from serial
char controlString[numChars];   // Stores string instructions from python
char tempChars[numChars];        // temporary array for use when parsing
float controlFloat;             // Stores float values
boolean newData = false;   

// Control
const long TCreportSpeed  = 1*pow(10,6);  //Time in microseconds between printing TC values
bool sendData = false;  
bool debug = false;
unsigned long lastTCSent;

#define PIN_OUTPUT 14

//Define Variables we'll be connecting to
float tSetPoint, Output;

float Kp = 125;  // oldest 25 
float Ki = 3.0;        // oldes: 10, old: 0.3
float Kd = 0.1;        // oldes 0.5

//Specify PID links
QuickPID myPID(&tempC, &Output, &tSetPoint);

// create the stepper motor object
ESP_FlexyStepper stepper;

void setup()
{

  Serial.begin(115200);

  ads1115.begin(0x48);  // Initialize ads1115 at address
  //initialize the variables we're linked to

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
  stepper.setSpeedInStepsPerSecond(motorSpeed);
  stepper.setAccelerationInStepsPerSecondPerSecond(acceleration);
  stepper.setDecelerationInStepsPerSecondPerSecond(acceleration);

  // Not start the stepper instance as a service in the "background" as a separate task
  // and the OS of the ESP will take care of invoking the processMovement() task regularily so you can do whatever you want in the loop function
  stepper.startAsService();
}

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

   // Turn toggle debug
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
  if ((strcmp(controlString, "TEMPSET") == 0) && (newData == true)) {
    tSetPoint = controlFloat;
    Serial.print("New set point: "); Serial.print(controlFloat); Serial.println("");
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
  if ((strcmp(controlString, "SPEED") == 0) && (newData == true)) {
    motorSpeed = controlFloat;
    stepper.setSpeedInStepsPerSecond(motorSpeed);
    Serial.print("New motor speed: "); Serial.print(controlFloat); Serial.println("");
  }

  // Print current motorSpeed
  if ((strcmp(controlString, "SPEEDX") == 0) && (newData == true)) {
    Serial.print("Motor speed: "); Serial.print(motorSpeed); Serial.println("");
  }

  // Update Acceleration
  if ((strcmp(controlString, "ACCEL") == 0) && (newData == true)) {
    acceleration = controlFloat;
    stepper.setAccelerationInStepsPerSecondPerSecond(acceleration);
    stepper.setDecelerationInStepsPerSecondPerSecond(acceleration);
    Serial.print("New acceleration set: "); Serial.print(controlFloat); Serial.println("");
  }

  // Print current Acceleration
  if ((strcmp(controlString, "ACCELX") == 0) && (newData == true)) {
    Serial.print("Acceleration set at: "); Serial.print(acceleration); Serial.println("");
  }

  // Move motor
  if ((strcmp(controlString, "MOVE") == 0) && (newData == true)) {
    stepperPosition += controlFloat;
    Serial.print("Moving: "); Serial.print(controlFloat); Serial.print(" steps to position: "); Serial.print(stepperPosition); Serial.println("");
    stepper.setTargetPositionInSteps(stepperPosition);
  }

  // Stop motor
  if ((strcmp(controlString, "STOP") == 0) && (newData == true)) {
    Serial.print("Stopping!"); Serial.println("");
    stepper.setTargetPositionToStop();
  }

  

  adc0 = ads1115.readADC_SingleEnded(0);
  inputVoltage = adc0 * multiplier; //

  // Copied from Excel 2nd order interpolation from raw data provided here: https://e3d-online.zendesk.com/hc/en-us/articles/360017153677-E3D-PT100-Amplifier-Guide
  // tempC = 2*pow(10,-8)*pow(inputVoltage,3) - 9*pow(10,-5)*pow(inputVoltage,2) + 0.2148*inputVoltage - 176.86;
  tempC = 4.46*pow(10,-5)*pow(inputVoltage,2) - 0.0485*inputVoltage + 15.545;


  // Update serial with info every set amount of time
  if ((micros() - lastTCSent) > TCreportSpeed) {
    Serial.print("TempSet, "); Serial.print(tSetPoint); Serial.print(", [C],");
    Serial.print(" TempAct, "); Serial.print(tempC); Serial.print(", [C],"); 
    Serial.print(" Voltage, "); Serial.print(inputVoltage); Serial.print(", [V],");
    Serial.print(" Duty Cycle, "); Serial.print(float(Output/MAX_DUTY_CYCLE*100)); Serial.print(", [%],");
    Serial.println("");
    lastTCSent = micros();
  }

  // compute PID
  myPID.Compute();

  // Write to temp output channel
  ledcWrite(PWMChannel, Output);

  delay(100);
  newData = false;
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

  strtokIndx = strtok(tempChars, ",");     // get the first part - the string
  strcpy(controlString, strtokIndx); // copy it to messageFromPC

  // Converts controlstring to uppercase https://forum.arduino.cc/t/changing-case-of-char-array/147763/7
  for (int i = 0; i < numChars; i++ )  {
    if ( controlString[i] == 0 ) break;
    controlString[i] = controlString[i] & 0b11011111;
  }

  // Only convert control float if a "," existed
  if (delimiterCheck) {

    strtokIndx = strtok(NULL, ",");
    controlFloat = atof(strtokIndx);
    
  }

  // Otherwise, set control float to zero
  else {
    controlFloat = 0;
  }
}
