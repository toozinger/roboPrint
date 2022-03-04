// Libraries used in this program
#include <ESP_FlexyStepper.h>

// IO Pin assignments
const int MOTOR_STEP_PIN = 26;
const int MOTOR_DIRECTION_PIN = 33;
const int LED_PIN = 13;

// Stepper settings
long acceleration = 1000000;
long stepperPosition = 0;
long motorSpeed = 25000;

// Communication
const byte numChars = 32;
char receivedChars[numChars];   // Stores incoming characters from serial
char controlString[numChars];   // Stores string instructions from python
char tempChars[numChars];        // temporary array for use when parsing
float controlFloat;             // Stores float values
boolean newData = false;

// create the stepper motor object
ESP_FlexyStepper stepper;


// Global variables
unsigned long lastTime;


// Setup
// ***** ***** ***** ***** ***** ***** ***** ***** ***** *****
void setup(void) {

  // Try to start serial until started
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  // connect and configure the stepper motor to its IO PIns
  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
  stepper.setSpeedInStepsPerSecond(motorSpeed);
  stepper.setAccelerationInStepsPerSecondPerSecond(acceleration);
  stepper.setDecelerationInStepsPerSecondPerSecond(acceleration);

  // Not start the stepper instance as a service in the "background" as a separate task
  // and the OS of the ESP will take care of invoking the processMovement() task regularily so you can do whatever you want in the loop function
  stepper.startAsService();

  // Last time motor changed initiation
  lastTime = millis();

  // Setup for start/stop button and LED indicator
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

}

// Main Loop
// ***** ***** ***** ***** ***** ***** ***** ***** ***** *****
void loop() {

  // Read input from serial
  recvWithStartEndMarkers();

  // if new data, parse it
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() used in parseData() replaces the commas with \0
    parseData();
  }

  // Update motorSpeed
  if ((strcmp(controlString, "SPEED") == 0) && (newData == true)) {
    motorSpeed = controlFloat;
    stepper.setSpeedInStepsPerSecond(motorSpeed);
    Serial.print("New motor speed: "); Serial.print(controlFloat); Serial.println("");
  }

  // Update Acceleration
  if ((strcmp(controlString, "ACCEL") == 0) && (newData == true)) {
    stepper.setAccelerationInStepsPerSecondPerSecond(controlFloat);
    stepper.setDecelerationInStepsPerSecondPerSecond(controlFloat);
    Serial.print("New acceleration set: "); Serial.print(controlFloat); Serial.println("");
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

  strtokIndx = strtok(tempChars, ",");      // get the first part - the string
  strcpy(controlString, strtokIndx);        // copy it to messageFromPC

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
  
}
