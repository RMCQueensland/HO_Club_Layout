/* RMCQ Control Module A1 Servo Control Unit - Module A1
  By David Lowe 20160330

  This control unit is to control by switch 4 Servo controlled turnouts. The switch is 5 momentary Normal push-button
  type used to control the turnout direction. Point numbering is determined by the HO Layout design documents.
  these numbers are based on the sequential number within the scope of the panel that immediately address the point.

  Theory of Operation:
  The system on start up with set the position of each servo into a known initial position. In most cases this will
  set each point to Mains route or Normal condition. This is needed so the system is aware of the position.

  On command to move the point position, the system will determine if the servo needs to be moved. In other words it
  checks to see if the commanded position is the current position. If not, the servo will be moved into the commanded
  position.

  The control input and outputs are as follows:

  Pin Configuration:
  T100: Servo output = D10, T110: Servo output = D11, T101: Servo output = D12, T102: Servo output = D13,
  pushbutton input = D2 (Yard Out), pushbutton input = D3 (Departure), pushbutton input = D4 (Arrival),
  pushbutton input = D5 (Main Inner), pushbutton input = D6 (Main Outer),

  Credits:
  Program Debounce created 21 November 2006,  by David A. Mellis, modified 30 Aug 2011, by Limor Fried modified 28 Dec 2012,
  by Mike Walters

  Change log:
  RMCQ Control Module A1-2 Updated: 20151130
  - Amended point numbering and turnout throw labelling
  - Signals programing removed
  RMCQ Control Module A1-3 Updated: 20160412
  - Changed rogram to deal with toggling of points instead of going from one side to the other. This will allow setting the
  throw value to allow for
  physical position of the servo assembly compared to the point direction.
  RMCQ Control Module A1-4 Updated: 20160914
  - Change the servo change procedure to add the servo attach and detach.
  RMCQ Control Module A1-5 Update 20170729
  - Change program to be inline with Control Template which includes the RMCQ library.
  - Change to libraries to make referening them simpler by removing the current and end positions
  statements. Now the library is aware and auto calculates the correct starting and end positions
  based on the initial servo variables within the program.
*/

#include <Servo.h>
#include <RMCQControl.h>
Servo servo100;    // create servo object to control a servo
Servo servo110;
Servo servo101;
Servo servo102;

// constants won't change.
const int inputYardOut = 2;      //Yard out
const int inputDeparture = 3;      //Departure
const int inputArrival = 4;      //Arrival
const int inputMainInner = 5;      //Main Inner
const int inputMainOuter = 6;      //Main Outer

RMCQControl rmcqControl = RMCQControl();

// Servo variables
int startPos = 80;
int throwNormal = 60;
int throwReverse = 30;
int timeBetweenServoChange = 20;
int servoRateOfChange = 30;

// Trunout Normal and Reverse positions for each servo used. Set to each individual Servo and physical setup.
int opPin100 = 10;
int opPin110 = 11;
int opPin101 = 12;
int opPin102 = 13;

// Servo variables
// the Turnout is in the Normal state (HIGH). LOW for Reverse.
boolean currentServoState100 = HIGH;
boolean currentServoState110 = HIGH;
boolean currentServoState101 = HIGH;
int offSet101 = 10;
boolean currentServoState102 = LOW;

// Variables will change:
int inputStateYardOut;             // the current reading from the input Yard out
int lastinputStateYardOut = HIGH;   // the previous reading from the input Yard Out
int inputStateDeparture;             // the current reading from the input Departure
int lastinputStateDeparture = HIGH;   // the previous reading from the input Departure
int inputStateArrival;             // the current reading from the input Arrival
int lastinputStateArrival = HIGH;   // the previous reading from the input Arrival
int inputStateMainInner;             // the current reading from the input Main Inner
int lastinputStateMainInner = HIGH;   // the previous reading from the input Main Inner
int inputStateMainOuter;             // the current reading from the input Main Outer
int lastinputStateMainOuter = HIGH;   // the previous reading from the input Main Outer

// the following variables athe long's because the time, measured in miliseconds, will quickly become a bigger number Ran can be stored in an int.
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 30;    // the debounce time; increase if the output flickers


void setup() {
  Serial.begin(9600);
  // Set Servo global variables
  rmcqControl.setUp(startPos, throwNormal, throwReverse, timeBetweenServoChange, servoRateOfChange);

  // set pin configuration
  pinMode(inputYardOut, INPUT_PULLUP);
  pinMode(inputDeparture, INPUT_PULLUP);
  pinMode(inputArrival, INPUT_PULLUP);
  pinMode(inputMainInner, INPUT_PULLUP);
  pinMode(inputMainOuter, INPUT_PULLUP);

  // set initial state for servos
  // Set Point 100
  rmcqControl.center(servo100, startPos, opPin100);
  rmcqControl.changeNormal(LOW, servo100, opPin100);  // Set the Servo to Normal
  currentServoState100 = HIGH;
  // Set Point 110
  rmcqControl.center(servo110, startPos, opPin110);
  rmcqControl.changeNormal(LOW, servo110, opPin110);    // Set the Servo to Normal
  currentServoState110 = HIGH;
  // Set Point 101
  rmcqControl.center(servo101, startPos, opPin101);
  rmcqControl.changeNormal(LOW, servo101, opPin101);    // Set the Servo to Normal
  currentServoState101 = HIGH;
  // Set Point 102
  rmcqControl.center(servo102, startPos, opPin102);
  rmcqControl.changeNormal(LOW, servo102, opPin102);    // Set the Servo to normal
  currentServoState102 = HIGH;
}
//RMCQ library functions
//void RMCQControl::changeNormal(boolean currentState, Servo servoName, int outputPin)
//void RMCQControl::changeReverse(boolean currentState, Servo servoName, int outputPin)

/*    Route         Point
   _____ __________ 102
   _____\__________ 101, 110
          \
   ________\_______ 100
*/

void loop() {
  // Read Yard Out input Pin2
  // On Yard Out: 100: NA, 110: NA, 101: NA, 102: N.
  int inputYardOutState = digitalRead(inputYardOut);
  if (inputYardOutState != lastinputStateYardOut) {
    lastDebounceTime = millis();
    Serial.println("Yard Out activated");
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (inputYardOutState != inputStateYardOut) {
      inputStateYardOut = inputYardOutState;
      if (inputYardOutState == LOW) {
        setYardOut();
      }
    }
  }
  lastinputStateYardOut = inputYardOutState;

  // Read Departure input Pin3
  // On Departure: 100: NA, 110: N, 101: R, 102: R.
  int inputDepartureState = digitalRead(inputDeparture);
  if (inputDepartureState != lastinputStateDeparture) {
    lastDebounceTime = millis();
    Serial.println("Departure activated");
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (inputDepartureState != inputStateDeparture) {
      inputStateDeparture = inputDepartureState;
      if (inputDepartureState == LOW) {
        setDeparture();
      }
    }
  }
  lastinputStateDeparture = inputDepartureState;

  // Read Arrival input Pin4
  // On Arrival: 100: R, 110: R, 101: R, 102: R.
  int inputArrivalState = digitalRead(inputArrival);
  if (inputArrivalState != lastinputStateArrival) {
    lastDebounceTime = millis();
    Serial.println("Arrival activated");
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (inputArrivalState != inputStateArrival) {
      inputStateArrival = inputArrivalState;
      if (inputArrivalState == LOW) {
        setArrival();
      }
    }
  }
  lastinputStateArrival = inputArrivalState;

  // Read Main Inner (Pin5) or Main Outer (Pin6)
  // On Main Inner: 100: N, 110: N, 101: N, 102: N.
  int inputMainInnerState = digitalRead(inputMainInner);
  int inputMainOuterState = digitalRead(inputMainOuter);
  if (inputMainInnerState != lastinputStateMainInner || inputMainOuterState != lastinputStateMainOuter) {
    // Reset the debouncing timer
    lastDebounceTime = millis();
    Serial.println("Main Inner or Outer activated");
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (inputMainInnerState != inputStateMainInner || inputMainOuterState != inputStateMainOuter) {
      inputStateMainInner = inputMainInnerState;
      inputStateMainOuter = inputMainOuterState;
      if (inputMainInnerState == LOW || inputMainOuterState == LOW) {
        setMain();
      }
    }
  }
  lastinputStateMainInner = inputMainInnerState;
  lastinputStateMainOuter = inputMainOuterState;
}



void setYardOut() {// On Yard Out: 100: NA, 110: NA, 101: NA, 102: N.
  // Point 102 to Normal
  rmcqControl.changeNormal(currentServoState102, servo102, opPin102);
  currentServoState102 = HIGH;
}

void setMain() { // On Main Inner: 100: N, 110: N, 101: N, 102: N.
  // Point 100 to Reverse
  rmcqControl.changeNormal(currentServoState100, servo100, opPin100);
  currentServoState100 = HIGH;
  // Point 110 to Normal
  rmcqControl.changeNormal(currentServoState110, servo110, opPin110);
  currentServoState110 = HIGH;
  // Point 101 to Normal
 rmcqControl.changeNormal(currentServoState101, servo101, opPin101);
  currentServoState101 = HIGH;
  // Point 102 to Reverse
  rmcqControl.changeNormal(currentServoState102, servo102, opPin102);
  currentServoState102 = HIGH;
}

void setArrival() {// On Arrival: 100: R, 110: R, 101: R, 102: R.
  // Point 100 to Normal
  rmcqControl.changeReverse(currentServoState100, servo100, opPin100);
  currentServoState100 = LOW;
  // Point 110 to Reverse
  rmcqControl.changeReverse(currentServoState110, servo110, opPin110);
  currentServoState110 = LOW;
  // Point 101 to Reverse
  rmcqControl.changeReverse(currentServoState101, servo101, opPin101);
  //changeReverse101();
  currentServoState101 = LOW;
  // Point 102 to Normal
  rmcqControl.changeReverse(currentServoState102, servo102, opPin102);
  currentServoState102 = LOW;
}

void setDeparture() {// On Departure: 100: NA, 110: N, 101: R, 102: R.
  // Point 110 to Normal
  rmcqControl.changeNormal(currentServoState110, servo110, opPin110);
  currentServoState110 = HIGH;
  // Point 101 to Reverse
  rmcqControl.changeReverse(currentServoState101, servo101, opPin101);
 // changeReverse101();
  currentServoState101 = LOW;
  //Point 102 to Normal
  rmcqControl.changeReverse(currentServoState102, servo102, opPin102);
  currentServoState102 = LOW;
}

void changeReverse101() {
  int x = 1;
  servo101.attach(opPin101);
  if (currentServoState101 == HIGH) {
    for (x = startPos + throwNormal; x >= (startPos-throwReverse-10); x --) {
      servo101.write(x);
      delay(servoRateOfChange);
    }
  }
  else {
    // do nothing as OP already Reverse
  }
  servo101.detach();
  delay(timeBetweenServoChange);
}

