/*
  This code controls the in/output of two pushbuttons, and three LEDs, to act as an Emergency Stop.

  BUTTONS:
  The two pushbuttons act as 'Emergency' (Red), and 'Reset (Yellow). When the 'Emergency' button is pressed, it will latch and render the system unsafe.
  Only when the 'Emergency' button is pressed again, will the latch status be removed. Once this occurs, the system needs to be reset via the 'Reset' button.
  Once the system is reset, the safety output will return to a 'safe' designation.

  LEDs:
  The green LED represents system safety. 
    If an 'Emergency' button is pressed, this LED will turn off.
  The red LED represents an active safety stop. 
    If the 'Emergency' button is pressed, this will turn on. It can only be turned off by pressing the 'Emergency' button once more.
  The yellow LED represents the need for a safety reset. 
    If the 'Reset' button is pressed whilst the yellow LED is active, the LED will turn off, though only if the red LED is already off.

  STATES:
              | Grn | Red | Yel |
  Safe        |  1  |  0  |  0  | Can only change by pressing 'Emergency' button
  Unsafe      |  0  |  1  |  1  | Can only change by pressing 'Emergency' button
  Reset Needed|  0  |  0  |  1  | Can only change by pressing 'Reset' button

*/

// constants won't change. They're used here to set pin numbers:
const int buttonPinYel = A1;       // the number of the yellow pushbutton pin
const int ledPinYel =  7;          // the number of the yellow LED pin
const int buttonPinRed = A0;       // the number of the red pushbutton pin
const int ledPinRed = 8;           // the number of the red LED pin
const int ledPinGrn = 10;          // the number of the green LED pin
const int safeOutPin = 1;          // the number of the comm. pin for safety feedback

// variables will change:
double buttonStateYel = 0;         // variable for reading the pushbutton status
double buttonStateRed = 0;         // variable for reading the pushbutton status
int ledStateYel = 0;               // variable for keeping memory of LED status
int ledStateRed = 0;               // variable for keeping memory of LED status
int ledStateGrn = 0;               // variable for keeping memory of LED status
int safeOut = 0;                   // Safety output variable, used by MATLAB to control movement of robot. 1 == Safe, 0 == Not safe

void setup() {
  Serial.begin(9600);
      // initialize the LED pin as an output:
  pinMode(ledPinYel, OUTPUT);
      // initialize the pushbutton pin as an input:
  pinMode(buttonPinYel, INPUT);
      // initialize the LED pin as an output:
  pinMode(ledPinRed, OUTPUT);
      // initialize the pushbutton pin as an input:
  pinMode(buttonPinRed, INPUT);
      // initialize the LED pin as an output:
  pinMode(ledPinGrn, OUTPUT);
      // status 'safe' - enable green LED & write safe output as 1
  digitalWrite(ledPinGrn, !digitalRead(ledPinGrn));
  ledStateGrn = 1;
  safeOut = 1;

}

void loop() {
  // read the pushbutton values:
  buttonStateYel = analogRead(buttonPinYel);
  buttonStateRed = analogRead(buttonPinRed);

  // check if the pushbutton is pressed. If it is, the buttonState is 1023:
  if (buttonStateRed == 1023 && ledStateRed != 1 && ledStateYel != 1 && ledStateGrn != 0) { // 'Emergency' button pressed, Red & Yel LEDs off, Grn LED on
    // toggle the LED & safe out states:
    safeOut = 0;
    digitalWrite(ledPinGrn, !digitalRead(ledPinGrn));
    ledStateGrn = 0;
    digitalWrite(ledPinRed, !digitalRead(ledPinRed));
    ledStateRed = 1;
    digitalWrite(ledPinYel, !digitalRead(ledPinYel));
    ledStateYel = 1;
    // debounce:
    delay(150);
  } else if (buttonStateRed == 1023 && ledStateRed == 1 && ledStateYel == 1) { // 'Emergency' button pressed, Red & Yel LEDs on
    digitalWrite(ledPinRed, !digitalRead(ledPinRed));
    ledStateRed = 0;
    delay(150);
  } else if (buttonStateYel == 1023 && ledStateRed != 1 && ledStateYel != 0) { // 'Reset' button pressed, Yel LED on
    digitalWrite(ledPinYel, !digitalRead(ledPinYel));
    ledStateYel = 0;
    delay(150);
  } else if (ledStateRed == 0 && ledStateYel == 0 && ledStateGrn == 0) { // All LEDs off, meaning system can now be considered safe
    digitalWrite(ledPinGrn, !digitalRead(ledPinGrn));
    ledStateGrn = 1;
    safeOut = 1;
  }
}