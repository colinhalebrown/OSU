/* Gooper.INO
Gooper is the CONTROL box located at the control station
Gooper is ORANGE

This project is developed by OSU AIAA ESRA 2024
Colin Hale-Brown
Dexter Carpenter
*/

/* -------------------- GLOBAL VARS -------------------- */

// define pins
const int BUZ_PIN = 5;
const int LED1_PIN = 6;
const int LED2_PIN = 9;
const int RLY1_PIN = 10;
const int RLY2_PIN = 11;
const int BAT_PIN = A0;
const int CHG_PIN = A1;
const int ARM_PIN = A2;
const int CON_PIN = A3;

// voltages
int BatVoltage = 0;
int ChgVoltage = 0;
int ArmVoltage = 0;
int ConVoltage = 0;

/* -------------------- CORE 0 -------------------- */

void setup() {
  // define pin modes
  pinMode(BUZ_PIN , OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(RLY1_PIN, OUTPUT);
  pinMode(RLY2_PIN, OUTPUT);

  // begin serial
  Serial.begin(115200);

  // check status of arming switch

  // check status of fire button

  // ensure startup success
  while (ARMED || FIRING) {
    // blink that fucking armed LED like crazy

    // re-read the arming switch
    // re-read firing button
  }
}

// STANDARD LOOP
void loop() {
  // read and print battery voltage
  BatVoltage = analogRead(BAT_PIN); Serial.println(BatVoltage);
    
  // check ARMING SWITCH

  // check FIRE BUTTON

  // OTHER USER INPUTS...

  // if armed
    // while armed, prepare to fire
    // check for signal from firing switch
    // blink firing button
    // beep to indicate armed
  
  // indicate battery status of WestSystems105
  // indicate cont. status of WS105
  // indicate battery status of GOOPER
  // indicate radio connection status
}

/* -------------------- CORE 1 -------------------- */

void setup1() {

}

// RADIO LOOP
void loop1() {
  // listen for signals

  // ping WestSystems105
    // simple connection ping (handshake)
    // data handoff
      // battery data
      // status data
    // sending commands to WS105

  // recieve battery and other status info from WestSystems105

}

/* -------------------- FUNCTIONS -------------------- */

void pulse(PIN,PINState,beginMillis,interval) {
  // see Examples > 02.Digital > BlinkWithoutDelay
  // PIN - PIN ID of associated component
  // PINState - state of the associated component
  // beginMillis - starting time of the pulse

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (PINState == LOW) {
      PINState = HIGH;
    } else {
      PINState = LOW;
    }

  digitalWrite(PIN, PINState);
  }

}
