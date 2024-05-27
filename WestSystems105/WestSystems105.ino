/* WestSystems105.INO
WS105 is the PAD box located at the ROCKET WITH EXPLOSIVES
WS105 is BLACK

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

// STANDARD SETUP
void setup() {
  // put your setup code here, to run once:
  pinMode(BUZ_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(RLY1_PIN, OUTPUT);
  pinMode(RLY2_PIN, OUTPUT);

  // begin serial
  Serial.begin(115200);

}

// STANDARD LOOP
void loop() {
  // collect WS105 battery voltage
  BatVoltage = analogRead(BAT_PIN); Serial.println(BatVoltage);

  // collect charge battery voltage

  // check status of relay circuit

  // confirm relays are in the position they are supposed to be in

  // 

}

/* -------------------- CORE 1 -------------------- */

// RADIO SETUP
void setup1() {

}

// RADIO LOOP
void loop1() {
  // listen for signals

  // ping GOOPER
    // simple connection ping (handshake)
    // data handoff
      // battery data
      // status data

  // recieve battery and other status info from WestSystems105

}

/* -------------------- FUNCTIONS -------------------- */

