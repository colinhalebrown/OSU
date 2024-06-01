/* WestSystems105.INO
WS105 is the PAD box located at the ROCKET WITH EXPLOSIVES
WS105 is BLACK

This project is developed by OSU AIAA ESRA 2024
Colin Hale-Brown
Dexter Carpenter
*/

/* -------------------- INCLUDED LIBRARIES -------------------- */

#include <SPI.h>
#include <RH_RF95.h> // v1.122.1

/* -------------------- DEFINE HARDWARE -------------------- */

// define hardware setup
#define RFM95_CS   16
#define RFM95_INT  21
#define RFM95_RST  17

// define radio frequency
#define RF95_FREQ 915.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

/* -------------------- GLOBAL VARS -------------------- */

// define pins
const int BUZZ_PIN  = 5;  // Peizo Buzzer
const int RLED_PIN  = 6;  // Radio Indicator
const int ALED_PIN  = 9;  // Armed Indicator
const int ARM_PIN   = 10; // Arming Relay
const int FIRE_PIN  = 11; // Firing Relay
const int BAT_PIN   = A0; // Controller voltage divider
const int IVOLT_PIN = A1; // Ignitor voltage divider
const int AVOLT_PIN = A2; // Arming continuity voltage divider
const int CVOLT_PIN = A3; // Firing continuity voltage divider

// voltages
int BatVoltage = 0;
int IgnVoltage = 0;
int ArmVoltage = 0;
int ConVoltage = 0;

// Commands
bool ArmSystem  = false;
bool FireSystem = false;

// West State
bool Standby = false;
bool Armed   = false;
bool Fire    = false;

// Checks
bool ArmIndicator    = false;
bool ConIndicator    = false;
bool LowBatIndicator = false;
bool LowChgIndicator = false;

// UI Variables
int ArmInterval = 400;
int ArmState = LOW;
unsigned long PreviousArmMillis = 0;
unsigned long CurrentArmMillis = 0;

/* -------------------- CORE 0 -------------------- */

// STANDARD SETUP
void setup() {
  // put your setup code here, to run once:
  pinMode(BUZZ_PIN, OUTPUT);
  pinMode(RLED_PIN, OUTPUT);
  pinMode(ALED_PIN, OUTPUT);
  pinMode(ARM_PIN, OUTPUT);
  pinMode(FIRE_PIN, OUTPUT);

  // begin serial
  Serial.begin(115200);
}

// STANDARD LOOP
void loop() {
  // collect WS105 battery voltage
  BatVoltage = analogRead(BAT_PIN); //Serial.print("Battery Voltage: "); Serial.println(BatVoltage);

  // collect charge battery voltage
  IgnVoltage = analogRead(IVOLT_PIN); //Serial.print("Ignitor Voltage "); Serial.println(IgnVoltage);

  // check status of relay circuits
  ArmVoltage = analogRead(AVOLT_PIN); //Serial.print("Arm Voltage "); Serial.println(ArmVoltage);
  ConVoltage = analogRead(CVOLT_PIN); //Serial.print("Continuity Voltage "); Serial.println(ConVoltage);
  
  // UPDATE ARMING VOLTAGE
  if (ArmVoltage > 100) { ArmIndicator = true; } else { ArmIndicator = false; }

  // UPDATE CONT. VOLTAGE
  if (ConVoltage > 100) { ConIndicator = true; } else { ConIndicator = false; }

  // UPDATE ARMING RELAY
  if (ArmSystem) {
    digitalWrite(ARM_PIN,HIGH); // ARM RELAY
    CurrentArmMillis = millis(); // get current time for arming
    // if within interval
    if (CurrentArmMillis - PreviousArmMillis >= ArmInterval) {
      PreviousArmMillis = CurrentArmMillis; // update previous
      if (ArmState == LOW) { // if currently off,
        ArmState = HIGH; // turn on
      } else {
        ArmState = LOW; // turn off
      }
    }
    // blink and buzz accordingly
    digitalWrite(BUZZ_PIN, ArmState);
    digitalWrite(ALED_PIN,!ArmState);
  } else {
    // don't blink or buzz
    digitalWrite(BUZZ_PIN, LOW); // off 
    digitalWrite(ALED_PIN,HIGH); // off
  }

  // UPDATE FIRING PIN
  if (ArmSystem && FireSystem) { digitalWrite(FIRE_PIN,HIGH); } else { digitalWrite(FIRE_PIN,LOW); }

  // Update ArmIndicator Boolean
  if (ArmVoltage > 20) { ArmIndicator = true; } else { ArmIndicator = false; }

  // Update ConIndicator Boolean
  if (ConVoltage > 20) { ConIndicator = true; } else { ConIndicator = false; }

}

/* -------------------- CORE 1 -------------------- */

// RADIO SETUP
void setup1() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while (!Serial) delay(1);
  delay(100);

  Serial.println("West Systems 105 RX");

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // set radio frequency
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // set radio power (23dBm)
  rf95.setTxPower(23, false);
}

// RADIO LOOP
void loop1() {

  if (rf95.available()) {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      // print data received
      digitalWrite(RLED_PIN, LOW); // Turn off LED
      //RH_RF95::printBuffer("Received: ", buf, len); // print out hexadecimal
      char* packetRX = (char*)buf; // decode packet
      Serial.print("RSSI: "); Serial.println(rf95.lastRssi(), DEC); // print signal strength
      Serial.print("RX: "); Serial.println(packetRX); // print decoded message

      // Adjust Commands and Checks
      if (packetRX[0] == '1') { ArmSystem  = true; } else { ArmSystem  = false; } // ArmSystem
      if (packetRX[2] == '1') { FireSystem = true; } else { FireSystem = false; } // FireSystem

      // Update packet
      sprintf(packetRX, "%d %d %d %d %d %d #      ", ArmSystem, FireSystem, ArmIndicator, ConIndicator, LowBatIndicator, LowChgIndicator);

      // Send a reply
      uint8_t* packetTX = (uint8_t*)atoi(packetRX);
      rf95.send(packetTX, sizeof(packetTX));
      rf95.waitPacketSent();
      Serial.print("TX: "); Serial.println(packetRX);
      digitalWrite(RLED_PIN, HIGH); // Blink LED
    } else {
      Serial.println("Receive failed");
    }
  }

  // listen for signals

  // ping GOOPER
    // simple connection ping (handshake)
    // data handoff
      // battery data
      // status data

  // recieve battery and other status info from WestSystems105

}

/* -------------------- FUNCTIONS -------------------- */

