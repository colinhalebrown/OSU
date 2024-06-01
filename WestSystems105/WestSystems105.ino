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
const int BUZZ_PIN = 5;   // Peizo Buzzer
const int RLED_PIN = 6;   // Radio Indicator
const int ALED_PIN = 9;   // Armed Indicator
const int ARM_PIN  = 10;  // Arming Relay
const int FIRE_PIN = 11;  // Firing Relay
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
bool ArmSystem = false;
bool FireSystem = false;

// West State
bool Armed = false;
bool Fire = false;

// Checks
bool Standby = false;
bool ArmIndicator = false;
bool ConIndicator = false;

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
  
  if (ArmVoltage > 100) {
    ArmIndicator = true;
  } else {
    ArmIndicator = false;
  }

  if (ConVoltage > 100) {
    ConIndicator = true;
  } else {
    ConIndicator = false;
  }


  // confirm relays are in the position they are supposed to be in

  // 

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
      digitalWrite(RLED_PIN, LOW);
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Got: ");
      Serial.println((char*)buf);
       Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);

      // Send a reply
      uint8_t data[] = "pong";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      Serial.println("Sent a reply");
      digitalWrite(RLED_PIN, HIGH);
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

