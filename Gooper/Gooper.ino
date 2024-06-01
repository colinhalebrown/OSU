/* Gooper.INO
Gooper is the CONTROL box located at the control station
Gooper is ORANGE

This project is developed by OSU AIAA ESRA 2024
Colin Hale-Brown
Dexter Carpenter
*/

/* -------------------- INCLUDED LIBRARIES ------------- */

#include <SPI.h>
#include <RH_RF95.h>

/* -------------------- DEFINE HARDWARE ---------------- */

// define hardware setup
#define RFM95_CS   16
#define RFM95_INT  21
#define RFM95_RST  17

// define radio frequency
#define RF95_FREQ 915.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

/* -------------------- GLOBAL VARS -------------------- */

// define pins
const int BUZZ_PIN = 5; // Peizo Buzzer
const int RLED_PIN = 6; // Radio Indicator
const int WEST_LP_PIN = 9; // Pad LP Indicator
const int WEST_LC_PIN = 10; // Pad LC Indicator
const int WEST_CON_PIN = 11; // Pad Continuity Indicator
const int FLED_PIN = 12; // Firing Button LED
const int PLED_PIN = 13; // Power Switch LED

const int ARM_PIN = 25; // Arming Switch Pin
const int FIRE_PIN = 24; // Firing Button Pin

const int BVOLT_PIN = A0; // Controller Voltage Divider

// voltage
int BatVoltage = 0;

// radio variables
int16_t packetnum = 0;  // packet counter, we increment per xmission
int RadioTimeout = 1000; // Timeout in ms

// Commands
bool ArmSystem = false;
bool FireSystem = false;

// West Checks
bool ArmIndicator = false;
bool ConIndicator = false;
bool WestVoltage = false;
bool IgnVoltage = false;

/* -------------------- CORE 0 -------------------- */

void setup() {
  // define pin modes
  pinMode(BUZZ_PIN , OUTPUT);
  pinMode(RLED_PIN, OUTPUT);
  pinMode(WEST_LP_PIN, OUTPUT);
  pinMode(WEST_LC_PIN, OUTPUT);
  pinMode(WEST_CON_PIN, OUTPUT);
  pinMode(FLED_PIN, OUTPUT);
  pinMode(PLED_PIN, OUTPUT);

  pinMode(ARM_PIN, INPUT);
  pinMode(FIRE_PIN, INPUT);

  // begin serial
  Serial.begin(115200);

  // Set power LED on Should check battery voltage in the final code
  digitalWrite(PLED_PIN, HIGH);

  // ensure startup success
  while (digitalRead(ARM_PIN) == HIGH || digitalRead(FIRE_PIN) == HIGH) {
    // blink that fucking armed LED like crazy

    // re-read the arming switch
    // re-read firing button
  }
}

// STANDARD LOOP
void loop() {
  // read and print battery voltage
  BatVoltage = analogRead(BVOLT_PIN); // Serial.println(BatVoltage);

  if (digitalRead(ARM_PIN) == HIGH) {
    ArmSystem = true;
    if (digitalRead(FIRE_PIN) == HIGH) {
      FireSystem = true;
    } else {
      FireSystem = false;
    }
  } else {
    ArmSystem = false;
    FireSystem = false;
  }

  // Serial.print("System Armed: "); Serial.println(ArmSystem);
  // Serial.print("Fireing: "); Serial.println(FireSystem);

  if (ArmSystem == true) {

  } else if (FireSystem == true) {

  }

  delay(200);
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
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while (!Serial) delay(1);
  delay(100);

  Serial.println("GOOPER TX");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // radio hardware status check
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
  delay(500); //wait between transmits

  Serial.println("Transmitting...");
  digitalWrite(RLED_PIN, HIGH);

  // Packet Format
  // "<ArmSystem> <FireSystem> <ArmIndicator> <ConIndicator> <WestVoltage> <IgnVoltage> #<PacketID>"
  // example: "1 0 1 0 0 1 #42"
  // This shows the system is sending an armed command and west has armed itself this is the 42nd handshake no firing or continuity. The box is low on igniter voltage but the main battery is fine.

  char radiopacket[15];
  sprintf(radiopacket, "%d %d %d %d %d %d #      ", ArmSystem, FireSystem, ArmIndicator, ConIndicator, WestVoltage, IgnVoltage);
  itoa((int)packetnum++, radiopacket+13, 10);

  Serial.print("Sending "); Serial.println(radiopacket);
  radiopacket[14] = 0;

  Serial.println("Sending...");
  delay(10);
  rf95.send((uint8_t *)radiopacket, 20);

  Serial.println("Waiting for packet to complete...");
  delay(10);
  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.println("Waiting for reply...");
  if (rf95.waitAvailableTimeout(RadioTimeout)) {
    // Should be a reply message for us now
    if (rf95.recv(buf, &len)) {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      digitalWrite(RLED_PIN, LOW);
    } else {
      Serial.println("Receive failed");
    }
  } else {
    Serial.println("No reply, is there a listener around?");
  }

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

void pulse(int PIN,int PINState,int previousMillis,int interval) {
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
