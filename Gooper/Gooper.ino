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

// radio variables
int16_t packetnum = 0;  // packet counter, we increment per xmission

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

  char radiopacket[20] = "Hello World #      ";
  itoa(packetnum++, radiopacket+13, 10);
  Serial.print("Sending "); Serial.println(radiopacket);
  radiopacket[19] = 0;

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
  if (rf95.waitAvailableTimeout(1000)) {
    // Should be a reply message for us now
    if (rf95.recv(buf, &len)) {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
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

void pulse(int PIN,int PINState,int beginMillis,int interval) {
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
