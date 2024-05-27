const int BUZ_PIN = 5;
const int LED1_PIN = 6;
const int LED2_PIN = 9;
const int RLY1_PIN = 10;
const int RLY2_PIN = 11;
const int BAT_PIN = A0;
const int CHG_PIN = A1;
const int ARM_PIN = A2;
const int CON_PIN = A3;

int BatVoltage = 0;
int ChgVoltage = 0;
int ArmVoltage = 0;
int ConVoltage = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(BUZ_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(RLY1_PIN, OUTPUT);
  pinMode(RLY2_PIN, OUTPUT);

  Serial.begin(115200);
}

void loop() {
  BatVoltage = analogRead(BAT_PIN);

  Serial.println(BatVoltage);

}


void fire() {
  
}

