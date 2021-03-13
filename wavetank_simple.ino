//digital output pins for relay
const int UP_R_VALVE = 5;
const int UP_P_VALVE = 6;
const int DN_R_VALVE = 10;
const int DN_P_VALVE = 11;
const int hz =12000;

void setup() {
  
  pinMode(UP_R_VALVE, OUTPUT);
  pinMode(UP_P_VALVE, OUTPUT);
  pinMode(DN_R_VALVE, OUTPUT);
  pinMode(DN_P_VALVE, OUTPUT);
}

void loop() {
  digitalWrite(UP_R_VALVE, HIGH);
  digitalWrite(DN_P_VALVE, HIGH);
  delay(hz/2);
  digitalWrite(UP_R_VALVE, LOW);
  digitalWrite(DN_P_VALVE, LOW);
  
  digitalWrite(DN_R_VALVE, HIGH);
  digitalWrite(UP_P_VALVE, HIGH);
  delay(hz/2);
  digitalWrite(DN_R_VALVE, LOW);
  digitalWrite(UP_P_VALVE, LOW);
}
