unsigned long lastSendTime = 0;
const unsigned long sendInterval = 10; // 10ms = 100 Hz

void setup() {
  Serial.begin(115200);
  pinMode(34, INPUT);
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastSendTime >= sendInterval) {
    int sensorValue = analogRead(34);
    Serial.println(sensorValue);
    lastSendTime = currentTime;
  }
}