unsigned int del = 100;
unsigned int lastHit = 0;
unsigned int hitTime = 50;
#define pin 8

void setup() {
  Serial.begin(9600);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(A0, INPUT);
}

void loop() {
  del = analogRead(A0);
  if (del < 950) {
    if (millis() - lastHit >= del+hitTime) {
      digitalWrite(pin, LOW);
      lastHit = millis();
    } else if (millis() - lastHit >= hitTime) {
      digitalWrite(pin, HIGH);
    }
  }
}
