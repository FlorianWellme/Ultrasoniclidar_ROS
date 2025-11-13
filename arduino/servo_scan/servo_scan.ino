#include <Servo.h>

Servo myservo;

// Broches du capteur ultrason
const int trigPin = 7;
const int echoPin = 6;

int pos = 0;

void setup() {
  Serial.begin(115200);
  myservo.attach(9);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

float measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // max 30 ms timeout
  float distance = duration * 0.0343 / 2.0; // cm
  return distance;
}

void loop() {
  // Balayage gauche → droite
  for (pos = 5; pos <= 175; pos += 5) {
    myservo.write(pos);
    delay(150);
    float dist = measureDistance();
    Serial.print(pos);
    Serial.print(",");
    Serial.println(dist);
  }

  // Balayage droite → gauche
  for (pos = 175; pos >= 5; pos -= 5) {
    myservo.write(pos);
    delay(150);
    float dist = measureDistance();
    Serial.print(pos);
    Serial.print(",");
    Serial.println(dist);
  }
}
