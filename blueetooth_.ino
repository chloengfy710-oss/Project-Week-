#include <SoftwareSerial.h>

SoftwareSerial BT(2, 12); // RX, TX
// Motor driver pins
#define ENA 3    // LEFT motor
#define IN1 A2
#define IN2 A3
#define IN3 A4
#define IN4 A5
#define ENB 11    // RIGHT motor

// Speeds
const int LEFT_BASE_SPEED  = 150;
const int RIGHT_BASE_SPEED = 150;

void moveForward();
void turnLeft();
void turnRight();
void stopCar();


void setup() {
   Serial.begin(9600);
  BT.begin(9600);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}
  

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, LEFT_BASE_SPEED);
  analogWrite(ENB, RIGHT_BASE_SPEED);
}

void turnLeft() {
  // Left slow, right fast
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, 150);     // left slow
  analogWrite(ENB, 150);    // right full speed
}

void turnRight() {
  // Right slow, left fast
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 140);    // left fast
  analogWrite(ENB, 140);     // right slow
}

void stopCar() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void backward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, LEFT_BASE_SPEED);
  analogWrite(ENB, RIGHT_BASE_SPEED);
}


void loop() {
 if (BT.available()) {
    char cmd = BT.read(); // read single character command

    if (cmd == 'F') moveForward();    // Forward
    else if (cmd == 'B') backward(); // Backward
    else if (cmd == 'L') turnLeft();     // Left
    else if (cmd == 'R') turnRight();    // Right
    else if (cmd == 'S') stopCar();      // Stop
  }
}
