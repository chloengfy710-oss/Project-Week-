#include <LiquidCrystal.h>
#include <PinChangeInterrupt.h>

// LCD pins: RS, E, D4, D5, D6, D7
LiquidCrystal lcd(8,9,4,5,6,7);

// L298N motor driver pins
const int ENA = 3;   // Enable pin for left motor (PWM)
const int IN1 = 2;   // Left motor direction 1
const int IN2 = 1;   // Left motor direction 2
const int ENB = 11;  // Enable pin for right motor (PWM)
const int IN3 = 12;  // Right motor direction 1
const int IN4 = 13;  // Right motor direction 2

#define leftIR A1
#define rightIR A2

//Encoder pins
const int LEFT_ENCODER= A3;
const int RIGHT_ENCODER= A4;

//lcd time display
unsigned long starttime= 0;
unsigned long duration= 0;

int lastturn= 0;

const int blackthreshold= 8; 
int blackcount = 0; 


volatile long leftencodercount= 0;
volatile long rightencodercount= 0;
bool robotdone= false;

unsigned long previousTime = 0;

void setup() {
  lcd.begin(16, 2);
  lcd.print("Robot Car Ready");
  delay(2000);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(leftIR, INPUT);
  pinMode(rightIR, INPUT);

  pinMode(LEFT_ENCODER, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER, INPUT_PULLUP);

  previousTime = millis(); // record start time

  attachPCINT(digitalPinToPCINT(LEFT_ENCODER), readleftencoder, RISING);
  attachPCINT(digitalPinToPCINT(RIGHT_ENCODER), readrightencoder, RISING);

}

void loop() {
  unsigned long currentTime = (millis() - previousTime) / 1000; // seconds
  displaydistance();
  //lcd.setCursor(0, 0);
  //lcd.print("Distance: ");
  //lcd.print(totaldistance);
  lcd.setCursor(0, 1);
  lcd.print("Time: ");
  lcd.print(currentTime);
  lcd.print(" sec ");
  
  bool L = digitalRead(leftIR); 
  bool R = digitalRead(rightIR);

  if (L && R) {
    moveForward();
  }
  else if (!L && R) {
      turnLeft();                 
  }
  else if (L && !R) {
      turnRight();
      delay(300);              
  }
  else {
    stop();
    while (true){};
  }

}

float calculatedistance(){
  const float pulseseachrev= 20.0;
  const float wheelcircum= 20.42; //in cm
  const float cmperpulse= wheelcircum/ pulseseachrev;

  //noInterrupts();
  long L = leftencodercount;
  long R = rightencodercount;
  //interrupts();

  float avgticks= (L+R)/2.0;
  float totaldistance= avgticks*cmperpulse;
  
  return totaldistance;  
 }

 void displaydistance(){
  float cm= calculatedistance();

  lcd.setCursor(0, 0);
  lcd.print("Dist:");
  lcd.print(cm, 2);
  lcd.print(" cm");
 }

void readleftencoder(void){
  leftencodercount++;
}

void readrightencoder(void){
  rightencodercount++;
}



// Move forward
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 150);
  analogWrite(ENB, 255);
}

void stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}