int ENA=3;
int IN1=2;
int IN2=1;

int ENB=11;
int IN3=12;
int IN4=13;

void setup() {
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

}

void loop(){
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  analogWrite(ENA,200);
  analogWrite(ENB,200);
  delay(2000);

  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  analogWrite(ENA,200);
  analogWrite(ENB,200);
  delay(2000);

  analogWrite(ENA,0);
  analogWrite(ENB,0);
  delay(1000);
}
