
#define pwmPin 37 // DC velocity
#define encPin 20 // Encoder 
#define inPin1 35 // DC dir1
#define inPin2 33 // DC dir2


int dir = 1;
int e = 0;
bool arrived = false;

int targetPos = 5;
int motorDir = 1;
int encPos = 0;

void setMotor(int dir, int pwmVal) {
  digitalWrite(inPin1, dir==1);
  digitalWrite(inPin2, dir==-1);
  analogWrite(pwmPin,pwmVal);
}

void readEncoder() {
  Serial.println(encPos);
  encPos -= dir;
  e = targetPos - encPos;
  Serial.println(encPos);
  arrived = (abs(e) < 2);
  if (arrived) { 
    encPos -= dir; 
    setMotor(0, 0); 
    dir=0;
  }
}

void setup() {
  Serial.begin(9600);
  delay(2000);
  pinMode(inPin1, OUTPUT);
  pinMode(inPin2, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(encPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encPin),readEncoder,RISING);
  targetPos = 3; // <------- CHANGE TARGET POS HERE 
  
}

void loop() {
  if (arrived) {
    setMotor(0,0);
    targetPos *= -1;
    arrived = false;
    delay(500);
  } else {
    dir = -1;
    e = targetPos - encPos;
    if (e < 0) {dir = 1;}
    setMotor(dir, 130);
  }
  delay(10);
}
