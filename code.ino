#include <Servo.h>

// servo motors
Servo PSer;
#define PS 12

// flame sensors
#define LF 8
#define FF 9
#define RF 10

// Sonar sensors
#define STr A3
#define SEc A4

// motors
#define LME A2
#define LM1 3
#define LM2 2

#define RME A1
#define RM1 5
#define RM2 4


// pump
#define PMP 7

// others
int RMSp = 150; // right motor speed
int LMSp = 150; // left motor speed
float AngV = 720.; // angular velocity
float LinV = 100.; // linear velocity
float delTm; // delay time
float fireDist = 60.; // fire distance
float minDist = 20.; // minimum distance
float colDist = 10.; // collision distance
float LFAng = 45.; // left flame sensor angle
float RFAng = 45.; // right flame sensor angle
int wait = 500;

// sensor reading
int LFVal;
int FFVal;
int RFVal;
float SSVal;

void setup() {
  // servo pins
  PSer.attach(PS);
  PSer.write(90);

  // flame sensor pins
  pinMode(LF, INPUT);
  pinMode(FF, INPUT);
  pinMode(RF, INPUT);

  // sonar sensor pins
  pinMode(STr, OUTPUT);
  pinMode(SEc, INPUT);

  // motor pins
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);

  // motor speed
  analogWrite(RME, RMSp);
  analogWrite(LME, LMSp);

  // stop state
  stop();

  pinMode(13, OUTPUT);
}

void loop() {
  // read sensors
  LFVal = digitalRead(LF);
  FFVal = digitalRead(FF);
  RFVal = digitalRead(RF);
  SSVal = measDist();
 
  // main maneuver
  if (SSVal < minDist){
    avoidColl();
  } else if (LFVal == 0 || FFVal == 0 || RFVal == 0){
    if (LFVal == 0){
      delTm = (LFAng / AngV) * 1000;
      leftTurn();
      delay(delTm);
      stop();
      delay(wait);
    } else if (RFVal == 0){
      delTm = (RFAng / AngV) * 1000;
      rightTurn();
      delay(delTm);
      stop();
      delay(wait);
    }
    if (FFVal == 0) {
      extFire();
    }
  }
}

// avoid collision
void avoidColl(){
  while (SSVal < minDist){
    SSVal = measDist();
    while (SSVal < colDist){
      delTm = ((minDist - SSVal) / LinV) * 1000;
      backMove();
      delay(delTm);
      stop();
      SSVal = measDist();
    }
  }
}

// extinguish fire
void extFire(){
  float moveTm = 0.; // move time
  float startTm; // start time
  float maneuTm = 0.; // avoid maneuver time
  delTm = (fireDist / LinV) * 1000;
  startTm = millis();
  forMove();
  while (moveTm < delTm) {
    SSVal = measDist();
    moveTm = millis() - startTm - maneuTm;
    if (SSVal < minDist){
      stop();
      maneuTm = avoidObj();
      forMove();
    }
  }
  stop();
  delay(wait);
  pumpManeu();
}

// avoid object
float avoidObj(){
  float dt; // delay time
  float strMan; // start time
  float forMan; // move time
  float stpMan; // stop time
 
  strMan = millis();
  delay(wait);
 
  //turn left
  dt = (90 / AngV) * 1000;
  leftTurn();
  delay(dt);
  stop();
  delay(wait);

  // move forward
  dt = (20 / LinV) * 1000;
  forMove();
  delay(dt);
  stop();
  delay(wait);

  //turn right
  dt = (90 / AngV) * 1000;
  rightTurn();
  delay(dt);
  stop();
  delay(wait);

  // move forward
  forMan = (20 / LinV) * 1000;
  forMove();
  delay(forMan);
  stop();
  delay(wait);

  //turn right
  dt = (90 / AngV) * 1000;
  rightTurn();
  delay(dt);
  stop();
  delay(wait);

  // move forward
  dt = (20 / LinV) * 1000;
  forMove();
  delay(dt);
  stop();
  delay(wait);

  //turn left
  dt = (90 / AngV) * 1000;
  leftTurn();
  delay(dt);
  stop();
  delay(wait);

  stpMan = millis();
  return stpMan - strMan - forMan;
}

// pump maneuver
void pumpManeu(){
  digitalWrite(PMP, HIGH);
  for (int pos = 40; pos < 141; pos++){
    PSer.write(pos);
    delay(10);
  }
  for (int pos = 140; pos > 39; pos--){
    PSer.write(pos);
    delay(10);
  }
  digitalWrite(PMP, LOW);
  PSer.write(90);
}

// measure distance
float measDist(){
  int dist; // distance
  float durt; // duration
  digitalWrite(STr, LOW);
  delayMicroseconds(2);
  digitalWrite(STr, HIGH);
  delayMicroseconds(10);
  digitalWrite(STr, LOW);
  durt = pulseIn(SEc, HIGH);
  dist = durt * 0.0344 / 2;
  return dist;
}

// move forward
void forMove(){
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, HIGH);
  digitalWrite(LM1, LOW);
  digitalWrite(LM2, HIGH);
}

// move backward
void backMove(){
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);
  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
}

// turn right
void rightTurn(){
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);
  digitalWrite(LM1, LOW);
  digitalWrite(LM2, HIGH);
}

// turn left
void leftTurn(){
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, HIGH);
  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
}

// stop
void stop(){
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, LOW);
  digitalWrite(LM1, LOW);
  digitalWrite(LM2, LOW);
}
