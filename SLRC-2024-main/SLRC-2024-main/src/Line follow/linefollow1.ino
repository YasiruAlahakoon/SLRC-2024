// connect motor controller pins to Arduino digital pins
// motor one
#include <QTRSensors.h>

QTRSensors qtr;

//[-60,-52,-44,-36,-28,-20,-12,-4,4,12,20,28,36,44,52,60]
// [ -0.60, -0.52, -0.44, -0.36, -0.28, -0.20, -0.12, -0.04, 0.04, 0.12, 0.20, 0.28, 0.36, 0.44, 0.52, 0.60 ]
//-0.60, -0.52, -0.44, -0.40, -0.32, -0.24, -0.18, -0.04, 0.04, 0.18, 0.24, 0.32, 0.40, 0.44, 0.52, 0.60
const uint8_t SensorCount = 8;
float sensorW[8] = { -3.78, -2.58, -1.86, -0.8, 0.8, 1.86, 2.58, 3.78 };//-1.38, -0.83, -0.44, -0.12, 0.12, 0.44, 0.83, 1.38
uint16_t sensorValues[SensorCount];
double weightedVal[SensorCount];
double dVal[SensorCount];
double digital_thres = 400;


double position = 0;
double P, I, D, PID, PreErr = 0;
double offset = -6;

double motorSpeedA;
double motorSpeedB;
double baseSpeed = 140;
double Kp = 1.58; //1.1
double Kd = 1.32;

int enA = 4;
int in1 = 3;//3
int in2 = 2;//2
// motor two
int enB = 5;
int in3 = 6;//6
int in4 = 7;//7

int left= 22;
int right= 52;

bool linefollow=true;

void setup()

{
  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 31, 32, 33, 34, 35, 36, 37, 38 }, SensorCount); //, 39, 40, 41, 42, 43, 44, 45, 46
  qtr.setEmitterPin(2);

  Serial.begin(9600);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void sensorRead() {
  double weightedSum = 0;
  double actualSum = 0;
  qtr.read(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++) {
    sensorValues[i] = map(sensorValues[i], 0, 2500, 1000, 0);
    weightedVal[i] = sensorW[i] * sensorValues[i];
    actualSum += sensorValues[i];
    weightedSum += weightedVal[i];
    if(sensorValues[i] > digital_thres){
      dVal[i] = 1;
    }
    else {
      dVal[i] = 0;
    }
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println("");

  position = weightedSum / actualSum;
  //Serial.println(position*100);


  delay(50);
}


void PID_control() {
  // Serial.println("Kp " + String(Kp,4) + "   KI " + String(Ki,4) + "   KD " + String(Kd,4) );
  //Serial.Println(positionLine);
  P = position * 100;
  D = P - PreErr;
  PID = Kp * P + Kd * D;

  PreErr = P;



  double MSpeedA = baseSpeed + offset - PID;  //
  double MSpeedB = baseSpeed - offset + PID;  //offset has been added to balance motor speeds


  // // constraints for speed

  if (MSpeedA > 140) {
    MSpeedA = 140;
  }

  if (MSpeedB > 140) {
    MSpeedB = 140;
  }

  if (MSpeedA < 40) {
    MSpeedA = 40;
  }

  if (MSpeedB < 30) {
    MSpeedB = 30;
  }

  //Serial.println("Position   " + String(position*100) + "  D  " + String(D) +  "  PID  "+ String(PID));
  // Serial.println("A   " + String(MSpeedA-offset)+ "   B   " +String(MSpeedB+offset));
  //enable pwm
  analogWrite(enA, MSpeedA);
  analogWrite(enB, MSpeedB);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  //delay(50);
}

void goForward(double forward_delay) {
  // this function will run the motors in
  //both directions at a fixed speed
  // turn on motor A
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // set speed to 200 out of possible range 0~255
  analogWrite(enA, 80 + offset);
  // turn on motor B
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  // set speed to 200 out of possible range 0~255
  analogWrite(enB, 80 - offset);
  delay(forward_delay);
  // now change motor directions
}

void TurnRight(double turn_delay){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // set speed to 200 out of possible range 0~255
  analogWrite(enA, 100 + offset);
  // turn on motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  // set speed to 200 out of possible range 0~255
  analogWrite(enB, 100 - offset);
  delay(turn_delay);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
}

void TurnLeft(double turn_delay){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  // set speed to 200 out of possible range 0~255
  analogWrite(enA, 100 + offset);
  // turn on motor B
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  // set speed to 200 out of possible range 0~255
  analogWrite(enB, 100 - offset);
  delay(turn_delay);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
}

void stop(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

}

void linefollow(){
  sensorRead();
  if (digitalRead(left)==LOW){
    goForward(600);  //Turn Left
    TurnLeft(1450);
  }else if (digitalRead(right)==LOW ){ 
    goForward(1200);  //Turn Right
    TurnRight(1500);

  }

  else{
  PID_control();

  }
}

void loop() {
    sensorRead();
   if 
    
    
    

}
  