// connect motor controller pins to Arduino digital pins
// motor one
#include <Wire.h>
#include <VL53L0X.h>
#include <TCA9548.h>
#include "Adafruit_TCS34725.h"
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

bool moveToSection1=false;

double position = 0;
double P, I, D, PID, PreErr = 0;
double offset = -1;

double motorSpeedA;
double motorSpeedB;
double baseSpeed = 100;
double Kp = 1.58; //1.1
double Kd = 1.32;

int enA = 4;
int in1 = 3;
int in2 = 2;
int enB = 5;
int in3 = 6;
int in4 = 7;

int left= 22;
int right= 52;

const int LED1_PIN = A0;
const int LED2_PIN = A1;

String task01Color ;
String task02Color ;
String countg;
String countb;

uint16_t distance1 =0;
uint16_t distance2=0;
uint16_t r1 = 0, g1 = 0, b1 = 0 ,c1=0;
uint16_t r2 = 0, g2 = 0, b2 = 0 ,c2=0;

int section =0;
int key =0;
const int numReadings = 50;
float readings[numReadings];

bool colourdetected=false;

TCA9548 multiplexer(0x70, &Wire);

VL53L0X sensor1;
VL53L0X sensor2;

Adafruit_TCS34725 tcs1 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 tcs2 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

void setup()
{
  Wire.begin();
  multiplexer.begin();
  multiplexer.selectChannel(5);
  sensor1.setAddress(0x29);
  sensor1.init();
  delay(100);
  sensor1.setTimeout(1500);
  sensor1.startContinuous();

  multiplexer.selectChannel(2);
  sensor2.setAddress(0x2A);
  sensor2.init();
  sensor2.setTimeout(500);
  sensor2.startContinuous();

  multiplexer.selectChannel(6);
  multiplexer.selectChannel(4);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 31, 32, 33, 34, 35, 36, 37, 38 }, SensorCount);
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
}

void PID_control() {
  P = position * 100;
  D = P - PreErr;
  PID = Kp * P + Kd * D;

  PreErr = P;

  double MSpeedA = baseSpeed + offset - PID;
  double MSpeedB = baseSpeed - offset + PID;

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

  analogWrite(enA, MSpeedA);
  analogWrite(enB, MSpeedB);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void lineFollow(){
   if (digitalRead(left)==LOW){
       goForward(1600);
       TurnLeft(1500);
       Serial.println("LEFT");
     }else if (digitalRead(right)==LOW ){ 
       goForward(1600);
       TurnRight(1500);
     }
    else{
    PID_control();
    }
}

void lineFollowWithRightPriority(){
   if (digitalRead(right)==LOW){
       goForward(1600);
       TurnRight(1500);
     }
     else if (digitalRead(left)==LOW ){ 
       goForward(1600);
       TurnLeft(1500);
     }  
     else if ((digitalRead(left)==LOW) &&(digitalRead(right)==LOW )){
       goForward(1600);
       TurnRight(1500);
     }
    else{
    PID_control();
    }
}

void lineFollowWithLeftPriority(){
   if( (digitalRead(left)==LOW)  &&(digitalRead(right)==LOW )){
       goForward(1600);
       TurnLeft(1500);
     }
     else if (digitalRead(left)==LOW ){ 
       goForward(1600);
       TurnLeft(1500);
     }  
     else if ((digitalRead(right)==LOW)){
       goForward(1600);
       TurnRight(1500);
     }
    else{
    PID_control();
    }
}

void goForward(double forward_delay) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, 80 + offset);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, 80 - offset);
  delay(forward_delay);
}

void goBackward(double forward_delay) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, 80 + offset);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, 80 - offset);
  delay(forward_delay);
}

void TurnRight(double turn_delay){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, 100 + offset);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
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
  analogWrite(enA, 100 + offset);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
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

int distanceFront(){
  multiplexer.selectChannel(5);
  uint16_t distance1 = sensor1.readRangeContinuousMillimeters();

  if (sensor1.timeoutOccurred()) {
    Serial.println("VL53L0X timeout on channel 1!");
  } else {
    Serial.print("Distance 1: ");
    Serial.print(distance1);
    Serial.println(" mm");
    return distance1;
  }
}

int distanceLSide(){
  multiplexer.selectChannel(2);
  uint16_t distance2 = sensor2.readRangeContinuousMillimeters();

  if (sensor2.timeoutOccurred()) {
    Serial.println("VL53L0X timeout on channel 2!");
  } else {
    Serial.print("Distance 2: ");
    Serial.print(distance2);
    Serial.println(" mm");
  }
  return distance2;
}

void colorBottom(){
  multiplexer.selectChannel(4);
  delay(750);
  uint16_t r2, g2, b2, c, colorTemp, lux;

  tcs1.getRawData(&r2, &g2, &b2, &c);

  Serial.print("R: "); Serial.print(r2, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g2, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b2, DEC); Serial.print(" ");
  Serial.println(" ");
  delay(10); 
  if(b2<g2){
    task02Color="green";
    Serial.println("Green Detected ");
  }
  else{
    task02Color="blue";
    Serial.println("Blue Detected ");
  }
}

void reversePID_Comtrol{
  p=-1.58;
  d=-1.32;
}

  P = position * 100;
  D = P - PreErr;
  PID = Kp * P + Kd * D;

  PreErr = P;

  double MSpeedA = baseSpeed + offset - PID;  //
  double MSpeedB = baseSpeed - offset + PID;  //offset has been added to balance motor speeds

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

  analogWrite(enA, MSpeedA);
  analogWrite(enB, MSpeedB);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  //delay(50);

}

void colorFront(){
 
    multiplexer.selectChannel(4);
    delay(750);//2000
    uint16_t r1, g1, b1, c, colorTemp, lux;

  tcs1.getRawData(&r1, &g1, &b1, &c);

  Serial.print("R: "); Serial.print(r1, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g1, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b1, DEC); Serial.print(" ");
  Serial.println(" ");
  delay(10); 
  if(b1<g1){
      task01Color="green";//Green
      //analogWrite(LED2_PIN,140);
       Serial.println("Green Detected ");
       colourdetected=true;
      
      
    }
    else{
      task01Color="blue";//Blue
      //analogWrite(LED1_PIN,140);
       Serial.println("Blue Detected ");
       colourdetected=true;
      
    }


}

//take a specified number of readings and store them in the array
void takeReadings(int numReadings) {
  Serial.println(" Readings are started to take");
  for (int i = 0; i < numReadings; i++) {
    int distance = distanceLSide();      //distanceSide();120000/50
    if (distance >= 0) {
      readings[i] = distance;
    }
    delay(250);
    
  }
  Serial.println("50 Readings are taken");
  float average = getAverageDistance(numReadings);
  float variance = getStdDevDistance( numReadings,average);
  cylinderOrCuboidCalculattion(variance);


}

// Function to calculate the average distance reading from the array
float getAverageDistance(int numReadings ) {
  int sum = 0;
  for (int i = 0; i < numReadings; i++) {
    sum += readings[i];
  }
  return (float)sum / numReadings;
}

// Function to calculate the standard deviation of distance readings
float getStdDevDistance(int numReadings,float average) {
  //float mean = getAverageDistance();
  float squareSum = 0;
  for (int i = 0; i < numReadings; i++) {
    float diff = (float)readings[i] - average;
    squareSum += diff * diff;
  }
  return sqrt(squareSum / numReadings);
}

float cylinderOrCuboidCalculattion(float variance)
{
  Serial.print("Variance is : ");
  Serial.println(variance);
  

  if (variance < 15) { // Adjust threshold as needed
    Serial.println("Object is likely a cylinder.");
    analogWrite(LED1_PIN,140);
  } else {
    Serial.println("Object is likely a rectangular.");
    analogWrite(LED2_PIN,140);
  }

   if (Serial.available() > 0) {
    // Read the incoming byte
    char command = Serial.read();

    // Check the command received
    if (command == '1') {
      Serial.println(variance);
     
      variance +=1;
      
    } else if (command == '0') {
      
      variance -=1;
      Serial.println(variance);
}

   }
}
  



void loop() {
  //lineFollow();
  sensorRead();
  //colorFront();
  if(section==0){
    
    int distancenew= distanceFront();
    //colorFront();
    if(distancenew<60){
      stop();
      delay(5000);
      // Set the flag to move to Section 1
      for (int i = 0; i < 2; i++) {
       colorFront();}

       if(task01Color=="green"){
        analogWrite(LED2_PIN,120);
        
        //analogWrite(LED1_PIN,0);
       }
       else {
        analogWrite(LED1_PIN,120);
        //analogWrite(LED2_PIN,0);
        
       }Serial.println("colourdetected");

  }else{  

  PID_control();
    Serial.println("PID Motion ");
  
  }

      while(colourdetected){
        offset=4;
        reversePID_Comtrol();
           if (digitalRead(left)==LOW){
                goBackward(1000);
                lineFollow();
       
                section=3;
                colourdetected=false;
                //stop();
     }
  }
  }
  

   if(section==3){
    baseSpeed = 100;
    offset=0;
    analogWrite(LED2_PIN,0);
    analogWrite(LED2_PIN,0);
    
    //sensorRead();
    //lineFollow();
     lineFollowWithLeftPriority();
    sensorRead();
    if((dVal[1]==1&&dVal[2]==1&&dVal[3]==1&&dVal[4]==1&&dVal[5]==1&&dVal[6]==1)){//||( (digitalRead(left)==LOW) &&(digitalRead(right)==LOW ) )    //dVal[1]==1&&dVal[2]==1&&dVal[3]==1&&dVal[4]==1&&dVal[5]==1&&dVal[6]==1
      //goForward(500);

   
      stop();
      delay(5000);
      goForward(1200);
      stop();
      delay(10000);
      
    for (int i = 0; i < 10; i++) {
        colorBottom();
        if(task02Color=="green"){
          countg+=1;
        }
        else{countb+=1;}
        if(countg<countb){
          task02Color="blue";
        }
        else{
          task02Color="green";
        }
        delay(1000);}

    if(task01Color==task02Color){
      goForward(1250);  //Turn Right
      TurnRight(1500);
      //key=2;
      section=4;

    }
    else{
      goForward(1250);  //Turn Left
      TurnLeft(1500);
      //key=1;
      section=4;

    }
    PID_control();
    }
   }

   

    if(section==4){
      
      analogWrite(LED2_PIN,0);
      lineFollow();

      takeReadings(50);
      analogWrite(LED2_PIN,140);

    }
}