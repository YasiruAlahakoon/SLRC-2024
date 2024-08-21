#include <Wire.h>
#include <VL53L0X.h>
#include <TCA9548.h>
#include "Adafruit_TCS34725.h"

#define led_01 A0
#define led_02 A1

/* Connect SCL    to analog 5
   Connect SDA    to analog 4
   Connect VDD    to 3.3V DC
   Connect GROUND to common ground */

/* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();

/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);


// Create a TCA9548 object with the I2C address 0x70
TCA9548 multiplexer(0x70, &Wire);

const int MAX_READINGS = 50;  // Define the maximum number of readings
int readings[MAX_READINGS];   // Declare and initialize the readings array


// Create two VL53L0X objects
VL53L0X sensor1;
VL53L0X sensor2;

uint16_t distance2=0;

void setup() {

  pinMode(led_01, OUTPUT);
  pinMode(led_02, OUTPUT);
  Serial.begin(9600);
  Wire.begin();

  // Initialize the multiplexer
  multiplexer.begin();

  // Initialize the VL53L0X sensors on channels 0 and 2
  multiplexer.selectChannel(1);
  sensor1.setAddress(0x29); // Set the correct I2C address for the sensor
  sensor1.init();
  sensor1.setTimeout(500);  // Set timeout here
  sensor1.startContinuous(); // Start continuous ranging measurements*/

  multiplexer.selectChannel(2);
  sensor2.setAddress(0x2A); // Set the correct I2C address for the sensor
  sensor2.init();
  sensor2.setTimeout(500);  // Set timeout here
  sensor2.startContinuous(); // Start continuous ranging measurements

  // Check if the sensors are correctly connected
  multiplexer.selectChannel(0);
  if (!sensor1.timeoutOccurred() && !sensor2.timeoutOccurred()) {
    Serial.println("VL53L0X sensors initialized and correctly connected!");
  } else {
    Serial.println("Failed to initialize VL53L0X sensors!");
    while (1);
  }

  /*
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }*/

}

void loop() {
  
  /*
  //Select the channel of the first sensor
  multiplexer.selectChannel(1);

  // Read distance from the first sensor
  uint16_t distance1 = sensor1.readRangeContinuousMillimeters();

  if (sensor1.timeoutOccurred()) {
    Serial.println("VL53L0X timeout on channel 0!");
  } else {
    // Print distance from the first sensor
    Serial.print("Distance 1: ");
    Serial.print(distance1);
    Serial.println(" mm");
  }*/

  // Select the channel of the second sensor
  multiplexer.selectChannel(2);

  // Read distance from the second sensor
  uint16_t distance2 = sensor2.readRangeContinuousMillimeters();

  if (sensor2.timeoutOccurred()) {
    Serial.println("VL53L0X timeout on channel 2!");
  } else {
    // Print distance from the second sensor
    Serial.print("Distance 2: ");
    Serial.print(distance2);
    Serial.println(" mm");
  }

  /*multiplexer.selectChannel(0);
  delay(2000);
    uint16_t r, g, b, c, colorTemp, lux;

  tcs.getRawData(&r, &g, &b, &c);
  // colorTemp = tcs.calculateColorTemperature(r, g, b);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);

  Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");
  delay(100); */


  takeReadings(50);
}

 //take a specified number of readings and store them in the array
void takeReadings(int numReadings) {
  Serial.println(" Readings are started to take");
  for (int i = 0; i < numReadings; i++) {
    int distance = distance2;      //distanceSide();120000/50
    if (distance >= 0) {
      readings[i] = distance;
    }
    delay(500);
    
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
    analogWrite(led_01,150);
  } else {
    Serial.println("Object is likely a rectangular.");
    analogWrite(led_02,150);
  }
}
