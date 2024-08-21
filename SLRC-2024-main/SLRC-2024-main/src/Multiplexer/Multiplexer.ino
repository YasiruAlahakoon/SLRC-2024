#include <Wire.h>
#include <VL53L0X.h>
#include <TCA9548.h>

// Create a TCA9548 object with the I2C address 0x70
TCA9548 multiplexer(0x70, &Wire);

// Create a VL53L0X object
VL53L0X sensor;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize the multiplexer
  multiplexer.begin();

  // Initialize the VL53L0X sensor on channel 0
  multiplexer.selectChannel(0);
  sensor.init();
  sensor.setTimeout(500);  // Set timeout here
  sensor.startContinuous(); // Start continuous ranging measurements

  // Check if the sensor is correctly connected
  if (!sensor.timeoutOccurred()) {
    Serial.println("VL53L0X sensor initialized and correctly connected!");
  } else {
    Serial.println("Failed to initialize VL53L0X sensor!");
    while (1);
  }
}

void loop() {
  // Select the channel of the sensor
  multiplexer.selectChannel(0);

  // Read distance
  uint16_t distance = sensor.readRangeContinuousMillimeters();

  if (sensor.timeoutOccurred()) {
    Serial.println("VL53L0X timeout!");
  } else {
    // Print distance
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");
  }

delay(2000);
}