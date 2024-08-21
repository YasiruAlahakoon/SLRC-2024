// Define the LED pin
const int ledPin = 13;

void setup() {
  Serial.begin(9600);
  // Initialize the LED pin as an output
  pinMode(ledPin, OUTPUT);

  // Start with the LED off
  digitalWrite(ledPin, HIGH);
}

void loop() {
  // Check if there's serial data available
  if (Serial.available() > 0) {
    // Read the incoming byte
    char command = Serial.read();

    // Check the command received
    if (command == '1') {
      Serial.println("OFF");
      // Turn the Magnet Off
      digitalWrite(ledPin, HIGH);
    } else if (command == '0') {
      // Turn the Magnet ON
      digitalWrite(ledPin, LOW);
      Serial.println("ON");
    }
  }
}
