#include <Wire.h>
#include <Stepper.h>

// Define the number of steps per revolution for your motor
const int stepsPerRevolution = 400; //originally 200

// Initialize the stepper library with the motor pins
Stepper myStepper(stepsPerRevolution, 2, 3, 4, 5); // Change pins as necessary

// Constants for AS5600
const int AS5600_Address = 0x36; // Default I2C address for AS5600
float offsetAngle; // Offset angle is used to set the initial angle to 0

unsigned long previousAngleDetectTime = 0;
unsigned long angleDetectionInterval = 200; // Interval for angle detection in milliseconds

bool magneticEncoderOn = false;

void setup() {
  delay(2000);
  Serial.begin(9600);   // Start the serial communication
  Wire.begin();         // Initialize I2C
  Serial.println("Ready to detect angle of rotation.");
  offsetAngle = readAS5600Angle();

  // Set the speed of the stepper motor (RPM)
  myStepper.setSpeed(60);
}

void loop() {
  // Read keyboard input
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read input until newline

    if (input == "flexion" || input == "f") { // Flexion towards patient's abdomen
      myStepper.step(stepsPerRevolution); // Move forward one revolution
      Serial.println("Moving forward");
    } 
    //digitalWrite(dirPin, HIGH);
      //dir = true;
      //stop = false; 
    if (input == "extension" || input == "r") { // Extension outwards
      myStepper.step(-stepsPerRevolution); // Move backward one revolution
      Serial.println("Moving backward");
    } 
    if (input == "halt") { // Stop the rotation of the stepper motor
      // No operation, the stepper motor will just stop receiving steps
      Serial.println("Halted");
    }
    if (input == "false") { // Turn on the magnetic encoder
      magneticEncoderOn = true;
      Serial.println("Magnetic encoder turned on");
    }
  }

  // Angle detection
  if (magneticEncoderOn) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousAngleDetectTime >= angleDetectionInterval) {
      previousAngleDetectTime = currentMillis;
      float angle = readAS5600Angle();
      Serial.print("Angle: ");
      Serial.println(angle);
    }
  }
}

float readAS5600Angle() {
  // Read 12-bit angle value from AS5600
  Wire.beginTransmission(AS5600_Address);
  Wire.write(0x0C); // Register for the high byte of the angle
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_Address, 2); // Request 2 bytes (high and low angle bytes)
  while (Wire.available() < 2); // Wait for bytes
  uint16_t highByte = Wire.read();
  uint16_t lowByte = Wire.read();
 
  uint16_t rawAngle = (highByte << 8) | lowByte; // Combine bytes
  float angle = ((float(rawAngle) * 360.0) / 4096.0) - offsetAngle; // Convert raw value to degrees

  // Convert the angle to be within 0 to 360 degrees
  if (angle < 0) {
    angle += 360;
  }
  if (angle > 360) {
    angle -= 360;
  }
  return angle;
}
