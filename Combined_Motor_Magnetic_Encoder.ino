#include <Wire.h>

// Motor pins
const int stepPin = 5;
const int dirPin = 2;
const int enPin = 8;

// Constants for AS5600
const int AS5600_Address = 0x36; // Default I2C address for AS5600
float offsetAngle; // Offset angle is used to set the initial angle to 0

unsigned long previousAngleDetectTime = 0;
unsigned long angleDetectionInterval = 200; // Interval for angle detection in milliseconds

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, LOW);
  
  Serial.begin(9600);   // Start the serial communication
  Wire.begin();         // Initialize I2C
  Serial.println("Ready to detect angle of rotation.");
  offsetAngle = readAS5600Angle();
}

void loop() {
  // Motor control (constant speed)
  digitalWrite(dirPin, HIGH); // Set direction for motor rotation
  digitalWrite(stepPin, HIGH); // Step motor
  delayMicroseconds(1600);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(2000);

  // Angle detection
  unsigned long currentMillis = millis();
  if (currentMillis - previousAngleDetectTime >= angleDetectionInterval) {
    previousAngleDetectTime = currentMillis;
    float angle = readAS5600Angle();
    Serial.print("Angle: ");
    Serial.println(angle);
  }
}

float readAS5600Angle() {
  // Read 12-bit angle value from AS5600
  Wire.beginTransmission(AS5600_Address);
  Wire.write(0x0C); // Register for the high byte of the angle
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_Address, 2); // Request 2 bytes (high and low angle bytes)
  while(Wire.available() < 2); // Wait for bytes
  uint16_t highByte = Wire.read();
  uint16_t lowByte = Wire.read();
 
  uint16_t rawAngle = (highByte << 8) | lowByte; // Combine bytes
  float angle = ((float(rawAngle) * 360.0) / 4096.0) - offsetAngle; // Convert raw value to degrees

  /**
  * Convert the angle to be within 0 to 360 degrees
  */
  if(angle < 0){
    angle += 360;
  }
  if(angle > 360){
    angle -= 360;
  }
  return angle;
}
