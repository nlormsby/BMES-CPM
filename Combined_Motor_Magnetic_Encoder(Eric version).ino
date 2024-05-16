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

// Motor controls
unsigned long dist = 400; // How many steps we take // original 4800
bool dir = false; // direction the motor is going (true is forward)
unsigned long distIter = 0; // What step we are on
unsigned long highDelay = 1600;
unsigned long lowDelay = 2000;
bool start = true;
bool startUp = true;
unsigned long count = 0;


void setup() {
  delay(2000);
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
  if (start == true) {
    digitalWrite(dirPin,HIGH); //Changes the direction of rotation//Nick's initialization
    //Eric's addition
      //while not stop
        //increment leg
        //count increase +increment value(0.25?)
      //assign dist = count
      if (Serial.available() > 0) {
        char inByte = Serial.read();
        if (inByte == 'w'){
          digitalWrite(dirPin,HIGH); //move leg up
          digitalWrite(stepPin, HIGH); // Step motor
          delayMicroseconds(highDelay);
          digitalWrite(stepPin, LOW);
          delayMicroseconds(lowDelay);
          count++;
        }
        else if  (inByte == 's'){
          digitalWrite(dirPin,LOW); //move leg down
          digitalWrite(stepPin, HIGH); // Step motor
          delayMicroseconds(highDelay);
          digitalWrite(stepPin, LOW);
          delayMicroseconds(lowDelay);
          count--;
        }
        else if(inByte == 'd'){
          startUp = false;
        }
      }
    //
    if(startUp == false){
      dist = count;
      start = false;
    }
    
  }else{
  // Motor change direction check
  if (distIter >= dist) {
    if (dir == true) {
      delay(1000); // One second delay
      digitalWrite(dirPin,HIGH); //Changes the direction of rotation
    }
    if (dir == false) {
      delay(1000); // One second delay
      digitalWrite(dirPin,LOW); //Changes the direction of rotation
    }
    dir = !dir;
    distIter = 0;
  }

  // Motor control (constant speed)
  // digitalWrite(dirPin, HIGH); // Set direction for motor rotation
  digitalWrite(stepPin, HIGH); // Step motor
  delayMicroseconds(highDelay);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(lowDelay);
  distIter++;

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
}
