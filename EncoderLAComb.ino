#include <Wire.h>




// Constants for AS5600
const int AS5600_Address = 0x36; // Default I2C address for AS5600
const float angleThreshold = 30; //To be changed
float offsetAngle; //offset angle is used to set the initial angle to 0
int loopIter;
int prevAngle;
float finalBreak = 180;
float gotFolded = 90;

// Constants for Motor
const int stepPin = 6; 
const int dirPin = 2; 
const int enPin = 8;
bool motorDir = false;
int motorCount = 0;


void setup() {
  Serial.begin(57600);   // Start the serial communication
  Wire.begin();         // Initialize I2C
  Serial.println("Ready to detect angle of rotation.");
  offsetAngle = readAS5600Angle();
  loopIter = 0;
  prevAngle = 0;

  // Motor time
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(enPin,OUTPUT);
  digitalWrite(enPin,LOW);
  digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
}




void loop() {
  float angle = readAS5600Angle();
  Serial.print("Angle: ");
  Serial.println(angle);
 
  //this if statement is used to see if patients leg angle
  //has changed too much
  if(loopIter == 5){
    double changedAngle = abs(prevAngle - angle);
    if(changedAngle > angleThreshold && changedAngle < 330){
      Serial.println("Uh oh"); //to be changed obviously
      delay(1000);
    }
    loopIter = 0;
    prevAngle = angle;
  }
  loopIter++;
  delay(100); // Delay for readability, adjust as needed for responsiveness

  // Motor Code
  if (motorDir == false) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(1600); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(2000);
    motorCount++; 
  }
  if (motorDir == true) {
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(1600);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(2000);
    motorCount++;
  }
  if (motorCount == 4800) {
    motorCount = 0;
    motorDir = !motorDir;
    delay(1000); // One second delay
    if (motorDir == false) {
       digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
    }
    if (motorDir == true) {
      digitalWrite(dirPin,LOW); //Changes the direction of rotation
    }
  }
  // delay(1000);
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
  float angle = ((float(rawAngle)* 360.0) / 4096.0) - offsetAngle; // Convert raw value to degrees




  /**
  * converts the angle to be within 0 to 360 degrees
  */
  if(angle < 0){
    angle = angle + 360;
  }
  if(angle > 360){
    angle = angle - 360;
  }
  return angle;
}
