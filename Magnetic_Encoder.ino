#include <Wire.h>




// Constants for AS5600
const int AS5600_Address = 0x36; // Default I2C address for AS5600
const float angleThreshold = 30; //To be changed
float offsetAngle; //offset angle is used to set the initial angle to 0
int loopIter;
int prevAngle;




void setup() {
  Serial.begin(9600);   // Start the serial communication
  Wire.begin();         // Initialize I2C
  Serial.println("Ready to detect angle of rotation.");
  offsetAngle = readAS5600Angle();
  loopIter = 0;
  prevAngle = 0;
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
