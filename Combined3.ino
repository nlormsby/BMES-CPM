/*
   -------------------------------------------------------------------------------------
   HX711_ADC
   Arduino library for HX711 24-Bit Analog-to-Digital Converter for Weight Scales
   Olav Kallhovd sept2017
   -------------------------------------------------------------------------------------
*/

/*
   This example file shows how to calibrate the load cell and optionally store the calibration
   value in EEPROM, and also how to change the value manually.
   The result value can then later be included in your project sketch or fetched from EEPROM.

   To implement calibration in your project sketch the simplified procedure is as follow:
       LoadCell.tare();
       //place known mass
       LoadCell.refreshDataSet();
       float newCalibrationValue = LoadCell.getNewCalibration(pain_threshold);
*/

#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#include <Wire.h>
#include <Stepper.h>
#endif

// Linear Actuator start
// change this to the number of steps on your motor
#define STEPS 200

// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to
Stepper stepper(STEPS, 8, 9, 10, 11);

// the previous reading from the analog input
int previous = 0;

//pins:
const int HX711_dout = 4; //mcu > HX711 dout pin
const int HX711_sck = 5; //mcu > HX711 sck pin

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

'''
Calibration vars
'''
const int calVal_eepromAdress = 0;
unsigned long t = 0;
float pain_threshold = 100;

'''
Other Function vars
'''
const int AS5600_Address = 0x36; // Default I2C address for AS5600
const float angleThreshold = 180; //To be changed
float offsetAngle; //offset angle is used to set the initial angle to 0
int loopIter;
int prevAngle;


void setup() {
  Serial.begin(57600); delay(10);
  Serial.println();
  Serial.println("Starting...");

  LoadCell.begin();
  Wire.begin();         // Initialize I2C

  // For LA, set the speed of the motor to 30 RPMs
  stepper.setSpeed(5);
  Serial.begin(9600);

  '''
  Calibration setup
  '''
  //LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive
  unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(1.0); // user set calibration value (float), initial value 1.0 may be used for this sketch
    Serial.println("Startup is complete");
  }
  while (!LoadCell.update());

  '''
  The other function
  '''
  Serial.println("Ready to detect angle of rotation.");
  offsetAngle = readAS5600Angle();
  loopIter = 0;
  prevAngle = 0;

  calibrate(); //start calibration procedure
}

void loop() {

  // For LA, get the sensor value
  int val = analogRead(0);

  int step=map(val, 0, 1023, 0, STEPS);

  // move a number of steps equal to the change in the
  // sensor reading
  stepper.step(val - previous);

  // remember the previous value of the sensor
  previous = val;

  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell.getData(); 
      Serial.print("Load_cell output val: ");
      Serial.println(i);
      //TO DO-----------------------------
      if(i >= pain_threshold) {//Determine the condition to where the send signal, i is the output
        Serial.println("Terminate");
        exit(0); //The signal, either 1 or 0
      }
      //----------------------------------
      newDataReady = 0;
      t = millis();
    }
  }

  // receive command from serial terminal
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay(); //tare
    else if (inByte == 'r') calibrate(); //calibrate
    else if (inByte == 'c') changeSavedCalFactor(); //edit calibration value manually
  }

  // check if last tare operation is complete
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }

  '''
  other function
  '''
  float angle = readAS5600Angle();
  Serial.print("Angle: ");
  Serial.println(angle);
  Serial.println();
 
  //this if statement is used to see if patients leg angle
  //has changed too much
  if(loopIter == 5){
    if(abs(prevAngle - angle) > angleThreshold){
      Serial.println("Uh oh"); //to be changed obviously
      delay(1000);
    }
    loopIter = 0;
    prevAngle = angle;
  }
  loopIter++;
  delay(100); // Delay for readability, adjust as needed for responsiveness

}

'''
other function methods
'''
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


'''
Calibration methods
'''
void calibrate() {
  Serial.println("***");
  Serial.println("Start calibration:");
  Serial.println("Place the load cell an a level stable surface.");
  Serial.println("Remove any load applied to the load cell.");
  Serial.println("Send 't' from serial monitor to set the tare offset.");

  boolean _resume = false;
  while (_resume == false) {
    LoadCell.update();
    if (Serial.available() > 0) {
      if (Serial.available() > 0) {
        char inByte = Serial.read();
        if (inByte == 't') LoadCell.tareNoDelay();
      }
    }
    if (LoadCell.getTareStatus() == true) {
      Serial.println("Tare complete");
      _resume = true;
    }
  }

  Serial.println("Now, place your known mass on the loadcell.");
  Serial.println("Then send the weight of this mass (i.e. 100.0) from serial monitor.");

  pain_threshold = 100;//use to be 0
  //known_mass = Serial.parseFloat();

  _resume = false;
  while (_resume == false) {
    LoadCell.update();
    //if (Serial.available() > 0) {
      pain_threshold = 100;
      if (pain_threshold != 0) {
        Serial.print("Known mass is: ");
        Serial.println(pain_threshold);
        _resume = true;
      }
    //}
  }

  LoadCell.refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct
  float newCalibrationValue = LoadCell.getNewCalibration(pain_threshold); //get the new calibration value

  Serial.print("New calibration value has been set to: ");
  Serial.print(newCalibrationValue);
  //calVal = newCalibrationValue;
  Serial.println(", use this as calibration value (calFactor) in your project sketch.");
  Serial.print("Save this value to EEPROM adress ");
  Serial.print(calVal_eepromAdress);
  Serial.println("? y/n");

  _resume = false;
  while (_resume == false) {
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 'y') {
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(calVal_eepromAdress, newCalibrationValue);
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.commit();
#endif
        EEPROM.get(calVal_eepromAdress, newCalibrationValue);
        Serial.print("Value ");
        Serial.print(newCalibrationValue);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(calVal_eepromAdress);
        _resume = true;

      }
      else if (inByte == 'n') {
        Serial.println("Value not saved to EEPROM");
        _resume = true;
      }
    }
  }

  Serial.println("End calibration");
  Serial.println("***");
  Serial.println("To re-calibrate, send 'r' from serial monitor.");
  Serial.println("For manual edit of the calibration value, send 'c' from serial monitor.");
  Serial.println("***");
}

void changeSavedCalFactor() {
  float oldCalibrationValue = LoadCell.getCalFactor();
  boolean _resume = false;
  Serial.println("***");
  Serial.print("Current value is: ");
  Serial.println(oldCalibrationValue);
  Serial.println("Now, send the new value from serial monitor, i.e. 696.0");
  float newCalibrationValue;
  while (_resume == false) {
    if (Serial.available() > 0) {
      newCalibrationValue = Serial.parseFloat();
      if (newCalibrationValue != 0) {
        Serial.print("New calibration value is: ");
        Serial.println(newCalibrationValue);
        LoadCell.setCalFactor(newCalibrationValue);
        _resume = true;
      }
    }
  }
  _resume = false;
  Serial.print("Save this value to EEPROM adress ");
  Serial.print(calVal_eepromAdress);
  Serial.println("? y/n");
  while (_resume == false) {
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 'y') {
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(calVal_eepromAdress, newCalibrationValue);
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.commit();
#endif
        EEPROM.get(calVal_eepromAdress, newCalibrationValue);
        Serial.print("Value ");
        Serial.print(newCalibrationValue);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(calVal_eepromAdress);
        _resume = true;
      }
      else if (inByte == 'n') {
        Serial.println("Value not saved to EEPROM");
        _resume = true;
      }
    }
  }
  Serial.println("End change calibration value");
  Serial.println("***");
}
