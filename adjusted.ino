#include <Stepper.h>

const int stepsPerRevolution = 200;  // Change this to fit the number of steps per revolution for your motor
//const int stepsToMove = 200; // Set a constant number of steps to move
const float distancePerStep = 0.040;


// Initialize the stepper library on pins 3, 4, 5, and 6
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

void setup() {
  // Set the speed at 60 rpm
  myStepper.setSpeed(60);

  // Initialize the serial port
  Serial.begin(9600);
  delay(1000); // Delay to allow the Serial monitor to open
  //Serial.print("\033[2J\033[H");
  Serial.println("Enter command (C for clockwise, W for counterclockwise) + Distance to travel in mm:");
}

void loop() {
  //myStepper.step(stepsToMove);
  if (Serial.available()) {
    processCommand();
  }
}

void processCommand() {
  // Read the entire input as a string
  String input = Serial.readStringUntil('\n');

  // Print the received input for debugging
  Serial.print("Received input: ");
  Serial.println(input);

  // Extract the direction command (first character) and the distance (remaining part of the string)
  char command = input.charAt(0);
  float distance = input.substring(1).toFloat();

  // Calculate the number of steps required to move the specified distance
  int stepsToMove = distance / distancePerStep;

  // Print the command, distance, and number of steps for debugging
  Serial.print("Command: ");
  Serial.print(command);
  Serial.print(", Distance: ");
  Serial.print(distance);
  Serial.print(", Steps to move: ");
  Serial.println(stepsToMove);

  // Move the stepper motor the calculated number of steps in the specified direction
  if (command == 'C') {
    myStepper.step(stepsToMove); // Move forward the specified steps
    Serial.println("Moved clockwise");
  } else if (command == 'W') {
    myStepper.step(-stepsToMove); // Move backward the specified steps
    Serial.println("Moved counterclockwise");
  } else {
    Serial.println("Unrecognized command. Enter 'C' for clockwise or 'W' for counterclockwise followed by the distance:");
  }

  // Prompt for the next command
  Serial.println("Enter command (C for clockwise, W for counterclockwise) followed by the distance:");
}