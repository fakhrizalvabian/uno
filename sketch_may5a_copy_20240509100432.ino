#include <AccelStepper.h>

// Define motor pins
#define MOTOR_PIN_STEP 3
#define MOTOR_PIN_DIR 4

// Define limit switch pins
#define LIMIT_SWITCH_OPEN 8
#define LIMIT_SWITCH_SLOW 6
#define LIMIT_SWITCH_CLOSE 5

// Define IR motion sensor pin
#define IR_SENSOR_PIN 7
int pirstate = LOW ;


// Define states of the FSM
enum State {
  IDLE,
  OPENING,
  CLOSING,
  SLOW,
  STOPPED,
  ERROR_STATE
};

// Initialize state
State currentState = IDLE;

// Define variables for motor control
AccelStepper stepper(AccelStepper::DRIVER, MOTOR_PIN_STEP, MOTOR_PIN_DIR);

// Flag to track state initialization
bool stateTime = false;

// Timer variables
unsigned long stateStartTime = 0;

void setup() {
  // Set up serial communication
  Serial.begin(115200);

  // Set up motor parameters
  stepper.setMaxSpeed(5000);
  stepper.setAcceleration(100);

  // Set up limit switch pins
  pinMode(LIMIT_SWITCH_OPEN, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_CLOSE, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_SLOW, INPUT_PULLUP);

  // Set up IR motion sensor pin
  pinMode(IR_SENSOR_PIN, INPUT);

}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    Serial.print("Received command: ");
    Serial.println(command);
    if (command == '1' && currentState == IDLE) {
      Serial.println("Transitioning from IDLE to OPENING");
      currentState = OPENING;
      stateTime = false;
    }
  }
  // State machine logic
  switch (currentState) {
    case IDLE:
      Serial.println("IDLE");

      stepper.stop(); // Ensure motor is stopped
        
      
      while (Serial.available() > 0) {
          Serial.print(Serial.read());
          currentState = OPENING;
          stateTime = false;
        }
      break;

    case OPENING:
      if (!stateTime) {
        Serial.println("OPENING");
        stateTime = true;
      }
      // Move motor in the opening direction
      stepper.setSpeed(3000); // Set speed in steps per second (adjust as needed)
      stepper.runSpeed(); // Continuously move at the set speed

      // Check for transition conditions
      if (digitalRead(LIMIT_SWITCH_OPEN) == LOW) {
        currentState = STOPPED;
        stateTime = false;
      }
      break;

    case CLOSING:
      if (!stateTime) {
        Serial.println("CLOSING");
        stateTime = true;
        stateStartTime = millis(); // Record the start time of the state
      }

      // Move motor in the closing direction
      stepper.setSpeed(-3000); // Set speed in steps per second (adjust as needed)
      stepper.runSpeed(); // Continuously move at the set speed
      
      if (millis() - stateStartTime > 4500){
      // Check for transition conditions
      if (digitalRead(LIMIT_SWITCH_SLOW) == LOW) {
        currentState = SLOW;
        stateTime = false;
      }
      }
      
      if (digitalRead(IR_SENSOR_PIN) == HIGH) {
        stepper.setSpeed(0); // Set speed in steps per second (adjust as needed)
        stepper.runSpeed(); // Continuously move at the set speed
        delay (2000);
        currentState = OPENING;
        stateTime = false;
      }
      
      break;

    case SLOW:
      if (!stateTime) {
        Serial.println("SLOW");
        stateTime = true;
      }
      stepper.setSpeed(-500); // Set speed in steps per second (adjust as needed)
      stepper.runSpeed(); // Continuously move at the set speed

      if (millis() - stateStartTime > 4500){
      // Check for transition conditions
      if (digitalRead(LIMIT_SWITCH_CLOSE) == LOW) {
        currentState = IDLE;
        stateTime = false;
      } 
      }
      break;

    case STOPPED:
      if (!stateTime) {
        Serial.println("STOPPED");
        stateTime = true;
      }
      stepper.stop();
      delay(4000);
      currentState = CLOSING;
      stateTime = false;
      break;

    case ERROR_STATE:
      stepper.stop();
      
    break;
  }
/*
  // Print debug information
  Serial.print("State: ");
  Serial.println(currentState);*/

}
