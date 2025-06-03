/*
 * Hydraulic Bridge Control System
 * Arduino-based automated bridge control with sensors and motors
 * Developed in C language for embedded systems
 */

#include <Arduino.h>
#include <Servo.h>

// Pin Definitions
#define MOTOR1_PIN_A 3      // Bridge Motor 1 - Forward
#define MOTOR1_PIN_B 4      // Bridge Motor 1 - Reverse
#define MOTOR2_PIN_A 5      // Bridge Motor 2 - Forward
#define MOTOR2_PIN_B 6      // Bridge Motor 2 - Reverse
#define PUMP_MOTOR_PIN 7    // Hydraulic Pump Motor
#define VALVE_PIN 8         // Hydraulic Valve Control

// Sensor Pins
#define ULTRASONIC_TRIG 9   // Ultrasonic Sensor Trigger
#define ULTRASONIC_ECHO 10  // Ultrasonic Sensor Echo
#define PRESSURE_SENSOR A0  // Pressure Sensor (Analog)
#define POSITION_SENSOR A1  // Bridge Position Sensor
#define IR_SENSOR 11        // IR Object Detection Sensor
#define BUTTON_UP 12        // Manual Control - Bridge Up
#define BUTTON_DOWN 13      // Manual Control - Bridge Down

// System Constants
#define BRIDGE_UP_POSITION 800    // Target position for bridge up
#define BRIDGE_DOWN_POSITION 200  // Target position for bridge down
#define SAFE_DISTANCE 20          // Safe distance in cm for object detection
#define MAX_PRESSURE 500          // Maximum safe pressure reading
#define MOTOR_SPEED 150           // PWM speed for bridge motors

// System States
enum BridgeState {
  BRIDGE_IDLE,
  BRIDGE_LIFTING,
  BRIDGE_LOWERING,
  BRIDGE_UP,
  BRIDGE_DOWN,
  SYSTEM_ERROR
};

// Global Variables
BridgeState currentState = BRIDGE_IDLE;
int bridgePosition = 0;
int targetPosition = BRIDGE_DOWN_POSITION;
bool manualMode = false;
bool systemError = false;
unsigned long lastSensorRead = 0;
unsigned long bridgeTimeout = 0;

void setup() {
  // Initialize Serial Communication
  Serial.begin(9600);
  Serial.println("Hydraulic Bridge Control System Starting...");
  
  // Initialize Motor Pins
  pinMode(MOTOR1_PIN_A, OUTPUT);
  pinMode(MOTOR1_PIN_B, OUTPUT);
  pinMode(MOTOR2_PIN_A, OUTPUT);
  pinMode(MOTOR2_PIN_B, OUTPUT);
  pinMode(PUMP_MOTOR_PIN, OUTPUT);
  pinMode(VALVE_PIN, OUTPUT);
  
  // Initialize Sensor Pins
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  pinMode(IR_SENSOR, INPUT);
  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  
  // Initialize System
  stopAllMotors();
  digitalWrite(VALVE_PIN, LOW);
  
  // Read initial bridge position
  bridgePosition = analogRead(POSITION_SENSOR);
  
  Serial.println("System Initialized Successfully");
  delay(1000);
}

void loop() {
  // Read sensors every 100ms
  if (millis() - lastSensorRead > 100) {
    readSensors();
    lastSensorRead = millis();
  }
  
  // Check for manual control
  checkManualControl();
  
  // Check for system errors
  if (checkSystemSafety()) {
    // Execute main control logic
    controlBridge();
  }
  
  // Print system status
  printSystemStatus();
  
  delay(50); // Main loop delay
}

void readSensors() {
  // Read bridge position
  bridgePosition = analogRead(POSITION_SENSOR);
  
  // Read pressure sensor
  int pressure = analogRead(PRESSURE_SENSOR);
  if (pressure > MAX_PRESSURE) {
    systemError = true;
    Serial.println("ERROR: Pressure too high!");
  }
}

int readUltrasonicDistance() {
  // Send ultrasonic pulse
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  
  // Read echo duration
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000);
  
  // Calculate distance in cm
  int distance = duration * 0.034 / 2;
  
  return distance;
}

void checkManualControl() {
  // Check manual control buttons
  if (digitalRead(BUTTON_UP) == LOW) {
    manualMode = true;
    targetPosition = BRIDGE_UP_POSITION;
    Serial.println("Manual Control: Bridge UP");
    delay(200); // Debounce
  }
  
  if (digitalRead(BUTTON_DOWN) == LOW) {
    manualMode = true;
    targetPosition = BRIDGE_DOWN_POSITION;
    Serial.println("Manual Control: Bridge DOWN");
    delay(200); // Debounce
  }
}

bool checkSystemSafety() {
  // Check for objects under bridge before lowering
  if (targetPosition == BRIDGE_DOWN_POSITION && currentState == BRIDGE_LOWERING) {
    int distance = readUltrasonicDistance();
    if (distance < SAFE_DISTANCE && distance > 0) {
      Serial.println("SAFETY: Object detected - stopping bridge");
      stopAllMotors();
      currentState = BRIDGE_IDLE;
      return false;
    }
  }
  
  // Check IR sensor for additional safety
  if (digitalRead(IR_SENSOR) == HIGH && currentState == BRIDGE_LOWERING) {
    Serial.println("SAFETY: IR sensor triggered - stopping bridge");
    stopAllMotors();
    currentState = BRIDGE_IDLE;
    return false;
  }
  
  // Check for system errors
  if (systemError) {
    stopAllMotors();
    digitalWrite(VALVE_PIN, LOW);
    currentState = SYSTEM_ERROR;
    return false;
  }
  
  return true;
}

void controlBridge() {
  switch (currentState) {
    case BRIDGE_IDLE:
      // Determine if bridge needs to move
      if (abs(bridgePosition - targetPosition) > 20) {
        if (targetPosition > bridgePosition) {
          startBridgeUp();
        } else {
          startBridgeDown();
        }
      }
      break;
      
    case BRIDGE_LIFTING:
      // Check if bridge reached up position
      if (bridgePosition >= targetPosition - 10) {
        stopAllMotors();
        currentState = BRIDGE_UP;
        Serial.println("Bridge UP position reached");
      }
      // Timeout check
      else if (millis() - bridgeTimeout > 15000) {
        stopAllMotors();
        currentState = SYSTEM_ERROR;
        Serial.println("ERROR: Bridge lift timeout");
      }
      break;
      
    case BRIDGE_LOWERING:
      // Check if bridge reached down position
      if (bridgePosition <= targetPosition + 10) {
        stopAllMotors();
        currentState = BRIDGE_DOWN;
        Serial.println("Bridge DOWN position reached");
      }
      // Timeout check
      else if (millis() - bridgeTimeout > 15000) {
        stopAllMotors();
        currentState = SYSTEM_ERROR;
        Serial.println("ERROR: Bridge lower timeout");
      }
      break;
      
    case BRIDGE_UP:
    case BRIDGE_DOWN:
      // Bridge in position, stay idle
      currentState = BRIDGE_IDLE;
      manualMode = false;
      break;
      
    case SYSTEM_ERROR:
      // Handle system error - require manual reset
      Serial.println("SYSTEM ERROR - Manual intervention required");
      delay(2000);
      break;
  }
}

void startBridgeUp() {
  Serial.println("Starting Bridge UP sequence");
  
  // Start hydraulic pump
  digitalWrite(PUMP_MOTOR_PIN, HIGH);
  delay(500); // Allow pump to build pressure
  
  // Open hydraulic valve
  digitalWrite(VALVE_PIN, HIGH);
  
  // Start bridge motors (lifting)
  analogWrite(MOTOR1_PIN_A, MOTOR_SPEED);
  digitalWrite(MOTOR1_PIN_B, LOW);
  analogWrite(MOTOR2_PIN_A, MOTOR_SPEED);
  digitalWrite(MOTOR2_PIN_B, LOW);
  
  currentState = BRIDGE_LIFTING;
  bridgeTimeout = millis();
}

void startBridgeDown() {
  Serial.println("Starting Bridge DOWN sequence");
  
  // Start hydraulic pump
  digitalWrite(PUMP_MOTOR_PIN, HIGH);
  delay(500); // Allow pump to build pressure
  
  // Open hydraulic valve for reverse operation
  digitalWrite(VALVE_PIN, HIGH);
  
  // Start bridge motors (lowering)
  digitalWrite(MOTOR1_PIN_A, LOW);
  analogWrite(MOTOR1_PIN_B, MOTOR_SPEED);
  digitalWrite(MOTOR2_PIN_A, LOW);
  analogWrite(MOTOR2_PIN_B, MOTOR_SPEED);
  
  currentState = BRIDGE_LOWERING;
  bridgeTimeout = millis();
}

void stopAllMotors() {
  // Stop all bridge motors
  digitalWrite(MOTOR1_PIN_A, LOW);
  digitalWrite(MOTOR1_PIN_B, LOW);
  digitalWrite(MOTOR2_PIN_A, LOW);
  digitalWrite(MOTOR2_PIN_B, LOW);
  
  // Stop hydraulic pump
  digitalWrite(PUMP_MOTOR_PIN, LOW);
  
  // Close hydraulic valve
  digitalWrite(VALVE_PIN, LOW);
  
  Serial.println("All motors stopped");
}

void printSystemStatus() {
  static unsigned long lastPrint = 0;
  
  // Print status every 2 seconds
  if (millis() - lastPrint > 2000) {
    Serial.println("=== SYSTEM STATUS ===");
    Serial.print("State: ");
    
    switch (currentState) {
      case BRIDGE_IDLE: Serial.println("IDLE"); break;
      case BRIDGE_LIFTING: Serial.println("LIFTING"); break;
      case BRIDGE_LOWERING: Serial.println("LOWERING"); break;
      case BRIDGE_UP: Serial.println("UP"); break;
      case BRIDGE_DOWN: Serial.println("DOWN"); break;
      case SYSTEM_ERROR: Serial.println("ERROR"); break;
    }
    
    Serial.print("Position: ");
    Serial.print(bridgePosition);
    Serial.print(" / Target: ");
    Serial.println(targetPosition);
    
    Serial.print("Distance: ");
    Serial.print(readUltrasonicDistance());
    Serial.println(" cm");
    
    Serial.print("Manual Mode: ");
    Serial.println(manualMode ? "YES" : "NO");
    
    Serial.println("==================");
    lastPrint = millis();
  }
}

// Interrupt service routine for emergency stop
void emergencyStop() {
  stopAllMotors();
  currentState = SYSTEM_ERROR;
  systemError = true;
  Serial.println("EMERGENCY STOP ACTIVATED!");
}

// Function to reset system after error
void resetSystem() {
  systemError = false;
  currentState = BRIDGE_IDLE;
  manualMode = false;
  bridgePosition = analogRead(POSITION_SENSOR);
  Serial.println("System Reset Complete");
}