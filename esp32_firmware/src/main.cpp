/**
 * ESP32-S3 Firmware for 4-DOF Robotic Arm
 * Simple Serial Protocol (compatible with any ESP32)
 * 
 * Protocol:
 *   Command: "J <j1> <j2> <j3> <j4>\n" (positions in radians)
 *   Response: "S <j1> <j2> <j3> <j4>\n" (current positions)
 * 
 * Connections:
 *   - Joint 1 (Base): GPIO 4
 *   - Joint 2 (Shoulder): GPIO 5
 *   - Joint 3 (Elbow): GPIO 6
 *   - Joint 4 (Wrist): GPIO 7
 */

#include <Arduino.h>
#include <ESP32Servo.h>

// ============== CONFIGURATION ==============
#define NUM_JOINTS 4
#define SERIAL_BAUD 115200
#define CONTROL_RATE_HZ 50
#define STATUS_RATE_HZ 20

// Pin definitions - ADJUST TO YOUR WIRING
const int SERVO_PINS[NUM_JOINTS] = {4, 5, 6, 7};

// Joint limits (radians) - must match your URDF
const float JOINT_MIN[NUM_JOINTS] = {-1.745, -0.873, -0.873, -1.571};
const float JOINT_MAX[NUM_JOINTS] = { 1.745,  1.745,  1.745,  1.571};

// Servo calibration (microseconds)
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

// Smooth motion settings
const float MAX_VELOCITY = 1.0;  // rad/s maximum joint velocity
const float SMOOTHING = 0.15;    // Lower = smoother but slower response

// ============== GLOBALS ==============
Servo servos[NUM_JOINTS];

// Current and target positions
float current_positions[NUM_JOINTS] = {0, 0, 0, 0};
float target_positions[NUM_JOINTS] = {0, 0, 0, 0};

// Timing
unsigned long last_control_time = 0;
unsigned long last_status_time = 0;
const unsigned long control_period_us = 1000000 / CONTROL_RATE_HZ;
const unsigned long status_period_ms = 1000 / STATUS_RATE_HZ;

// Serial buffer
String input_buffer = "";
bool connected = false;
unsigned long last_cmd_time = 0;

// Forward declaration
void setServoPosition(int joint, float position_rad);

// ============== SERVO CONTROL ==============
void initServos() {
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    
    for (int i = 0; i < NUM_JOINTS; i++) {
        servos[i].setPeriodHertz(50);
        servos[i].attach(SERVO_PINS[i], SERVO_MIN_US, SERVO_MAX_US);
    }
    
    // Move to home position
    for (int i = 0; i < NUM_JOINTS; i++) {
        setServoPosition(i, 0.0);
        current_positions[i] = 0.0;
        target_positions[i] = 0.0;
    }
}

void setServoPosition(int joint, float position_rad) {
    // Clamp to limits
    position_rad = constrain(position_rad, JOINT_MIN[joint], JOINT_MAX[joint]);
    
    // Convert radians to servo microseconds
    // Map from [JOINT_MIN, JOINT_MAX] to [SERVO_MIN_US, SERVO_MAX_US]
    float normalized = (position_rad - JOINT_MIN[joint]) / (JOINT_MAX[joint] - JOINT_MIN[joint]);
    int us = SERVO_MIN_US + (int)(normalized * (SERVO_MAX_US - SERVO_MIN_US));
    
    servos[joint].writeMicroseconds(us);
}

// ============== SMOOTH MOTION ==============
void updateMotion() {
    unsigned long now = micros();
    if (now - last_control_time < control_period_us) return;
    
    float dt = (now - last_control_time) / 1000000.0;
    last_control_time = now;
    
    for (int i = 0; i < NUM_JOINTS; i++) {
        // Calculate error
        float error = target_positions[i] - current_positions[i];
        
        // Limit velocity
        float max_step = MAX_VELOCITY * dt;
        if (error > max_step) error = max_step;
        if (error < -max_step) error = -max_step;
        
        // Apply smoothing (exponential filter)
        current_positions[i] += error * SMOOTHING + error * (1.0 - SMOOTHING) * 0.5;
        
        // Clamp to limits
        current_positions[i] = constrain(current_positions[i], JOINT_MIN[i], JOINT_MAX[i]);
        
        // Send to servo
        setServoPosition(i, current_positions[i]);
    }
}

// ============== SERIAL PROTOCOL ==============
void parseCommand(String cmd) {
    cmd.trim();
    
    if (cmd.startsWith("J ") || cmd.startsWith("j ")) {
        // Joint command: "J <j1> <j2> <j3> <j4>"
        float positions[NUM_JOINTS];
        int idx = 2;
        
        for (int i = 0; i < NUM_JOINTS; i++) {
            int space_idx = cmd.indexOf(' ', idx);
            if (space_idx == -1) space_idx = cmd.length();
            
            String val = cmd.substring(idx, space_idx);
            positions[i] = val.toFloat();
            idx = space_idx + 1;
        }
        
        // Set targets
        for (int i = 0; i < NUM_JOINTS; i++) {
            target_positions[i] = constrain(positions[i], JOINT_MIN[i], JOINT_MAX[i]);
        }
        
        connected = true;
        last_cmd_time = millis();
    }
    else if (cmd == "HOME" || cmd == "home") {
        // Home all joints
        for (int i = 0; i < NUM_JOINTS; i++) {
            target_positions[i] = 0.0;
        }
        Serial.println("OK HOME");
    }
    else if (cmd == "PING" || cmd == "ping") {
        Serial.println("PONG ESP32_ARM");
    }
    else if (cmd == "INFO" || cmd == "info") {
        Serial.println("INFO ESP32_ARM_CONTROLLER");
        Serial.print("JOINTS ");
        Serial.println(NUM_JOINTS);
        Serial.print("LIMITS ");
        for (int i = 0; i < NUM_JOINTS; i++) {
            Serial.print(JOINT_MIN[i], 3);
            Serial.print(" ");
            Serial.print(JOINT_MAX[i], 3);
            if (i < NUM_JOINTS - 1) Serial.print(" ");
        }
        Serial.println();
    }
}

void sendStatus() {
    unsigned long now = millis();
    if (now - last_status_time < status_period_ms) return;
    last_status_time = now;
    
    // Send current joint positions
    Serial.print("S ");
    for (int i = 0; i < NUM_JOINTS; i++) {
        Serial.print(current_positions[i], 4);
        if (i < NUM_JOINTS - 1) Serial.print(" ");
    }
    Serial.println();
}

void processSerial() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (input_buffer.length() > 0) {
                parseCommand(input_buffer);
                input_buffer = "";
            }
        } else {
            input_buffer += c;
            if (input_buffer.length() > 100) {
                input_buffer = "";  // Overflow protection
            }
        }
    }
}

// ============== SAFETY ==============
void checkConnection() {
    // If no command received for 2 seconds, stop movement
    if (connected && (millis() - last_cmd_time > 2000)) {
        connected = false;
        // Hold current position (don't go to home for safety)
        for (int i = 0; i < NUM_JOINTS; i++) {
            target_positions[i] = current_positions[i];
        }
    }
}

// ============== MAIN ==============
void setup() {
    // Built-in LED for status
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    
    // Initialize serial
    Serial.begin(SERIAL_BAUD);
    delay(100);
    
    // Initialize servos
    initServos();
    
    Serial.println("ESP32_ARM_READY");
    
    last_control_time = micros();
    last_status_time = millis();
    
    digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
    // Process incoming commands
    processSerial();
    
    // Update smooth motion
    updateMotion();
    
    // Send status at regular intervals
    sendStatus();
    
    // Check connection timeout
    checkConnection();
    
    // Blink LED when connected
    if (connected) {
        digitalWrite(LED_BUILTIN, (millis() / 500) % 2);
    } else {
        digitalWrite(LED_BUILTIN, LOW);
    }
}
