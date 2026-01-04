/**
 * ESP32-S3 Firmware for 4-DOF Robotic Arm + 2 Gripper Servos
 * Simple Serial Protocol (compatible with any ESP32)
 * 
 * Protocol:
 *   Command: "J <base> <j1> <j2> <j3> <grip1> <grip2>\n" (positions in degrees)
 *   Response: "S <base> <j1> <j2> <j3> <grip1> <grip2>\n" (current positions)
 * 
 * Connections:
 *   - Base Servo: GPIO 4   (80-130°)
 *   - Joint 1:    GPIO 5   (20-85°)
 *   - Joint 2:    GPIO 6   (60-85°)
 *   - Joint 3:    GPIO 7   (0-180°)
 *   - Gripper 1:  GPIO 15  (0-60°)
 *   - Gripper 2:  GPIO 16  (0-100°)
 */

#include <Arduino.h>
#include <ESP32Servo.h>

// ============== CONFIGURATION ==============
#define NUM_ARM_JOINTS 4
#define NUM_GRIPPERS 2
#define NUM_SERVOS (NUM_ARM_JOINTS + NUM_GRIPPERS)
#define SERIAL_BAUD 115200
#define CONTROL_RATE_HZ 100
#define STATUS_RATE_HZ 20

// Pin definitions - ADJUST TO YOUR WIRING
const int SERVO_PINS[NUM_SERVOS] = {4, 5, 6, 7, 15, 16};

// Joint limits in DEGREES
const float JOINT_MIN[NUM_SERVOS] = {80,  20, 60,   0,  0,   0};   // Base, J1, J2, J3, Grip1, Grip2
const float JOINT_MAX[NUM_SERVOS] = {130, 85, 85, 180, 60, 100};

// Home positions in DEGREES
const float HOME_POS[NUM_SERVOS] = {98, 85, 85, 90, 0, 0};

// Smooth motion settings
const float MAX_VELOCITY = 60.0;      // degrees/s
const float MAX_ACCELERATION = 120.0; // degrees/s²
const float SMOOTHING_ALPHA = 0.12;   // Filter coefficient

// ============== GLOBALS ==============
Servo servos[NUM_SERVOS];

// Current and target positions (in degrees)
float current_positions[NUM_SERVOS];
float target_positions[NUM_SERVOS];
float current_velocities[NUM_SERVOS] = {0};

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
void setServoAngle(int servo, float angle_deg);

// ============== SERVO CONTROL ==============
void initServos() {
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    
    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].setPeriodHertz(50);
        servos[i].attach(SERVO_PINS[i], 500, 2500);
        
        // Initialize to home position
        current_positions[i] = HOME_POS[i];
        target_positions[i] = HOME_POS[i];
        setServoAngle(i, HOME_POS[i]);
    }
}

void setServoAngle(int servo, float angle_deg) {
    // Clamp to limits
    angle_deg = constrain(angle_deg, JOINT_MIN[servo], JOINT_MAX[servo]);
    
    // Write angle directly (servo library handles degrees)
    servos[servo].write((int)angle_deg);
}

// ============== SMOOTH MOTION ==============
void updateMotion() {
    unsigned long now = micros();
    if (now - last_control_time < control_period_us) return;
    
    float dt = (now - last_control_time) / 1000000.0;
    last_control_time = now;
    
    static float smoothed_vel[NUM_SERVOS] = {0};
    
    for (int i = 0; i < NUM_SERVOS; i++) {
        // Calculate position error
        float error = target_positions[i] - current_positions[i];
        
        // P controller for desired velocity
        float desired_velocity = error * 4.0;
        desired_velocity = constrain(desired_velocity, -MAX_VELOCITY, MAX_VELOCITY);
        
        // Limit acceleration
        float velocity_error = desired_velocity - current_velocities[i];
        float max_accel_step = MAX_ACCELERATION * dt;
        velocity_error = constrain(velocity_error, -max_accel_step, max_accel_step);
        current_velocities[i] += velocity_error;
        
        // Smooth velocity
        smoothed_vel[i] += SMOOTHING_ALPHA * (current_velocities[i] - smoothed_vel[i]);
        
        // Update position
        current_positions[i] += smoothed_vel[i] * dt;
        current_positions[i] = constrain(current_positions[i], JOINT_MIN[i], JOINT_MAX[i]);
        
        // Stop at limits
        if (current_positions[i] <= JOINT_MIN[i] || current_positions[i] >= JOINT_MAX[i]) {
            current_velocities[i] = 0;
            smoothed_vel[i] = 0;
        }
        
        // Send to servo
        setServoAngle(i, current_positions[i]);
    }
}

// ============== SERIAL PROTOCOL ==============
void parseCommand(String cmd) {
    cmd.trim();
    
    if (cmd.startsWith("J ") || cmd.startsWith("j ")) {
        // Joint command: "J <base> <j1> <j2> <j3> <grip1> <grip2>"
        float positions[NUM_SERVOS];
        int idx = 2;
        
        for (int i = 0; i < NUM_SERVOS; i++) {
            int space_idx = cmd.indexOf(' ', idx);
            if (space_idx == -1) space_idx = cmd.length();
            
            String val = cmd.substring(idx, space_idx);
            positions[i] = val.toFloat();
            idx = space_idx + 1;
            
            if (idx >= cmd.length() && i < NUM_SERVOS - 1) {
                // Not enough values, keep current for remaining
                for (int j = i + 1; j < NUM_SERVOS; j++) {
                    positions[j] = target_positions[j];
                }
                break;
            }
        }
        
        // Set targets (clamped to limits)
        for (int i = 0; i < NUM_SERVOS; i++) {
            target_positions[i] = constrain(positions[i], JOINT_MIN[i], JOINT_MAX[i]);
        }
        
        connected = true;
        last_cmd_time = millis();
    }
    else if (cmd == "HOME" || cmd == "home") {
        for (int i = 0; i < NUM_SERVOS; i++) {
            target_positions[i] = HOME_POS[i];
        }
        Serial.println("OK HOME");
    }
    else if (cmd.startsWith("G1 ") || cmd.startsWith("g1 ")) {
        // Gripper 1 command: "G1 <angle>"
        float angle = cmd.substring(3).toFloat();
        target_positions[4] = constrain(angle, JOINT_MIN[4], JOINT_MAX[4]);
        connected = true;
        last_cmd_time = millis();
    }
    else if (cmd.startsWith("G2 ") || cmd.startsWith("g2 ")) {
        // Gripper 2 command: "G2 <angle>"
        float angle = cmd.substring(3).toFloat();
        target_positions[5] = constrain(angle, JOINT_MIN[5], JOINT_MAX[5]);
        connected = true;
        last_cmd_time = millis();
    }
    else if (cmd == "PING" || cmd == "ping") {
        Serial.println("PONG ESP32_ARM_6DOF");
    }
    else if (cmd == "INFO" || cmd == "info") {
        Serial.println("INFO ESP32_ARM_CONTROLLER_6DOF");
        Serial.print("JOINTS ");
        Serial.println(NUM_SERVOS);
        Serial.print("LIMITS ");
        for (int i = 0; i < NUM_SERVOS; i++) {
            Serial.print(JOINT_MIN[i], 1);
            Serial.print("-");
            Serial.print(JOINT_MAX[i], 1);
            if (i < NUM_SERVOS - 1) Serial.print(" ");
        }
        Serial.println();
        Serial.print("HOME ");
        for (int i = 0; i < NUM_SERVOS; i++) {
            Serial.print(HOME_POS[i], 1);
            if (i < NUM_SERVOS - 1) Serial.print(" ");
        }
        Serial.println();
    }
}

void sendStatus() {
    unsigned long now = millis();
    if (now - last_status_time < status_period_ms) return;
    last_status_time = now;
    
    // Send current positions
    Serial.print("S ");
    for (int i = 0; i < NUM_SERVOS; i++) {
        Serial.print(current_positions[i], 2);
        if (i < NUM_SERVOS - 1) Serial.print(" ");
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
                input_buffer = "";
            }
        }
    }
}

void checkConnection() {
    if (connected && (millis() - last_cmd_time > 2000)) {
        connected = false;
        // Hold current position on disconnect
        for (int i = 0; i < NUM_SERVOS; i++) {
            target_positions[i] = current_positions[i];
        }
    }
}

// ============== MAIN ==============
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    
    Serial.begin(SERIAL_BAUD);
    delay(100);
    
    initServos();
    
    Serial.println("ESP32_ARM_6DOF_READY");
    Serial.print("HOME: ");
    for (int i = 0; i < NUM_SERVOS; i++) {
        Serial.print(HOME_POS[i], 1);
        Serial.print(" ");
    }
    Serial.println();
    
    last_control_time = micros();
    last_status_time = millis();
    
    digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
    processSerial();
    updateMotion();
    sendStatus();
    checkConnection();
    
    if (connected) {
        digitalWrite(LED_BUILTIN, (millis() / 500) % 2);
    } else {
        digitalWrite(LED_BUILTIN, LOW);
    }
}
