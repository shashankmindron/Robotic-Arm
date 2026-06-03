#include <Arduino.h>
#include <FastAccelStepper.h>

// ==========================================
// PIN DEFINITIONS (T60 Drivers)
// ==========================================
const int PUL_A = 13;
const int DIR_A = 14;
const int ENA_A = 27;
const int ALM_A = 25;

const int PUL_B = 4;
const int DIR_B = 16;
const int ENA_B = 18;
const int ALM_B = 26;

// ==========================================
// SYSTEM OBJECTS
// ==========================================
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepperA = NULL;
FastAccelStepper *stepperB = NULL;

// Protocol Constraints
const uint8_t RX_HEADER = 0xAA;
const uint8_t RX_FOOTER = 0xBB;
const int RX_PACKET_SIZE = 18;

const uint8_t TX_HEADER = 0xCC;
const uint8_t TX_FOOTER = 0xDD;

uint8_t rx_buffer[RX_PACKET_SIZE];
int rx_index = 0;

// Struct for Jetson -> ESP32 (Commands)
#pragma pack(push, 1)
struct ArmCommand {
  uint8_t start_byte;
  float motor_a_rad;
  float motor_b_rad;
  float motor_a_hz;
  float motor_b_hz;
  uint8_t end_byte;
};
#pragma pack(pop)

// Struct for ESP32 -> Jetson (Feedback)
#pragma pack(push, 1)
struct ArmFeedback {
  uint8_t start_byte;
  uint8_t status_code; // 0 = Normal, 1 = Hardware Alarm
  float current_a_rad;
  float current_b_rad;
  uint8_t end_byte;
};
#pragma pack(pop)

ArmFeedback telemetry;

// Alarm Tracking
unsigned long alarm_trigger_time = 0;
bool is_in_alarm_state = false;
const uint8_t ALARM_ACTIVE_STATE = HIGH; // Change to HIGH if T60 uses NO logic

// Telemetry Timing
unsigned long last_telemetry_time = 0;

void setupSteppers() {
    engine.init();

    stepperA = engine.stepperConnectToPin(PUL_A);
    if (stepperA) {
        stepperA->setDirectionPin(DIR_A);
        stepperA->setEnablePin(ENA_A);
        stepperA->setAutoEnable(false); 
        stepperA->enableOutputs();
        stepperA->setAcceleration(20000); 
    }

    stepperB = engine.stepperConnectToPin(PUL_B);
    if (stepperB) {
        stepperB->setDirectionPin(DIR_B);
        stepperB->setEnablePin(ENA_B);
        stepperB->setAutoEnable(false);
        stepperB->enableOutputs();
        stepperB->setAcceleration(20000); 
    }
}

void processValidPacket(uint8_t* buffer) {
    float angleA, angleB, speedA, speedB;

    memcpy(&angleA, &buffer[1], 4);
    memcpy(&angleB, &buffer[5], 4);
    memcpy(&speedA, &buffer[9], 4);
    memcpy(&speedB, &buffer[13], 4);

    const float radToPulse = 3200.0f / TWO_PI;
    int32_t targetPulsesA = round(angleA * radToPulse);
    int32_t targetPulsesB = round(angleB * radToPulse);

    uint32_t speedHzA = (uint32_t)abs(speedA);
    uint32_t speedHzB = (uint32_t)abs(speedB);

    if (speedHzA > 0) stepperA->setSpeedInHz(speedHzA);
    if (speedHzB > 0) stepperB->setSpeedInHz(speedHzB);

    stepperA->moveTo(targetPulsesA);
    stepperB->moveTo(targetPulsesB);
}

void setup() {
    Serial.begin(115200);
    
    pinMode(ALM_A, INPUT_PULLUP);
    pinMode(ALM_B, INPUT_PULLUP);

    setupSteppers();
}

void loop() {
    // 1. RECEIVE COMMANDS
    if (!is_in_alarm_state) {
        while (Serial.available() > 0) {
            uint8_t incomingByte = Serial.read();

            if (rx_index == 0) {
                if (incomingByte == RX_HEADER) rx_buffer[rx_index++] = incomingByte;
            } else {
                rx_buffer[rx_index++] = incomingByte;
                if (rx_index == RX_PACKET_SIZE) {
                    if (rx_buffer[RX_PACKET_SIZE - 1] == RX_FOOTER) {
                        processValidPacket(rx_buffer); 
                    } else {
                        while(Serial.available()) Serial.read(); 
                    }
                    rx_index = 0; 
                }
            }
        }
    }

    // 2. T60 ALARM CHECK (50ms Anti-Noise Debounce)
    bool a_fault = false;//(digitalRead(ALM_A) == ALARM_ACTIVE_STATE);
    bool b_fault = false;//(digitalRead(ALM_B) == ALARM_ACTIVE_STATE);

    if (a_fault || b_fault) {
        if (alarm_trigger_time == 0) alarm_trigger_time = millis(); 
        else if (millis() - alarm_trigger_time > 50) { 
            if (!is_in_alarm_state) {
                stepperA->stopMove();
                stepperB->stopMove();
                is_in_alarm_state = true;
            }
        }
    } else {
        alarm_trigger_time = 0; 
    }

    // 3. TRANSMIT TELEMETRY (Every 20ms = 50Hz)
    if (millis() - last_telemetry_time >= 20) {
        last_telemetry_time = millis();

        telemetry.start_byte = TX_HEADER;
        telemetry.status_code = is_in_alarm_state ? 1 : 0;
        
        // Convert physical pulses back into Radians
        const float pulseToRad = TWO_PI / 3200.0f;
        telemetry.current_a_rad = stepperA->getCurrentPosition() * pulseToRad;
        telemetry.current_b_rad = stepperB->getCurrentPosition() * pulseToRad;
        
        telemetry.end_byte = TX_FOOTER;

        // Blast 11-byte binary packet directly down the USB cable
        Serial.write((uint8_t*)&telemetry, sizeof(ArmFeedback));
    }
}


// // Define the pin connection
// const int dataPin = 15; // GPIO 15 corresponds to D15 on most ESP32 boards
// int sensorValue = 0;    // Variable to store the data

// void setup() {
//   // Initialize serial communication at 115200 baud (standard for ESP32)
//   Serial.begin(250000);
  
//   // Configure the data pin as an input
//   pinMode(dataPin, INPUT);
// }

// void loop() {
//   // Read the state of the pin (HIGH or LOW / 1 or 0)
//   sensorValue = digitalRead(dataPin);
  
//   // Print the value to the Serial Monitor
//   Serial.print("Data from D15: ");
//   Serial.println(sensorValue);
  
//   // Wait for half a second before reading again to avoid flooding the monitor
//   delay(500); 
// }