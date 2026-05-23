/**
 * ESP32 + DUAL TMC5160 + 57AM23ED V2.0 (5-Bar Linkage Parallel Drive)
 * Open-loop stepper control.
 * Binary serial protocol with telemetry – both motors controlled.
 */

#include <Arduino.h>
#include <TMCStepper.h>
#include <SPI.h>
#include <FastAccelStepper.h>

// ── DRIVER 1 PINOUT (Left / Primary) ──────────────────────────────
#define EN_1_PIN     22
#define STEP_1_PIN   27
#define DIR_1_PIN    26
#define CS_1_PIN     5

// ── DRIVER 2 PINOUT (Right / Secondary) ───────────────────────────
#define EN_2_PIN     14
#define STEP_2_PIN   25
#define DIR_2_PIN    33
#define CS_2_PIN     17

// ── SHARED SPI BUS ─────────────────────────────────────────────────
#define SW_MOSI      23
#define SW_MISO      19
#define SW_SCK       18

// ── MOTOR SETTINGS (Ruitech 57AM23ED V2.0 – 5 A peak, 1.8°/step) ──
#define R_SENSE       0.022f
#define MOTOR_CURRENT 3500       // 3.5 A RMS – safe starting point for 5 A peak motor
                                 // Increase to 4000 if torque feels weak under load.
                                 // Do NOT exceed 4500 with TMC5160 on 0.022 R_SENSE.
#define MICROSTEPS    16

const float STEPS_PER_REV  = 200.0;
const float TOTAL_STEPS    = STEPS_PER_REV * MICROSTEPS;
const float RAD_TO_STEPS   = TOTAL_STEPS / (2.0 * M_PI);
const float STEPS_TO_RAD   = (2.0 * M_PI) / TOTAL_STEPS;

// ── GLOBAL OBJECTS ────────────────────────────────────────────────
TMC5160Stepper driver1(CS_1_PIN, R_SENSE);
TMC5160Stepper driver2(CS_2_PIN, R_SENSE);

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;

// ── SETUP ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 booted – 57AM23ED V2.0 open-loop");

  // Enable both drivers
  pinMode(EN_1_PIN, OUTPUT);
  digitalWrite(EN_1_PIN, LOW);
  pinMode(EN_2_PIN, OUTPUT);
  digitalWrite(EN_2_PIN, LOW);

  // SPI & TMC5160 init
  SPI.begin(SW_SCK, SW_MISO, SW_MOSI);

  driver1.begin(); driver1.rms_current(MOTOR_CURRENT); driver1.microsteps(MICROSTEPS); driver1.toff(4);
  driver2.begin(); driver2.rms_current(MOTOR_CURRENT); driver2.microsteps(MICROSTEPS); driver2.toff(4);

  // FastAccelStepper engine
  engine.init();

  // Stepper 1
  stepper1 = engine.stepperConnectToPin(STEP_1_PIN);
  if (stepper1) {
    stepper1->setDirectionPin(DIR_1_PIN);
    stepper1->setSpeedInHz(1000);
    stepper1->setAcceleration(500);
    Serial.println("Stepper1 OK");
  } else {
    Serial.println("Stepper1 FAIL");
  }

  // Stepper 2
  stepper2 = engine.stepperConnectToPin(STEP_2_PIN);
  if (stepper2) {
    stepper2->setDirectionPin(DIR_2_PIN);
    stepper2->setSpeedInHz(1000);
    stepper2->setAcceleration(500);
    Serial.println("Stepper2 OK");
  } else {
    Serial.println("Stepper2 FAIL");
  }
}

// ── MAIN LOOP ─────────────────────────────────────────────────────
void loop() {
  // ----- 1. RECEIVE COMMAND PACKET (10 bytes: 0xAA + 8 payload + 0xBB) -----
  while (Serial.available() >= 10) {
    if (Serial.read() != 0xAA) {
      continue;
    }

    uint8_t payload[8];
    Serial.readBytes(payload, 8);
    uint8_t end_marker = Serial.read();

    if (end_marker == 0xBB) {
      float target_a, target_b;
      memcpy(&target_a, &payload[0], 4);
      memcpy(&target_b, &payload[4], 4);

      Serial.printf("CMD: a=%.3f rad  b=%.3f rad\n", target_a, target_b);

      long target_steps_1 = round(target_a * RAD_TO_STEPS);
      long target_steps_2 = round(target_b * RAD_TO_STEPS*-1);

      if (stepper1) stepper1->moveTo(target_steps_1);
      if (stepper2) stepper2->moveTo(target_steps_2);
    } else {
      // Corrupt packet – flush
      while (Serial.available() > 0) Serial.read();
    }
  }

  // ----- 2. SEND TELEMETRY (50 Hz) ----------------------------------------
  static unsigned long last_telemetry_time = 0;
  if (millis() - last_telemetry_time >= 20) {
    last_telemetry_time = millis();

    if (stepper1 && stepper2) {
      float current_rad_1 = (float)stepper1->getCurrentPosition() * STEPS_TO_RAD;
      float current_rad_2 = (float)stepper2->getCurrentPosition() * STEPS_TO_RAD;

      uint8_t outbound_packet[10];
      outbound_packet[0] = 0xCC;
      memcpy(&outbound_packet[1], &current_rad_1, sizeof(float));
      memcpy(&outbound_packet[5], &current_rad_2, sizeof(float));
      outbound_packet[9] = 0xDD;

      Serial.write(outbound_packet, sizeof(outbound_packet));
    }
  }
}