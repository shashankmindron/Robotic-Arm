/**
 * ESP32 + DUAL TMC5160 + 57AM23ED V2.0 (5-Bar Linkage Parallel Drive)
 * CLOSED-LOOP PID CONTROL using encoder feedback via X_ENC.
 * 
 * - Receives target angles (radians) from Jetson over serial.
 * - Reads actual position from TMC5160's X_ENC register.
 * - Runs PID loop at 500 Hz.
 * - Writes corrected target to XTARGET.
 * - Monitors ENC_STATUS for deviation warning (stall/slip).
 * - Sends telemetry back to Jetson.
 */

#include <Arduino.h>
#include <TMCStepper.h>
#include <SPI.h>

// ── DRIVER 1 PINOUT (Left / Primary) ──────────────────────────────
#define EN_1_PIN     22
#define CS_1_PIN     5

// ── DRIVER 2 PINOUT (Right / Secondary) ───────────────────────────
#define EN_2_PIN     14
#define CS_2_PIN     17

// ── SHARED SPI BUS ─────────────────────────────────────────────────
#define SW_MOSI      23
#define SW_MISO      19
#define SW_SCK       18

// ── MOTOR SETTINGS (based on your answers) ────────────────────────
#define MOTOR_FULL_STEPS    200
#define MICROSTEPS          16
#define MICROSTEPS_PER_REV  (MOTOR_FULL_STEPS * MICROSTEPS)   // = 3200
#define RAD_TO_MICROSTEP     (MICROSTEPS_PER_REV / (2.0 * M_PI))
#define MICROSTEP_TO_RAD     (2.0 * M_PI / MICROSTEPS_PER_REV)

// ── ENCODER SETTINGS (1000 lines → 4000 counts/rev after 4x decoding) ──
#define ENCODER_COUNTS_PER_REV  4000

// Calculate ENC_CONST = (motor microsteps/rev) / (encoder counts/rev) * 65536
// = (3200 / 4000) * 65536 = 0.8 * 65536 = 52428.8 ≈ 52429
#define ENC_CONST_VALUE  52429   // 0xCCCD

// ── PID SETTINGS ──────────────────────────────────────────────────
#define PID_FREQ_HZ      500
#define PID_INTERVAL_US  (1000000 / PID_FREQ_HZ)   // 2000 µs = 2 ms

// Start with P-only; you will tune these later
float Kp = 0.5;
float Ki = 0.0;
float Kd = 0.0;

// Output limits (radians per cycle)
#define MAX_CORRECTION_RAD  0.2

// ── COMMUNICATION ─────────────────────────────────────────────────
#define SERIAL_BAUD       115200
#define PACKET_START      0xAA
#define PACKET_END        0xBB
#define TELEMETRY_START   0xCC
#define TELEMETRY_END     0xDD

// ── GLOBAL VARIABLES ──────────────────────────────────────────────
TMC5160Stepper driver1(CS_1_PIN, 0.022f);   // R_SENSE = 0.022
TMC5160Stepper driver2(CS_2_PIN, 0.022f);

// Target angles (radians) received from Jetson
float target_rad_1 = 0.0, target_rad_2 = 0.0;

// Actual angles from encoder (radians)
float actual_rad_1 = 0.0, actual_rad_2 = 0.0;

// PID state variables
float prev_error_1 = 0.0, prev_error_2 = 0.0;
float integral_1 = 0.0, integral_2 = 0.0;

// Safety flags
bool fault_1 = false, fault_2 = false;

// -------------------------------------------------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.println("ESP32 closed‑loop PID controller starting...");

  // 1. Enable both drivers
  pinMode(EN_1_PIN, OUTPUT);
  pinMode(EN_2_PIN, OUTPUT);
  digitalWrite(EN_1_PIN, LOW);   // LOW = enabled (check your hardware)
  digitalWrite(EN_2_PIN, LOW);

  // 2. Initialise SPI bus
  SPI.begin(SW_SCK, SW_MISO, SW_MOSI);

  // 3. Initialise TMC5160 drivers
  driver1.begin();      // SPI begin
  driver1.rms_current(3500);   // 3.5 A RMS
  driver1.microsteps(MICROSTEPS);
  driver1.toff(4);              // required for stealthChop / spreadCycle
  driver1.en_spreadCycle(0);    // 0 = stealthChop (quiet), 1 = spreadCycle (more torque)
  // You can change to spreadCycle if needed.

  driver2.begin();
  driver2.rms_current(3500);
  driver2.microsteps(MICROSTEPS);
  driver2.toff(4);
  driver2.en_spreadCycle(0);

  // 4. Configure encoder interface for both drivers
  //    ENC_CONST scaling factor
  driver1.ENC_CONST(ENC_CONST_VALUE);
  driver2.ENC_CONST(ENC_CONST_VALUE);
  //    ENCMODE: default mode (0) – count on both edges, active high
  driver1.ENCMODE(0);
  driver2.ENCMODE(0);

  Serial.print("ENC_CONST set to ");
  Serial.println(ENC_CONST_VALUE, HEX);
  Serial.println("Encoder interface ready.");

  // 5. Optional: set current motor position as zero (if you have homing later)
  //    For now, we read whatever position the encoder reports.
  //    You can also set XACTUAL or XTARGET to zero, but encoder zero is fixed by mechanical position.

  Serial.println("Setup complete. Waiting for Jetson commands...");
}

// -------------------------------------------------------------------
// PID update for one joint (called in timer)
float update_pid(float target, float actual, float dt, float *prev_error, float *integral) {
  float error = target - actual;
  *integral += error * dt;
  // Anti‑windup: clamp integral to reasonable range (e.g., ±1 rad)
  if (*integral > 1.0) *integral = 1.0;
  if (*integral < -1.0) *integral = -1.0;
  float derivative = (error - *prev_error) / dt;
  float output = Kp * error + Ki * (*integral) + Kd * derivative;
  if (output > MAX_CORRECTION_RAD) output = MAX_CORRECTION_RAD;
  if (output < -MAX_CORRECTION_RAD) output = -MAX_CORRECTION_RAD;
  *prev_error = error;
  return output;
}

// -------------------------------------------------------------------
// Read packet from Jetson (blocking, with timeout)
bool readJetsonPacket() {
  if (Serial.available() < 10) return false;   // minimum packet size (1+8+1)

  // Look for start byte
  if (Serial.read() != PACKET_START) return false;

  uint8_t payload[8];
  if (Serial.readBytes(payload, 8) != 8) return false;

  if (Serial.read() != PACKET_END) return false;

  // Convert payload to two floats
  float new_target_1, new_target_2;
  memcpy(&new_target_1, &payload[0], 4);
  memcpy(&new_target_2, &payload[4], 4);

  // Optional: apply joint limits if you have them (none specified, but we add a sanity limit)
  const float MAX_JOINT_RAD = 10.0;   // about 570 degrees – safe upper bound
  if (abs(new_target_1) < MAX_JOINT_RAD && abs(new_target_2) < MAX_JOINT_RAD) {
    target_rad_1 = new_target_1;
    target_rad_2 = new_target_2;
    return true;
  } else {
    Serial.println("WARNING: target angle out of range, ignored");
    return false;
  }
}

// -------------------------------------------------------------------
// Send telemetry to Jetson
void sendTelemetry() {
  uint8_t packet[10];
  packet[0] = TELEMETRY_START;
  memcpy(&packet[1], &actual_rad_1, 4);
  memcpy(&packet[5], &actual_rad_2, 4);
  packet[9] = TELEMETRY_END;
  Serial.write(packet, 10);
}

// -------------------------------------------------------------------
// Main loop
void loop() {
  static unsigned long last_pid_time = micros();
  static unsigned long last_telemetry_time = millis();

  // 1. Receive new target angles from Jetson (when available)
  readJetsonPacket();

  // 2. Run PID at fixed frequency (500 Hz)
  unsigned long now = micros();
  if (now - last_pid_time >= PID_INTERVAL_US) {
    float dt = (now - last_pid_time) / 1000000.0;
    last_pid_time = now;
    if (dt > 0.01) dt = 0.01;   // limit dt to avoid spikes

    // --- Read actual encoder positions (in microsteps) from X_ENC ---
    int32_t enc_microsteps_1 = driver1.X_ENC();
    int32_t enc_microsteps_2 = driver2.X_ENC();

    // Convert to radians
    actual_rad_1 = enc_microsteps_1 * MICROSTEP_TO_RAD;
    actual_rad_2 = enc_microsteps_2 * MICROSTEP_TO_RAD;

    // --- PID calculation for joint 1 ---
    float correction_rad_1 = update_pid(target_rad_1, actual_rad_1, dt,
                                        &prev_error_1, &integral_1);
    // --- PID calculation for joint 2 ---
    float correction_rad_2 = update_pid(target_rad_2, actual_rad_2, dt,
                                        &prev_error_2, &integral_2);

    // --- Compute corrected target in microsteps ---
    int32_t target_microsteps_1 = (int32_t)(target_rad_1 * RAD_TO_MICROSTEP);
    int32_t target_microsteps_2 = (int32_t)(target_rad_2 * RAD_TO_MICROSTEP);
    int32_t correction_microsteps_1 = (int32_t)(correction_rad_1 * RAD_TO_MICROSTEP);
    int32_t correction_microsteps_2 = (int32_t)(correction_rad_2 * RAD_TO_MICROSTEP);
    int32_t new_target_1 = target_microsteps_1 + correction_microsteps_1;
    int32_t new_target_2 = target_microsteps_2 + correction_microsteps_2;

    // --- Write corrected target to XTARGET register ---
    driver1.XTARGET(new_target_1);
    driver2.XTARGET(new_target_2);

    // --- Safety: check deviation warning ---
    uint8_t status1 = driver1.ENC_STATUS();
    uint8_t status2 = driver2.ENC_STATUS();
    if (status1 & 0x02) {   // bit 1 = deviation_warn
      if (!fault_1) {
        fault_1 = true;
        Serial.println("FAULT: Motor 1 deviation warning (stall/slip)");
        // Stop motor: set XTARGET to current position
        driver1.XTARGET(enc_microsteps_1);
        // Optional: disable driver
        // digitalWrite(EN_1_PIN, HIGH);
      }
    } else {
      fault_1 = false;
    }
    if (status2 & 0x02) {
      if (!fault_2) {
        fault_2 = true;
        Serial.println("FAULT: Motor 2 deviation warning (stall/slip)");
        driver2.XTARGET(enc_microsteps_2);
      }
    } else {
      fault_2 = false;
    }
  }

  // 3. Send telemetry back to Jetson at ~20 Hz (50 ms interval)
  if (millis() - last_telemetry_time >= 50) {
    last_telemetry_time = millis();
    sendTelemetry();
  }
}
