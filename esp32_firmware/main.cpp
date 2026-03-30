/*
 * main.cpp — ESP32 Firmware (Modules 4 + 5)
 * ==========================================
 * Inverted Pendulum Race Car — embedded control system.
 *
 * Architecture
 * ------------
 *  • A hardware timer fires a 1 kHz callback (control_loop_cb) that:
 *      1. Reads the BNO055 IMU (angle θ and angular velocity ω).
 *      2. Runs the active stabiliser (LQR or PID) to compute reaction torque.
 *      3. Commands the reaction motor via PWM.
 *      4. Enforces the safety watchdog.
 *
 *  • The Arduino loop() function (main thread) handles:
 *      – Parsing incoming UDP drive/mode/gain commands.
 *      – Sending JSON telemetry every 20 ms.
 *      – Printing diagnostics to Serial.
 *
 * Control modes (switchable at runtime via WiFi "MODE LQR" / "MODE PID"):
 *   LQR   — default; optimal for small angles; gains from lqr_design.py
 *   PID   — cascade angle→ω→torque; fallback during hardware bring-up
 *   TORQUE — direct torque override for system identification tests
 *   STOP  — all motors off
 *
 * Safety watchdog
 * ---------------
 * The reaction motor is immediately stopped if:
 *   • BNO055 stops responding (isConnected() == false), OR
 *   • |θ| exceeds SAFE_ANGLE_DEG (default 45°)
 * The PID integrator is also reset to prevent wind-up on recovery.
 *
 * First-time setup checklist
 * --------------------------
 *  1. Set WIFI_SSID and WIFI_PASSWORD below.
 *  2. Run `python simulation/lqr_design.py` and paste K values into
 *     lqr_controller.h (or send them at runtime via "LQR <K0> <K1>").
 *  3. Check pin assignments in motor_control.h and bno055_driver.h.
 *  4. Flash with:  pio run --target upload
 *  5. Open Serial monitor at 115200 baud and verify "BNO055 OK" and "WiFi connected".
 *  6. Run transmitter/transmitter.py on your PC and verify telemetry arrives.
 */

#include <Arduino.h>
#include "bno055_driver.h"
#include "motor_control.h"
#include "lqr_controller.h"
#include "pid_controller.h"
#include "wifi_comm.h"

// ─── WiFi credentials ─────────────────────────────────────────────────────────
// Change these to your network before flashing.
#define WIFI_SSID     "YOUR_SSID"
#define WIFI_PASSWORD "YOUR_PASSWORD"

// ─── Safety limits ────────────────────────────────────────────────────────────
// Pendulum tilt beyond this angle → safe-stop reaction motor until recovery.
static constexpr float SAFE_ANGLE_DEG = 45.0f;

// ─── Telemetry rate ───────────────────────────────────────────────────────────
static constexpr unsigned long TELEMETRY_PERIOD_MS = 20;   // 50 Hz

// ─── Control mode ─────────────────────────────────────────────────────────────
// Changed from loop() when a MODE command is received over WiFi.
enum class CtrlMode : uint8_t { LQR, PID, TORQUE_OVERRIDE, STOP };
volatile CtrlMode g_mode = CtrlMode::LQR;

// ─── Shared state written by control_loop_cb, read by loop() ─────────────────
// Declared volatile so the compiler does not cache across ISR/main boundary.
volatile float         g_theta_deg  = 0.0f;   // pendulum angle (degrees)
volatile float         g_omega_dps  = 0.0f;   // angular velocity (degrees/s)
volatile float         g_pwm_pct    = 0.0f;   // reaction motor PWM (%)
volatile unsigned long g_loop_dt_us = 0;       // last control-loop duration (µs)
volatile bool          g_imu_ok     = false;   // false → watchdog triggered

// Direct torque override value (used in TORQUE_OVERRIDE mode for system ID)
volatile float g_torque_override_Nm = 0.0f;

// ─── Module instances ─────────────────────────────────────────────────────────
BNO055Driver         imu;
MotorControl         motors;
LQRController        lqr;
CascadePIDController pid_ctrl;
WifiComm             wifi;

// ─── Hardware timer handle ────────────────────────────────────────────────────
static esp_timer_handle_t g_ctrl_timer = nullptr;


// ─── 1 kHz control loop (runs in a high-priority FreeRTOS task) ───────────────
/*
 * This function is called every 1 ms by esp_timer.  Keep it short:
 *   • No heap allocation, no Serial prints, no WiFi calls.
 *   • Shared variables are read/written atomically (float on ESP32 is 4 bytes
 *     and naturally aligned, so reads/writes are atomic on the Xtensa core).
 */
static void IRAM_ATTR control_loop_cb(void* /*arg*/) {
    unsigned long t0 = micros();

    // ── 1. Read IMU ──────────────────────────────────────────────────────────
    imu.update();
    float theta_deg  = imu.getAngleDeg();
    float omega_dps  = imu.getOmegaDegps();
    bool  imu_ok     = imu.isConnected();

    g_imu_ok    = imu_ok;
    g_theta_deg = theta_deg;
    g_omega_dps = omega_dps;

    // ── 2. Safety watchdog ───────────────────────────────────────────────────
    bool safe = imu_ok && (fabsf(theta_deg) < SAFE_ANGLE_DEG);

    if (!safe) {
        motors.setReactionMotor(0.0f);
        pid_ctrl.reset();   // clear integrator so recovery starts cleanly
        g_pwm_pct    = 0.0f;
        g_loop_dt_us = micros() - t0;
        return;
    }

    // ── 3. Compute reaction motor torque ─────────────────────────────────────
    float theta_rad  = theta_deg  * (float)DEG_TO_RAD;
    float omega_rads = omega_dps  * (float)DEG_TO_RAD;
    float torque_Nm  = 0.0f;

    switch (g_mode) {
        case CtrlMode::LQR:
            torque_Nm = lqr.compute(theta_rad, omega_rads);
            break;

        case CtrlMode::PID:
            torque_Nm = pid_ctrl.compute(theta_rad, omega_rads, 0.001f);
            break;

        case CtrlMode::TORQUE_OVERRIDE:
            // Used during system identification step tests.
            // Torque is clamped to ±MAX_TORQUE for safety.
            torque_Nm = g_torque_override_Nm;
            if      (torque_Nm >  lqr.getMaxTorque()) torque_Nm =  lqr.getMaxTorque();
            else if (torque_Nm < -lqr.getMaxTorque()) torque_Nm = -lqr.getMaxTorque();
            break;

        case CtrlMode::STOP:
        default:
            torque_Nm = 0.0f;
            break;
    }

    // ── 4. Send torque to reaction motor ─────────────────────────────────────
    // Normalise N·m → [-1, +1] using the known motor torque constant.
    float normalized = torque_Nm / lqr.getMaxTorque();
    motors.setReactionMotor(normalized);

    g_pwm_pct    = motors.getReactionPWM();
    g_loop_dt_us = micros() - t0;
}


// ─── Setup ───────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(200);   // allow USB-Serial to enumerate
    Serial.println("\n=== Inverted Pendulum Race Car — ESP32 Firmware ===");

    // ── IMU ──────────────────────────────────────────────────────────────────
    Serial.print("Initialising BNO055... ");
    if (!imu.begin()) {
        Serial.println("FAILED!  Check SDA/SCL wiring and I2C address.");
        // Halt here — cannot run safely without an IMU
        while (true) {
            Serial.println("BNO055 not found.  Waiting...");
            delay(2000);
        }
    }
    Serial.println("OK");

    // ── Motors ───────────────────────────────────────────────────────────────
    Serial.print("Initialising motors... ");
    motors.begin();
    motors.stopAll();
    Serial.println("OK");

    // ── WiFi ─────────────────────────────────────────────────────────────────
    Serial.print("Connecting WiFi... ");
    if (!wifi.begin(WIFI_SSID, WIFI_PASSWORD)) {
        Serial.println("WARNING: No WiFi — continuing in standalone mode.");
        Serial.println("         Drive commands and telemetry will not work.");
    }

    // ── 1 kHz control timer ──────────────────────────────────────────────────
    const esp_timer_create_args_t timer_args = {
        .callback        = control_loop_cb,
        .arg             = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name            = "ctrl_1khz",
        .skip_unhandled_events = false,
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &g_ctrl_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(g_ctrl_timer, 1000));   // 1000 µs = 1 kHz
    Serial.println("Control loop started at 1 kHz.");

    Serial.println("=== Setup complete — waiting for commands ===");
}


// ─── Main loop ────────────────────────────────────────────────────────────────

void loop() {
    static unsigned long last_telemetry_ms = 0;

    // ── Parse incoming WiFi commands ─────────────────────────────────────────
    WifiCommand cmd;
    if (wifi.poll(cmd)) {
        switch (cmd.type) {

            case WifiCommand::DRIVE:
                // Wheel speed command — only sets wheel motors; the control
                // loop handles the reaction motor independently.
                motors.setWheelSpeed(cmd.drive_left, cmd.drive_right);
                break;

            case WifiCommand::MODE_LQR:
                g_mode = CtrlMode::LQR;
                pid_ctrl.reset();
                Serial.println("Switched to LQR");
                break;

            case WifiCommand::MODE_PID:
                g_mode = CtrlMode::PID;
                pid_ctrl.reset();
                Serial.println("Switched to PID");
                break;

            case WifiCommand::SET_PID:
                // Update gains and reset integrators for bumpless transfer
                pid_ctrl.setOuterGains(cmd.pid_gains[0], cmd.pid_gains[1], cmd.pid_gains[2]);
                pid_ctrl.setInnerGains(cmd.pid_gains[3], cmd.pid_gains[4], cmd.pid_gains[5]);
                pid_ctrl.reset();
                Serial.printf("PID gains updated: outer=[%.3f,%.3f,%.3f] inner=[%.3f,%.3f,%.3f]\n",
                              cmd.pid_gains[0], cmd.pid_gains[1], cmd.pid_gains[2],
                              cmd.pid_gains[3], cmd.pid_gains[4], cmd.pid_gains[5]);
                break;

            case WifiCommand::SET_LQR:
                lqr.setGains(cmd.lqr_K[0], cmd.lqr_K[1]);
                Serial.printf("LQR gains updated: K=[%.4f, %.4f]\n",
                              cmd.lqr_K[0], cmd.lqr_K[1]);
                break;

            case WifiCommand::SET_TORQUE:
                // Switch to direct torque override for system ID tests
                g_torque_override_Nm = cmd.torque_Nm;
                g_mode               = CtrlMode::TORQUE_OVERRIDE;
                Serial.printf("Torque override: %.3f N·m\n", cmd.torque_Nm);
                break;

            case WifiCommand::STOP:
                g_mode = CtrlMode::STOP;
                motors.stopAll();
                pid_ctrl.reset();
                Serial.println("STOP");
                break;

            case WifiCommand::PING:
                // Response already printed by WifiComm::_parse()
                break;

            default:
                break;
        }
    }

    // ── Send telemetry at 50 Hz ───────────────────────────────────────────────
    unsigned long now = millis();
    if (now - last_telemetry_ms >= TELEMETRY_PERIOD_MS) {
        last_telemetry_ms = now;

        // Select mode label for JSON
        const char* mode_str;
        switch (g_mode) {
            case CtrlMode::LQR:             mode_str = "LQR";    break;
            case CtrlMode::PID:             mode_str = "PID";    break;
            case CtrlMode::TORQUE_OVERRIDE: mode_str = "TORQUE"; break;
            default:                        mode_str = "STOP";   break;
        }

        wifi.sendTelemetry(now,
                           g_theta_deg,
                           g_omega_dps,
                           g_pwm_pct,
                           mode_str,
                           g_loop_dt_us);
    }

    // Small delay to yield time to the WiFi stack (FreeRTOS task)
    delay(1);
}
