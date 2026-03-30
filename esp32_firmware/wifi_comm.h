#pragma once
/*
 * wifi_comm.h — UDP WiFi communication layer (Module 5, ESP32 side)
 * ==================================================================
 * Manages the bidirectional UDP link between the PC transmitter and the ESP32.
 *
 * ── Protocol overview ──────────────────────────────────────────────────────
 *
 * PC → ESP32  (port WIFI_CMD_PORT = 4210):
 *   Text commands, one per packet, terminated by newline (optional).
 *
 *   DRIVE <left_pct> <right_pct>
 *       Set wheel motor speeds, -100 … +100 percent.
 *
 *   MODE LQR  |  MODE PID
 *       Switch the active pendulum stabiliser.
 *
 *   PID <kpo> <kio> <kdo> <kpi> <kii> <kdi>
 *       Update cascade PID gains (outer then inner loop).
 *       Triggers a reset of the PID integrators.
 *
 *   LQR <K0> <K1>
 *       Update LQR gain vector directly (e.g. from lqr_design.py output or
 *       the LQR tuner GUI).
 *
 *   TORQUE <tau_Nm>
 *       Direct torque override for system identification step-response tests.
 *       Bypasses both LQR and PID.
 *
 *   STOP
 *       Immediately stop all motors and hold.
 *
 *   PING
 *       Heartbeat check — ESP32 prints "PONG" on Serial (no UDP reply needed).
 *
 * ESP32 → PC  (port WIFI_TEL_PORT = 4211, every 20 ms):
 *   JSON string:
 *   {"t":<ms>,"th":<deg>,"om":<dps>,"pwm":<pct>,"mode":<str>,"dt_us":<us>}
 *   where:
 *     t      — ESP32 millis()
 *     th     — pendulum angle (degrees)
 *     om     — angular velocity (degrees/s)
 *     pwm    — reaction motor PWM percent
 *     mode   — "LQR" | "PID" | "STOP" | "TORQUE"
 *     dt_us  — control-loop execution time in microseconds (latency indicator)
 *
 * ── Usage ──────────────────────────────────────────────────────────────────
 *   WifiComm wifi;
 *   wifi.begin("ssid", "password");   // call in setup()
 *
 *   // In loop():
 *   WifiCommand cmd;
 *   if (wifi.poll(cmd)) { ... handle cmd ... }
 *   wifi.sendTelemetry(millis(), theta, omega, pwm, "LQR", dt_us);
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

static constexpr int WIFI_CMD_PORT = 4210;   // PC → ESP32 (commands)
static constexpr int WIFI_TEL_PORT = 4211;   // ESP32 → PC  (telemetry)

/* Decoded command from one UDP packet. */
struct WifiCommand {
    enum Type {
        NONE,         // no packet / parse failure
        DRIVE,        // wheel motor speed command
        MODE_LQR,     // switch to LQR stabiliser
        MODE_PID,     // switch to PID stabiliser
        SET_PID,      // update PID gains
        SET_LQR,      // update LQR gain vector
        SET_TORQUE,   // direct torque override (for system ID)
        STOP,         // safe-stop all motors
        PING          // heartbeat
    } type = NONE;

    float drive_left  = 0.0f;   // DRIVE: left wheel speed  (-100..100 %)
    float drive_right = 0.0f;   // DRIVE: right wheel speed (-100..100 %)

    float pid_gains[6] = {};    // SET_PID: [kpo,kio,kdo,kpi,kii,kdi]
    float lqr_K[2]     = {};    // SET_LQR: [K0, K1]
    float torque_Nm    = 0.0f;  // SET_TORQUE: commanded torque (N·m)
};


class WifiComm {
public:
    /*
     * Connect to the given WiFi network and open the UDP command socket.
     * Retries for up to 10 seconds.  Returns true on success.
     */
    bool begin(const char* ssid, const char* password);

    /* True while WiFi is connected. */
    bool isConnected() const;

    /*
     * Non-blocking receive.  If a UDP command packet is available, parses it
     * into cmd and returns true.  Also records the sender's IP for telemetry.
     * Call every iteration of loop().
     */
    bool poll(WifiCommand& cmd);

    /*
     * Transmit a JSON telemetry packet to the last sender's IP on WIFI_TEL_PORT.
     * Does nothing if no command has been received yet (sender IP unknown).
     */
    void sendTelemetry(unsigned long t_ms, float theta_deg, float omega_dps,
                        float pwm_pct, const char* mode, unsigned long dt_us);

    /* Override the destination IP (e.g. hardcode the PC's IP). */
    void setRemoteIP(IPAddress ip);

private:
    WiFiUdp    _udp;
    IPAddress  _remote_ip;
    bool       _has_remote = false;
    char       _buf[256];

    /* Parse raw packet text into a WifiCommand.  Returns true if recognised. */
    bool _parse(const char* msg, int len, WifiCommand& cmd);
};
