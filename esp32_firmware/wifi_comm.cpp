/*
 * wifi_comm.cpp — UDP WiFi communication layer implementation
 * ============================================================
 * See wifi_comm.h for the full protocol specification and usage notes.
 */

#include "wifi_comm.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

bool WifiComm::begin(const char* ssid, const char* password) {
    Serial.printf("Connecting to WiFi SSID: %s ", ssid);

    WiFi.begin(ssid, password);

    // Wait up to 10 s for connection (20 attempts × 500 ms)
    for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi connection failed.");
        return false;
    }

    Serial.printf("Connected. IP: %s\n", WiFi.localIP().toString().c_str());

    // Open UDP socket for receiving commands
    _udp.begin(WIFI_CMD_PORT);
    _has_remote = false;

    return true;
}

bool WifiComm::isConnected() const {
    return WiFi.status() == WL_CONNECTED;
}

bool WifiComm::poll(WifiCommand& cmd) {
    cmd.type = WifiCommand::NONE;

    int len = _udp.parsePacket();
    if (len <= 0) return false;

    // Record sender IP — used to direct telemetry back to the same PC
    if (!_has_remote) {
        _remote_ip = _udp.remoteIP();
        _has_remote = true;
        Serial.printf("Remote PC: %s\n", _remote_ip.toString().c_str());
    }

    // Read payload into buffer, null-terminate
    int n = _udp.read(_buf, (int)sizeof(_buf) - 1);
    if (n <= 0) return false;
    _buf[n] = '\0';

    return _parse(_buf, n, cmd);
}

bool WifiComm::_parse(const char* msg, int /*len*/, WifiCommand& cmd) {
    // ── DRIVE <left_pct> <right_pct> ────────────────────────────────────────
    if (strncmp(msg, "DRIVE", 5) == 0) {
        cmd.type = WifiCommand::DRIVE;
        sscanf(msg + 6, "%f %f", &cmd.drive_left, &cmd.drive_right);
        return true;
    }

    // ── MODE LQR / MODE PID ──────────────────────────────────────────────────
    if (strncmp(msg, "MODE LQR", 8) == 0) {
        cmd.type = WifiCommand::MODE_LQR;
        return true;
    }
    if (strncmp(msg, "MODE PID", 8) == 0) {
        cmd.type = WifiCommand::MODE_PID;
        return true;
    }

    // ── PID <kpo> <kio> <kdo> <kpi> <kii> <kdi> ────────────────────────────
    if (strncmp(msg, "PID", 3) == 0 && msg[3] == ' ') {
        cmd.type = WifiCommand::SET_PID;
        sscanf(msg + 4, "%f %f %f %f %f %f",
               &cmd.pid_gains[0], &cmd.pid_gains[1], &cmd.pid_gains[2],
               &cmd.pid_gains[3], &cmd.pid_gains[4], &cmd.pid_gains[5]);
        return true;
    }

    // ── LQR <K0> <K1> ────────────────────────────────────────────────────────
    if (strncmp(msg, "LQR", 3) == 0 && msg[3] == ' ') {
        cmd.type = WifiCommand::SET_LQR;
        sscanf(msg + 4, "%f %f", &cmd.lqr_K[0], &cmd.lqr_K[1]);
        return true;
    }

    // ── TORQUE <tau_Nm> (system ID step test) ───────────────────────────────
    if (strncmp(msg, "TORQUE", 6) == 0) {
        cmd.type = WifiCommand::SET_TORQUE;
        sscanf(msg + 7, "%f", &cmd.torque_Nm);
        return true;
    }

    // ── STOP ─────────────────────────────────────────────────────────────────
    if (strncmp(msg, "STOP", 4) == 0) {
        cmd.type = WifiCommand::STOP;
        return true;
    }

    // ── PING (heartbeat) ─────────────────────────────────────────────────────
    if (strncmp(msg, "PING", 4) == 0) {
        cmd.type = WifiCommand::PING;
        Serial.println("PONG");
        return true;
    }

    return false;   // unrecognised command
}

void WifiComm::sendTelemetry(unsigned long t_ms, float theta_deg, float omega_dps,
                              float pwm_pct, const char* mode, unsigned long dt_us) {
    if (!isConnected() || !_has_remote) return;

    // Build compact JSON — fits in a single UDP packet (< 512 bytes)
    char json[256];
    snprintf(json, sizeof(json),
             "{\"t\":%lu,\"th\":%.2f,\"om\":%.2f,\"pwm\":%.1f,\"mode\":\"%s\",\"dt_us\":%lu}",
             t_ms, theta_deg, omega_dps, pwm_pct, mode, dt_us);

    _udp.beginPacket(_remote_ip, WIFI_TEL_PORT);
    _udp.write(reinterpret_cast<const uint8_t*>(json), strlen(json));
    _udp.endPacket();
}

void WifiComm::setRemoteIP(IPAddress ip) {
    _remote_ip  = ip;
    _has_remote = true;
}
