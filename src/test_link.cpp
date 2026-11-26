#include "test_link.h"

#if defined(HUSTROY_TEST_BLUETOOTH) || defined(HUSTROY_TEST_WIFI)
#include <Arduino.h>
#include <cstring>
#include "modes.h"
#include "tuning.h"

#ifdef HUSTROY_TEST_BLUETOOTH
#include <BluetoothSerial.h>
#else
#include <WiFi.h>
#include <esp_http_server.h>
#include "test_page.h"
#endif

namespace {
struct LogLine { char text[192]; };
constexpr std::size_t COMMAND_SIZE = 64;
constexpr unsigned long LOG_INTERVAL_MS = 67; // ~15 Hz, no catch-up bursts.
QueueHandle_t latestLog;
unsigned long lastLogAt = 0;
bool ready = false;
bool radioStartedRun = false;

void execute(const char* command, unsigned long now, LogLine& reply) {
    const auto& status = modesStatus();
    if (tuningCommand(command, status.running, reply.text, sizeof(reply.text))) return;

    const char* message = "ERR unknown command; use HELP";
    if (std::strcmp(command, "HELP") == 0) {
        message = "GET | KP=1.2 | KD=6 | SPD=120 | SAVE | MODE SENSOR_TEST | MODE LINE_FOLLOW | ARM | GO | STOP | DISARM";
    } else if (std::strcmp(command, "MODE SENSOR_TEST") == 0) {
        modesSelect(RobotMode::SENSOR_TEST, now);
        message = "OK MODE SENSOR_TEST";
    } else if (std::strcmp(command, "MODE LINE_FOLLOW") == 0) {
        modesSelect(RobotMode::LINE_FOLLOW, now);
        message = "OK MODE LINE_FOLLOW; ARM required";
    } else if (std::strcmp(command, "ARM") == 0) {
        message = status.mode == RobotMode::LINE_FOLLOW && modesArm() ? "OK ARMED" : "ERR ARM requires LINE_FOLLOW and motor ready";
    } else if (std::strcmp(command, "GO") == 0) {
        if (modesRun(now)) {
            radioStartedRun = true;
            message = "OK GO";
        } else {
            message = status.running ? "ERR already running" : "ERR GO blocked; check ARM, calibration/bias and stable line";
        }
    } else if (std::strcmp(command, "STOP") == 0) {
        modesStop("STOP");
        message = "OK STOP";
    } else if (std::strcmp(command, "DISARM") == 0) {
        modesDisarm("DISARM");
        message = "OK DISARM";
    }
    std::snprintf(reply.text, sizeof(reply.text), "%s", message);
}

void publishLog(unsigned long now) {
    if (now - lastLogAt < LOG_INTERVAL_MS) return;
    lastLogAt = now;
    const auto& status = modesStatus();
    const auto& bits = status.sensors.active;
    LogLine line{};
    std::snprintf(line.text, sizeof(line.text),
                  "t=%lu MODE=%s RUN=%d ARM=%d LINE=%s BITS=%d%d%d%d%d e=%.2f PWM_L=%d PWM_R=%d STOP=%s",
                  now, modeName(status.mode), status.running, status.armed, lineStateName(status.sensors.state),
                  bits[0], bits[1], bits[2], bits[3], bits[4], status.error,
                  status.leftCommand, status.rightCommand, status.running ? "-" : status.stopReason);
    xQueueOverwrite(latestLog, &line);
}

#ifdef HUSTROY_TEST_BLUETOOTH
BluetoothSerial bluetooth;
QueueHandle_t replies;
char input[COMMAND_SIZE];
std::size_t length = 0;
bool discardLine = false;

void bluetoothWriter(void*) {
    // BluetoothSerial::write can wait on a congested SPP queue. Keep it off loop().
    for (;;) {
        LogLine line;
        if ((xQueueReceive(replies, &line, pdMS_TO_TICKS(10)) == pdPASS ||
             xQueueReceive(latestLog, &line, 0) == pdPASS) && bluetooth.hasClient()) {
            const std::size_t size = std::strlen(line.text);
            line.text[size] = '\n';
            bluetooth.write(reinterpret_cast<const uint8_t*>(line.text), size + 1);
        }
    }
}

bool startRadio() {
    replies = xQueueCreate(6, sizeof(LogLine));
    if (!replies || !bluetooth.begin("HUSTROY-HRC2026")) return false;
    bluetooth.setTimeout(0);
    if (xTaskCreate(bluetoothWriter, "hustroy-bt-tx", 3072, nullptr, 1, nullptr) != pdPASS) {
        bluetooth.end();
        return false;
    }
    Serial.println("Test Bluetooth SPP: HUSTROY-HRC2026");
    return true;
}

bool radioConnected() { return bluetooth.hasClient(); }

void pollRadio() {
    if (!radioConnected()) {
        // Don't let bytes left in the SPP receive queue start a new session's line.
        for (std::size_t i = 0; i < COMMAND_SIZE && bluetooth.available() > 0; ++i) bluetooth.read();
        length = 0;
        discardLine = false;
        return;
    }
    for (std::size_t i = 0; i < COMMAND_SIZE && bluetooth.available() > 0; ++i) {
        const int byte = bluetooth.read();
        if (byte < 0) break;
        LogLine reply{};
        if (byte == '!') {
            modesDisarm("EMERGENCY_COMMAND");
            length = 0;
            discardLine = true;
            std::snprintf(reply.text, sizeof(reply.text), "OK DISARM; send Enter");
        } else if (byte == '\n' || byte == '\r') {
            if (!discardLine && length > 0) {
                input[length] = '\0';
                execute(input, millis(), reply);
            }
            length = 0;
            discardLine = false;
        } else if (!discardLine) {
            if (length == sizeof(input) - 1 || byte < 32 || byte > 126) {
                discardLine = true;
                std::snprintf(reply.text, sizeof(reply.text), "ERR invalid command line");
            } else input[length++] = static_cast<char>(byte);
        }
        if (reply.text[0]) xQueueSend(replies, &reply, 0);
    }
}
#else
struct WebCommand { uint32_t id; unsigned long queuedAt; char text[COMMAND_SIZE]; };
struct WebReply { uint32_t id; LogLine line; };
QueueHandle_t commands;
QueueHandle_t replies;
constexpr unsigned long WEB_COMMAND_MAX_AGE_MS = 500;

esp_err_t servePage(httpd_req_t* request) {
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    return httpd_resp_send(request, TEST_PAGE, HTTPD_RESP_USE_STRLEN);
}

esp_err_t serveLog(httpd_req_t* request) {
    LogLine line{};
    xQueuePeek(latestLog, &line, 0);
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_sendstr(request, line.text);
}

esp_err_t serveCommand(httpd_req_t* request) {
    char header[2];
    if (httpd_req_get_hdr_value_str(request, "X-Hustroy-Control", header, sizeof(header)) != ESP_OK || header[0] != '1') {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "ERR missing control header");
    }
    static uint32_t nextId = 0; // Only the HTTP server task uses this counter.
    WebCommand command{};
    const auto size = request->content_len;
    if (size == 0 || size >= sizeof(command.text)) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "ERR command length");
    }
    for (std::size_t received = 0; received < size;) {
        const int count = httpd_req_recv(request, command.text + received, size - received);
        if (count <= 0) return ESP_FAIL;
        received += count;
    }
    for (std::size_t i = 0; i < size; ++i) {
        if (command.text[i] < 32 || command.text[i] > 126) {
            return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "ERR invalid command line");
        }
    }
    command.id = ++nextId;
    command.queuedAt = millis();
    if (xQueueSend(commands, &command, 0) == pdPASS) {
        WebReply reply;
        while (millis() - command.queuedAt < WEB_COMMAND_MAX_AGE_MS) {
            if (xQueueReceive(replies, &reply, pdMS_TO_TICKS(20)) == pdPASS && reply.id == command.id) {
                return httpd_resp_sendstr(request, reply.line.text);
            }
        }
    }
    httpd_resp_set_status(request, "503 Service Unavailable");
    return httpd_resp_sendstr(request, "ERR no reply; check log before retrying");
}

bool startRadio() {
    commands = xQueueCreate(1, sizeof(WebCommand));
    replies = xQueueCreate(1, sizeof(WebReply));
    if (!commands || !replies) return false;

    constexpr char password[] = "12345678";
    if (!WiFi.softAP("HUSTROY-HRC2026", password, 1, false, 1)) return false;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.recv_wait_timeout = config.send_wait_timeout = 1;
    config.lru_purge_enable = true;
    httpd_handle_t server = nullptr;
    if (httpd_start(&server, &config) != ESP_OK) {
        WiFi.softAPdisconnect(true);
        return false;
    }
    const httpd_uri_t routes[] = {
        {"/", HTTP_GET, servePage, nullptr},
        {"/log", HTTP_GET, serveLog, nullptr},
        {"/command", HTTP_POST, serveCommand, nullptr},
    };
    for (const auto& route : routes) {
        if (httpd_register_uri_handler(server, &route) != ESP_OK) {
            httpd_stop(server);
            WiFi.softAPdisconnect(true);
            return false;
        }
    }
    Serial.printf("Test Wi-Fi: HUSTROY-HRC2026 password=%s http://%s/\n", password, WiFi.softAPIP().toString().c_str());
    return true;
}

bool radioConnected() { return WiFi.softAPgetStationNum() > 0; }

void pollRadio() {
    WebCommand command;
    if (xQueueReceive(commands, &command, 0) != pdPASS) return;
    const unsigned long now = millis();
    WebReply reply{};
    reply.id = command.id;
    if (now - command.queuedAt >= WEB_COMMAND_MAX_AGE_MS || !radioConnected()) {
        std::snprintf(reply.line.text, sizeof(reply.line.text), "ERR command expired");
    } else execute(command.text, now, reply.line);
    xQueueOverwrite(replies, &reply);
}
#endif
}

void testLinkInit() {
    latestLog = xQueueCreate(1, sizeof(LogLine));
    ready = latestLog && startRadio();
    if (!ready) Serial.println("ERR test radio unavailable; use USB Serial");
}

void testLinkUpdate(unsigned long now) {
    if (!ready) return;
    if (radioStartedRun && now - lastLogAt >= LOG_INTERVAL_MS && !radioConnected()) {
        modesDisarm("TEST_LINK_LOST");
    }
    pollRadio();
    if (!modesStatus().running) radioStartedRun = false;
    publishLog(now);
}
#else
void testLinkInit() {}
void testLinkUpdate(unsigned long) {}
#endif
