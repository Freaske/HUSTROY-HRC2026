#include <Arduino.h>
#include <driver/gpio.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "config.h"
#include "modes.h"
#include "tuning.h"
#include "line_follow.h"

void setup();
void loop();

namespace {
constexpr double pi = 3.141592653589793;
constexpr double lineWidth = 18;
constexpr double wheelTrack = 88;
constexpr double sensorAhead = 48;
constexpr double sensorSpacing = 12;
constexpr double speedPerPwm = 5.6; // mm/s per PWM unit for this model.
unsigned long clockMs = 0;
std::array<bool, SENSOR_COUNT> sensorInputs{};

struct Point { double x, y; };
Point operator+(Point a, Point b) { return {a.x + b.x, a.y + b.y}; }
Point operator-(Point a, Point b) { return {a.x - b.x, a.y - b.y}; }
Point operator*(Point p, double scale) { return {p.x * scale, p.y * scale}; }
double dot(Point a, Point b) { return a.x * b.x + a.y * b.y; }
Point forward(double angle) { return {std::cos(angle), std::sin(angle)}; }

struct Pose { Point center; double angle = 0; };
struct Rect { double x, y, width, height; };
struct Checkpoint { Point center; double angle; };
using Path = std::vector<Point>;
struct Track {
    Pose start;
    std::vector<Path> paths;
    std::vector<Rect> darkZones;
    std::vector<Checkpoint> checkpoints;
};

Track makeTrack(const std::string& id) {
    if (id == "curve") {
        Path path{{95, 230}, {540, 230}};
        for (int i = 1; i <= 30; ++i) {
            const double angle = -pi / 2 + i * pi / 60;
            path.push_back(Point{540, 330} + forward(angle) * 100);
        }
        path.push_back({640, 557});
        return {{{155, 230}}, {path}, {}, {{{640, 510}, pi / 2}}};
    }
    if (id == "corner") {
        const Point vertex{790, 205};
        const double exitAngle = 2 * pi / 3;
        return {{{155, 205}}, {{{95, 205}, vertex, vertex + forward(exitAngle) * 370}},
                {}, {{vertex + forward(exitAngle) * 305, exitAngle}}};
    }
    if (id == "cross") {
        return {{{155, 310}}, {{{95, 310}, {1220, 310}}, {{715, 100}, {715, 510}}}, {}, {}};
    }
    if (id == "gap") {
        return {{{155, 310}}, {{{95, 310}, {655, 310}}, {{755, 310}, {1220, 310}}}, {}, {}};
    }
    if (id == "finish") {
        return {{{155, 310}}, {{{95, 310}, {820, 310}}}, {{820, 160, 400, 300}}, {}};
    }
    return {{{155, 310}}, {{{95, 310}, {1230, 310}}}, {}, {{{1155, 310}, 0}}};
}

double distanceSquared(Point point, Point start, Point end) {
    const Point segment = end - start;
    const double fraction = std::clamp(dot(point - start, segment) / dot(segment, segment), 0.0, 1.0);
    const Point offset = point - (start + segment * fraction);
    return dot(offset, offset);
}

bool isDark(Point point, const Track& track) {
    for (const auto& zone : track.darkZones) {
        if (point.x >= zone.x && point.x <= zone.x + zone.width &&
            point.y >= zone.y && point.y <= zone.y + zone.height) return true;
    }
    for (const auto& path : track.paths) {
        for (std::size_t i = 1; i < path.size(); ++i) {
            if (distanceSquared(point, path[i - 1], path[i]) <= lineWidth * lineWidth / 4) return true;
        }
    }
    return false;
}

void sampleSensors(const Pose& pose, const Track& track) {
    const Point center = pose.center + forward(pose.angle) * sensorAhead;
    const Point right = forward(pose.angle + pi / 2);
    for (std::size_t i = 0; i < sensorInputs.size(); ++i) {
        sensorInputs[i] = isDark(center + right * (static_cast<int>(i) - 2) * sensorSpacing, track);
    }
}

void sendCommand(const std::string& command) {
    for (const char c : command + '\n') Serial.input.push_back(c);
    loop();
}

void advancePose(Pose& pose, const RobotStatus& status) {
    const double dt = (LINE_PERIOD_US / 1000) / 1000.0;
    const double left = status.leftCommand * speedPerPwm;
    const double right = status.rightCommand * speedPerPwm;
    const double turn = (left - right) * dt / wheelTrack;
    pose.center = pose.center + forward(pose.angle + turn / 2) * ((left + right) * dt / 2);
    pose.angle += turn;
}

bool crossedCheckpoint(Point previous, Point current, const Checkpoint& checkpoint) {
    const Point direction = forward(checkpoint.angle);
    const double before = dot(previous - checkpoint.center, direction);
    const double after = dot(current - checkpoint.center, direction);
    if (before >= 0 || after < 0) return false;
    const Point crossing = previous + (current - previous) * (-before / (after - before));
    return std::abs(dot(crossing - checkpoint.center, forward(checkpoint.angle + pi / 2))) <= 35;
}

void writePoint(Point point) { std::cout << '[' << point.x << ',' << point.y << ']'; }

void writeTrack(const Track& track) {
    std::cout << "{\"width\":1320,\"height\":620,\"lineWidth\":" << lineWidth
              << ",\"sensorAhead\":" << sensorAhead << ",\"sensorSpacing\":" << sensorSpacing
              << ",\"wheelTrack\":" << wheelTrack << ",\"start\":";
    writePoint(track.start.center);
    std::cout << ",\"paths\":[";
    for (std::size_t i = 0; i < track.paths.size(); ++i) {
        if (i) std::cout << ',';
        std::cout << '[';
        for (std::size_t j = 0; j < track.paths[i].size(); ++j) {
            if (j) std::cout << ',';
            writePoint(track.paths[i][j]);
        }
        std::cout << ']';
    }
    std::cout << "],\"zones\":[";
    for (std::size_t i = 0; i < track.darkZones.size(); ++i) {
        if (i) std::cout << ',';
        const auto& zone = track.darkZones[i];
        std::cout << '[' << zone.x << ',' << zone.y << ',' << zone.width << ',' << zone.height << ']';
    }
    std::cout << "],\"checkpoints\":[";
    for (std::size_t i = 0; i < track.checkpoints.size(); ++i) {
        if (i) std::cout << ',';
        const auto& checkpoint = track.checkpoints[i];
        std::cout << '[' << checkpoint.center.x << ',' << checkpoint.center.y << ',' << checkpoint.angle << ']';
    }
    std::cout << "]}";
}

void writeFrame(const Pose& pose, unsigned long ms, double distance) {
    const auto& status = modesStatus();
    std::cout << "{\"t\":" << ms << ",\"x\":" << pose.center.x << ",\"y\":" << pose.center.y
              << ",\"angle\":" << pose.angle << ",\"distance\":" << distance << ",\"bits\":\"";
    for (bool active : status.sensors.active) std::cout << active;
    std::cout << "\",\"line\":\"" << lineStateName(status.sensors.state) << "\",\"armed\":" << status.armed
              << ",\"running\":" << status.running << ",\"error\":" << status.error
              << ",\"left\":" << status.leftCommand << ",\"right\":" << status.rightCommand
              << ",\"stop\":\"" << (status.running ? "" : status.stopReason) << "\"}";
}
}

FakeSerial Serial;
unsigned long micros() { return clockMs * 1000; }
unsigned long millis() { return clockMs; }
void pinMode(int, int) {}
void digitalWrite(int, int) {}
int digitalRead(int pin) {
    const auto sensor = std::find(SENSOR_PINS.begin(), SENSOR_PINS.end(), pin);
    if (sensor == SENSOR_PINS.end()) return HIGH;
    return sensorInputs[sensor - SENSOR_PINS.begin()] != INVERT_LINE_SENSOR ? HIGH : LOW;
}
int analogRead(int) { return -1; }
void analogReadResolution(uint8_t) {}
void analogSetPinAttenuation(int, int) {}
bool ledcAttach(uint8_t, uint32_t, uint8_t) { return true; }
bool ledcWrite(uint8_t, uint32_t) { return true; }
int gpio_set_level(gpio_num_t, unsigned int) { return 0; }

int main(int argc, char** argv) {
    if (argc != 1 && argc != 5) return 2;
    std::cout << std::fixed;
    std::cout.precision(6);
    if (argc == 1) {
        std::cout << "{\"kp\":" << PID_KP << ",\"kd\":" << PID_KD << ",\"speed\":" << BASE_SPEED
                  << ",\"maxSpeed\":" << MAX_SPEED << ",\"stepMs\":" << (LINE_PERIOD_US / 1000)
                  << ",\"runMs\":" << LINE_FOLLOW_MAX_RUN_MS << "}\n";
        return 0;
    }

    const Track track = makeTrack(argv[1]);
    Pose pose = track.start;
    setup();
    sendCommand("MODE LINE_FOLLOW");
    sendCommand(std::string("KP=") + argv[2]);
    sendCommand(std::string("KD=") + argv[3]);
    sendCommand(std::string("SPD=") + argv[4]);
    sendCommand("ARM");
    sampleSensors(pose, track);
    for (unsigned int i = 0; i < LINE_START_VALID_SAMPLES; ++i) {
        clockMs += (LINE_PERIOD_US / 1000);
        loop();
    }
    sendCommand("GO");
    const unsigned long startedAt = clockMs;

    std::cout << "{\"track\":";
    writeTrack(track);
    double distance = 0;
    bool reached = false;
    std::cout << ",\"frames\":[";
    writeFrame(pose, 0, distance);
    while (modesStatus().running) {
        const Point previous = pose.center;
        advancePose(pose, modesStatus());
        const Point movement = pose.center - previous;
        distance += std::hypot(movement.x, movement.y);
        sampleSensors(pose, track);
        clockMs += (LINE_PERIOD_US / 1000);
        loop();
        if (modesStatus().running && !track.checkpoints.empty() &&
            crossedCheckpoint(previous, pose.center, track.checkpoints.back())) {
            reached = true;
            sendCommand("STOP");
        }
        std::cout << ',';
        writeFrame(pose, clockMs - startedAt, distance);
    }
    std::cout << "],\"checkpointReached\":" << reached << "}\n";
}
