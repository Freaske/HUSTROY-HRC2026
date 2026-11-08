#pragma once
// Host-only API double. This does NOT emulate ESP32 timing/electrical behavior.
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <deque>
#include <string>

constexpr int LOW = 0;
constexpr int HIGH = 1;
constexpr int OUTPUT = 1;

struct FakeSerial {
    std::deque<unsigned char> input;
    std::string output;
    void begin(uint32_t) {}
    int available() { return static_cast<int>(input.size()); }
    int read() { const auto c = input.front(); input.pop_front(); return c; }
    void println(const char* text) { output += text; output += '\n'; }
    template <typename... Args> void printf(const char* format, Args... args) {
        char buffer[512];
        std::snprintf(buffer, sizeof(buffer), format, args...);
        output += buffer;
    }
};

extern FakeSerial Serial;
unsigned long millis();
void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
bool ledcAttach(uint8_t pin, uint32_t frequency, uint8_t bits);
bool ledcWrite(uint8_t pin, uint32_t duty);
