// A tiny stand-in for the Arduino core, so the REAL firmware file can be compiled and tested on a PC.
// It provides a virtual clock (micros/millis/delayMicroseconds advance it), pins, and a fake Serial.
#pragma once
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <string>
#include <vector>

#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT_PULLUP 2

struct String {
  std::string s;
  String() {}
  String(const char *c) : s(c) {}
  String &operator+=(char c) { s += c; return *this; }
  unsigned length() const { return (unsigned)s.size(); }
  void trim() {
    size_t a = s.find_first_not_of(" \t\r\n"), b = s.find_last_not_of(" \t\r\n");
    s = (a == std::string::npos) ? "" : s.substr(a, b - a + 1);
  }
  bool startsWith(const char *p) const { return s.rfind(p, 0) == 0; }
  int indexOf(char c) const { size_t i = s.find(c); return i == std::string::npos ? -1 : (int)i; }
  const char *c_str() const { return s.c_str(); }
  bool operator==(const char *o) const { return s == o; }
};

// ---- test-bench state (the tests read and write these) ----
extern uint64_t g_us;                                  // virtual time in microseconds
extern std::string g_out;                              // everything the firmware printed
extern std::deque<char> g_in;                          // bytes waiting on the serial port
extern int g_pin[40];                                  // last level written to each pin
extern std::vector<std::pair<int, uint64_t>> g_steps;  // (pin, time) of each STEP rising edge
extern std::function<int(int)> g_read_hook;            // digitalRead(pin) -> level
extern std::function<void(int, int)> g_write_hook;     // called on every digitalWrite

inline uint32_t micros() { return (uint32_t)g_us; }
inline unsigned long millis() { return (unsigned long)(g_us / 1000ULL); }
inline void delayMicroseconds(unsigned n) { g_us += n; }
inline void pinMode(int, int) {}
void digitalWrite(int pin, int level);
int digitalRead(int pin);

struct SerialClass {
  void begin(long) {}
  int available() { return (int)g_in.size(); }
  int read() { int c = (unsigned char)g_in.front(); g_in.pop_front(); return c; }
  void print(const char *c) { g_out += c; }
  void print(char c) { g_out += c; }
  void print(int v) { g_out += std::to_string(v); }
  void print(long v) { g_out += std::to_string(v); }
  void print(unsigned long v) { g_out += std::to_string(v); }
  void print(const String &v) { g_out += v.s; }
  template <class T> void println(T v) { print(v); g_out += '\n'; }
};
extern SerialClass Serial;
