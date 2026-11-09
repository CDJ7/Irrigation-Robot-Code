// Host-side tests of the REAL firmware (../Delta_Robot_ESP32/Delta_Robot_ESP32.ino), compiled with a fake Arduino core.
// Build + run:  sh firmware/tests/run_tests.sh      (needs only g++)
#include <Arduino.h>

uint64_t g_us = 0;
std::string g_out;
std::deque<char> g_in;
int g_pin[40];
std::vector<std::pair<int, uint64_t>> g_steps;
std::function<int(int)> g_read_hook;
std::function<void(int, int)> g_write_hook;
SerialClass Serial;
void digitalWrite(int pin, int level) {
  if ((pin == 25 || pin == 26 || pin == 27) && level == HIGH && g_pin[pin] == LOW) g_steps.push_back({pin, g_us});
  g_pin[pin] = level;
  if (g_write_hook) g_write_hook(pin, level);
}
int digitalRead(int pin) { return g_read_hook ? g_read_hook(pin) : HIGH; }

#include "../Delta_Robot_ESP32/Delta_Robot_ESP32.ino"

static int failures = 0, checks = 0;
#define CHECK(cond, msg) do { checks++; if (!(cond)) { failures++; printf("  FAIL line %d: %s\n", __LINE__, msg); } } while (0)

static void reset_world() {
  g_us = 1000000; g_out.clear(); g_in.clear(); g_steps.clear(); g_read_hook = nullptr; g_write_hook = nullptr;
  memset(g_pin, 0, sizeof g_pin);
  for (int i = 0; i < NUM_AXES; i++) { currentSteps[i] = 0; }
  moving = false; pendingCount = 0; serialBuf = ""; vacuumOn = false; driversEnabled = false;
  setup();
}
static void send(const char *cmd) { for (const char *c = cmd; *c; c++) g_in.push_back(*c); g_in.push_back('\n'); }
static std::string take() { std::string s = g_out; g_out.clear(); return s; }
static bool has(const std::string &s, const char *needle) { return s.find(needle) != std::string::npos; }
// run loop() until pred() or `limit_us` of virtual time has passed; each pass costs 3 us like a real loop would
static bool run_until(const std::function<bool()> &pred, uint64_t limit_us) {
  uint64_t end = g_us + limit_us;
  while (g_us < end) { loop(); g_us += 3; if (pred()) return true; }
  return false;
}
static void run_for(uint64_t us) { run_until([] { return false; }, us); }
static long pulses(int pin) { long n = 0; for (auto &p : g_steps) n += (p.first == pin); return n; }

static void test_boot_and_compat() {
  printf("1. boot, old commands unchanged\n");
  reset_world();
  CHECK(g_pin[VACUUM_PIN] == LOW, "vacuum pin LOW (off) after boot");
  CHECK(g_pin[5] == HIGH && g_pin[18] == HIGH && g_pin[19] == HIGH, "drivers disabled at boot (ENA high)");
  CHECK(has(take(), "READY DeltaRobotSlave"), "READY printed");
  send("P?"); run_for(100); CHECK(take() == "POS 0 0 0\n", "P? -> POS 0 0 0");
  send("M 0 0 0 100"); run_for(100); CHECK(take() == "OK\n", "zero-length move answers OK at once");
  send("M 1 2"); run_for(100); CHECK(take() == "ERR BAD_M_ARGS\n", "bad M args");
  send("FOO"); run_for(100); CHECK(take() == "ERR UNKNOWN_CMD FOO\n", "unknown command");
  send("EN 1"); run_for(100); CHECK(take() == "OK\n" && g_pin[5] == LOW && driversEnabled, "EN 1 -> OK, ENA low");
  send("Z"); run_for(100); CHECK(take() == "HOMED\n", "Z -> HOMED");
}

static void test_vacuum() {
  printf("2. vacuum on GPIO 17, active high\n");
  reset_world(); take();
  send("V 1"); run_for(100); CHECK(take() == "OK\n" && g_pin[17] == HIGH && vacuumOn, "V 1 -> OK, pin HIGH");
  send("V?"); run_for(100); CHECK(take() == "VAC 1\n", "V? says VAC 1");
  send("V 0"); run_for(100); CHECK(take() == "OK\n" && g_pin[17] == LOW, "V 0 -> OK, pin LOW");
  send("V 1"); run_for(100); take();
  send("STOP"); run_for(100); CHECK(has(take(), "POS") && g_pin[17] == LOW && !vacuumOn, "STOP switches the vacuum off");
  send("V 1"); run_for(100); take();
  send("EN 1"); run_for(100); take();
  send("M 20000 0 0 2000"); run_for(20000);
  send("V 1"); run_for(100); CHECK(has(take(), "ERR BUSY"), "V 1 refused while moving");
  send("V 0"); run_for(100); CHECK(has(take(), "OK") && g_pin[17] == LOW, "V 0 still honoured while moving");
  send("STOP"); run_for(100); take();
  send("V 1"); run_for(100); take();
  g_us += 31000000ULL; run_for(100);
  CHECK(has(take(), "WARN VACUUM_TIMEOUT") && g_pin[17] == LOW, "vacuum switches itself off after 30 s");
}

static void test_profile() {
  printf("3. trapezoidal speed profile\n");
  reset_world(); take(); send("EN 1"); run_for(100); take();
  g_steps.clear();
  uint64_t t0 = g_us;
  send("M 10000 5000 -2500 1000");
  bool done = run_until([] { return !moving; }, 10000000);
  uint64_t dur = g_us - t0;
  CHECK(done, "move finishes");
  CHECK(has(take(), "OK"), "reply OK");
  CHECK(currentSteps[0] == 10000 && currentSteps[1] == 5000 && currentSteps[2] == -2500, "exact final steps");
  CHECK(pulses(25) == 10000 && pulses(26) == 5000 && pulses(27) == 2500, "exact pulse counts per axis");
  // expected: ramp (10000-300)/20000 s each side + 5000 steps cruise
  double v0 = 300, vc = 10000, a = 20000;
  double expect = 2 * (vc - v0) / a + (10000 - (vc * vc - v0 * v0) / a) / vc;
  printf("   duration %.3f s (analytic %.3f s, requested cruise time 1.000 s)\n", dur / 1e6, expect);
  CHECK(fabs(dur / 1e6 - expect) < 0.08 * expect, "total time matches the analytic trapezoid within 8 %");
  // interval profile of the master axis (pin 25)
  std::vector<double> iv; uint64_t last = 0;
  for (auto &p : g_steps) if (p.first == 25) { if (last) iv.push_back((double)(p.second - last)); last = p.second; }
  auto avg_rate = [&](size_t from, size_t n) { double t = 0; for (size_t i = from; i < from + n; i++) t += iv[i]; return 1e6 * (double)n / t; };
  double first_rate = 1e6 / iv[0], end_rate = 1e6 / iv[iv.size() - 1], mid_rate = avg_rate(iv.size() / 2 - 500, 1000), peak = 0;
  for (size_t i = 3; i < iv.size(); i++) peak = fmax(peak, avg_rate(i - 3, 3));
  printf("   rate: first step %.0f, middle %.0f, last step %.0f, peak %.0f steps/s\n", first_rate, mid_rate, end_rate, peak);
  CHECK(first_rate < 400 && end_rate < 800, "starts and ends slowly");
  CHECK(mid_rate > 9500 && peak < 10500, "cruises at the requested rate, never above it");
  bool rising = true, falling = true;                        // 500-step windows: faster and faster at the start, slower and slower at the end
  for (size_t w = 0; w < 4; w++) {
    if (!(avg_rate((w + 1) * 500, 500) > avg_rate(w * 500, 500))) rising = false;
    if (!(avg_rate(iv.size() - (w + 2) * 500, 500) > avg_rate(iv.size() - (w + 1) * 500 - 1, 500))) falling = false;
  }
  CHECK(rising, "speed rises steadily at the start");
  CHECK(falling, "speed falls steadily at the end");

  printf("   short moves (triangle profile)\n");
  auto peak_of = [&](int pin) {                              // fastest 10-step average (single intervals jitter with the 3 us loop timing)
    std::vector<uint64_t> t; for (auto &p : g_steps) if (p.first == pin) t.push_back(p.second);
    double pk = 0; for (size_t i = 10; i < t.size(); i++) pk = fmax(pk, 1e7 / (double)(t[i] - t[i - 10])); return pk; };
  currentSteps[0] = 0; g_steps.clear(); send("M 4000 0 0 100"); run_until([] { return !moving; }, 5000000); take();
  peak = peak_of(25);
  printf("   4000 steps asked in 100 ms (= 40000/s, above MAX_STEP_RATE %.0f): peak %.0f steps/s (braking distance limits it to about %.0f)\n", MAX_STEP_RATE, peak, sqrt(ACCEL_STEPS_S2 * 4000));
  CHECK(peak < MAX_STEP_RATE * 1.05 && peak < 9800 && peak > 7000, "limited by the braking distance, never above MAX_STEP_RATE");
  CHECK(currentSteps[0] == 4000 && pulses(25) == 4000, "short move lands exactly");
  currentSteps[0] = 0; g_steps.clear(); send("M 100 0 0 10"); run_until([] { return !moving; }, 5000000); take();
  CHECK(peak_of(25) < 1700 && currentSteps[0] == 100 && pulses(25) == 100, "100-step move never reaches cruise (triangle) and lands exactly");
}

static void test_stop_busy_wrap() {
  printf("4. STOP, BUSY rules, micros() wrap\n");
  reset_world(); take(); send("EN 1"); run_for(100); take();
  send("M 50000 0 0 5000"); run_for(200000);
  send("EN 0"); run_for(500); CHECK(has(take(), "ERR BUSY") && driversEnabled, "EN refused while moving");
  send("Z"); run_for(500); CHECK(has(take(), "ERR BUSY"), "Z refused while moving");
  send("M 1 1 1 10"); run_for(500); CHECK(has(take(), "ERR BUSY"), "second M refused while moving");
  send("P?"); run_for(500); CHECK(has(take(), "POS"), "P? answered while moving");
  send("STOP"); run_for(500);
  std::string o = take();
  CHECK(has(o, "ERR STOPPED") && has(o, "POS") && !moving, "STOP aborts and reports");
  long at_stop = currentSteps[0]; size_t n = g_steps.size();
  run_for(500000);
  CHECK(currentSteps[0] == at_stop && g_steps.size() == n, "no pulses after STOP");
  CHECK(at_stop > 0 && at_stop < 50000, "it was stopped part-way");
  send("EN 0"); run_for(100); CHECK(has(take(), "OK") && !driversEnabled, "EN 0 accepted once stopped");
  send("EN 1"); run_for(100); take();
  // wrap: start 0.1 s before the 32-bit microsecond counter wraps
  currentSteps[0] = 0; g_steps.clear(); uint64_t r0 = g_us;
  send("M 4000 0 0 1000"); run_until([] { return !moving; }, 5000000); take();
  uint64_t ref = g_us - r0;                                   // the same move far from the wrap, for comparison
  currentSteps[0] = 0; g_us = 0xFFFFFFFFULL - 100000ULL; g_steps.clear(); uint64_t w0 = g_us;
  send("M 4000 0 0 1000"); bool done = run_until([] { return !moving; }, 5000000);
  uint64_t wrapped = g_us - w0;
  printf("   same move: %.3f s normally, %.3f s across the wrap\n", ref / 1e6, wrapped / 1e6);
  CHECK(done && currentSteps[0] == 4000 && pulses(25) == 4000, "a move that crosses the micros() wrap still completes exactly");
  CHECK(g_us > 0xFFFFFFFFULL, "the clock really did wrap during the move");
  CHECK(wrapped > ref * 97 / 100 && wrapped < ref * 103 / 100, "and takes the same time (no stall, no burst of steps)");
  take();
  // garbage line
  std::string junk(120, 'A'); send(junk.c_str()); run_for(2000);
  CHECK(has(take(), "ERR LINE_TOO_LONG"), "over-long line discarded");
}

static void test_homing() {
  printf("5. homing: debounce, queueing, abort, refusal\n");
  const long switch_at[3] = {1500, 2500, 1000};
  reset_world(); take();
  send("H"); run_for(1000); CHECK(has(take(), "ERR NOT_ENABLED") && g_steps.empty(), "homing refused with drivers off, nothing moved");
  send("EN 1"); run_for(100); take();

  // switches close after N steps on each axis; axis 1 also gets a 1-sample noise glitch earlier
  g_steps.clear();
  bool glitched = false;
  g_read_hook = [&](int pin) -> int {
    for (int i = 0; i < 3; i++) if (pin == LIMIT_PIN[i]) {
      if (pulses(STEP_PIN[i]) >= switch_at[i]) return LOW;
      if (i == 1 && pulses(STEP_PIN[1]) == 400 && !glitched) { glitched = true; return LOW; }   // one noisy reading
    }
    return HIGH;
  };
  send("H"); send("P?"); send("EN 0");       // two commands arrive WHILE homing
  bool done = run_until([] { return has(g_out, "HOMED"); }, 600000000ULL);
  CHECK(done, "homing completes");
  CHECK(glitched, "the noise glitch happened");
  long p0 = pulses(25), p1 = pulses(26), p2 = pulses(27);
  printf("   pulses per axis (switch at %ld/%ld/%ld + %ld back-off): %ld %ld %ld\n", switch_at[0], switch_at[1], switch_at[2], BACKOFF_STEPS, p0, p1, p2);
  CHECK(p0 >= switch_at[0] + BACKOFF_STEPS && p0 <= switch_at[0] + BACKOFF_STEPS + LIMIT_DEBOUNCE_SAMPLES, "axis 0 stopped at its switch (+ back-off)");
  CHECK(p1 >= switch_at[1] + BACKOFF_STEPS && p1 <= switch_at[1] + BACKOFF_STEPS + LIMIT_DEBOUNCE_SAMPLES, "axis 1 ignored the 1-sample glitch");
  CHECK(p2 >= switch_at[2] + BACKOFF_STEPS && p2 <= switch_at[2] + BACKOFF_STEPS + LIMIT_DEBOUNCE_SAMPLES, "axis 2 stopped at its switch (+ back-off)");
  run_for(100000);
  std::string o = take();
  CHECK(has(o, "HOMED\nPOS 0 0 0\n"), "HOMED then POS 0 0 0");
  CHECK(has(o, "POS 0 0 0\nPOS 0 0 0\n"), "the P? sent during homing was answered AFTER homing (queued, not lost)");
  CHECK(!driversEnabled, "the EN 0 sent during homing was executed AFTER homing (queued, not lost)");

  // STOP during homing
  reset_world(); take(); send("EN 1"); run_for(100); take(); send("V 1"); run_for(100); take();
  g_steps.clear(); g_read_hook = [](int) { return HIGH; };
  g_write_hook = [](int pin, int level) { if (pin == 25 && level == HIGH && pulses(25) == 300) send("STOP"); };
  send("H"); run_until([] { return has(g_out, "HOMING_ABORTED"); }, 100000000ULL);
  CHECK(has(take(), "ERR HOMING_ABORTED") && g_pin[17] == LOW, "STOP aborts homing and switches the vacuum off");
  g_write_hook = nullptr; size_t n = g_steps.size(); run_for(300000);
  CHECK(g_steps.size() == n, "no pulses after the abort");

  // axis 0 already on its switch: it must not be driven further into it
  reset_world(); take(); send("EN 1"); run_for(100); take(); g_steps.clear();
  g_read_hook = [&](int pin) -> int {
    if (pin == LIMIT_PIN[0]) return pulses(25) > BACKOFF_STEPS ? HIGH : LOW;
    for (int i = 1; i < 3; i++) if (pin == LIMIT_PIN[i]) return pulses(STEP_PIN[i]) >= 800 ? LOW : HIGH;
    return HIGH;
  };
  send("H"); run_until([] { return has(g_out, "HOMED"); }, 600000000ULL);
  CHECK(pulses(25) == BACKOFF_STEPS, "axis 0, already on its switch, made only the back-off steps");
  take();

  // broken switch: timeout error instead of homing forever
  reset_world(); take(); send("EN 1"); run_for(100); take(); g_read_hook = [](int) { return HIGH; };
  send("H"); run_until([] { return has(g_out, "HOMING_TIMEOUT"); }, 4000000000ULL);
  CHECK(has(take(), "ERR HOMING_TIMEOUT_AXIS_0"), "unwired switch -> HOMING_TIMEOUT");
}

int main() {
  test_boot_and_compat();
  test_vacuum();
  test_profile();
  test_stop_busy_wrap();
  test_homing();
  printf("\n%d checks, %d failed\n", checks, failures);
  if (failures == 0) printf("ALL TESTS PASSED\n");
  return failures ? 1 : 0;
}
