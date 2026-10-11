/*
 * Lam Research Challenge 3.0 - Stage 2 (Hardware Hustle)
 * Flow skid controller firmware for the "LAM Stage 2 PCB" (T-Works, rev 1.0)
 * Target: NodeMCU-32S / ESP32-DevKitC, Arduino framework (PlatformIO esp32dev)
 *
 * What it does
 *   CASE 1  ON/OFF button -> SOV-01 opens, P-02 runs at 100 %, P-01 at 50 %.
 *           Float switch in the sump goes HIGH -> P-01 stops, P-02 keeps
 *           draining. Float clears (and stays clear) -> P-01 restarts.
 *           FT-01 is read continuously: flow in L/min and mL/s, totalised litres.
 *   CASE 2  After CASE1_DURATION_S the controller closes SOV-01, stops P-01 and
 *           gives the operator BV02_WINDOW_S seconds to open BV-02 by hand
 *           (board LED blinks fast, countdown on serial). Then P-03, P-01 and
 *           P-02 all run until the ON/OFF button is pressed (no automatic stop).
 *   Button: IDLE -> CASE 1 -> (press) -> CASE 2 -> (press) -> STOP.
 *   A press moves on at once, whatever time is left. Case 1 still moves to
 *   Case 2 by itself after CASE1_DURATION_S if nobody presses. Type "stop"
 *   in the serial monitor for an immediate stop from any state.
 *
 * Data acquisition
 *   Serial 115200 baud. Every LOG_MS a CSV line starting with "D," is printed:
 *     D,t_ms,state,pulses,f_Hz,Q_Lmin,Q_mLs,total_L,float_high,p1,p2,p3,sov
 *   Lines starting with "#" are human messages. Open the PlatformIO serial
 *   monitor, or run tools/flow_logger.py to save a CSV and plot the trends.
 *   Type "help" in the monitor for calibration / test commands.
 *
 * Calibration
 *   Q [L/min] = a * f [Hz] + b. Defaults are the YF-S401 datasheet value
 *   (f = 98 * Q  ->  a = 1/98, b = 0). The "trial" commands record pulses,
 *   time and the collected volume for each run, "fit" computes a, b, R^2 and
 *   errors by least squares, "fit save" stores them in flash.
 */

#include <Arduino.h>
#include <Preferences.h>

// ============================================================================
// PIN MAP  (read from the PCB schematic "LAM Stage 2 PCB", sheet 1/1)
// ============================================================================
// Motor driver headers U2/U3/U4 (XH 6-pin): 1 VCC(5V) 2 GND 3 L_EN 4 R_EN
// 5 L_PWM 6 R_PWM. They go to the IBT_2 (BTS7960) 8-pin header:
// 1 RPWM 2 LPWM 3 R_EN 4 L_EN 5 R_IS 6 L_IS 7 VCC 8 GND (IS pins unused).
// Pump 1 -> U2
static const uint8_t PIN_P1_RPWM = 33; // R_PWM1
static const uint8_t PIN_P1_LPWM = 32; // L_PWM1
static const uint8_t PIN_P1_REN = 26;  // R_EN1
static const uint8_t PIN_P1_LEN = 13;  // L_EN1
// Pump 2 -> U3
static const uint8_t PIN_P2_RPWM = 19; // R_PWM2 - trace missing on the PCB (organiser notice);
                                       // P-02 is driven from L_PWM2 instead, see P2_REVERSE
static const uint8_t PIN_P2_LPWM = 18; // L_PWM2
static const uint8_t PIN_P2_REN = 17;  // R_EN2
static const uint8_t PIN_P2_LEN = 16;  // L_EN2
// Pump 3 -> U4
static const uint8_t PIN_P3_RPWM = 25; // R_PWM3
static const uint8_t PIN_P3_LPWM = 23; // L_PWM3
static const uint8_t PIN_P3_REN = 22;  // R_EN3
static const uint8_t PIN_P3_LEN = 21;  // L_EN3

static const uint8_t PIN_SOLENOID = 27;  // SV  -> SOLENOID_VALVE pin 1 -> D4184 SIG
static const uint8_t PIN_FLOW = 14;      // FS  -> FLOW_SENSOR pin 1 (YF-S401 yellow)
static const uint8_t PIN_ONOFF_SW = 34;  // SW  -> ON/OFF_SWITCH, 10k pull-up to 3V3 on PCB
static const uint8_t PIN_FLOAT_SW = 35;  // FSW -> FLOAT_SWITCH, 10k pull-up to 3V3 on PCB
static const uint8_t PIN_STATUS_LED = 2; // blue LED on the NodeMCU-32S module

// ============================================================================
// SETTINGS
// ============================================================================
// Duty cycles (%)
static const uint8_t CASE1_DUTY_P1 = 50;  // problem statement: P-01 at 50 %
static const uint8_t CASE1_DUTY_P2 = 100; // problem statement: P-02 at 100 %
static const uint8_t FLUSH_DUTY_P1 = 100; // P-03 feeds P-01 in series; matched so
static const uint8_t FLUSH_DUTY_P2 = 100; // the tee is not over-pressurised
static const uint8_t FLUSH_DUTY_P3 = 100;

// Set true for any pump that turns the wrong way (driven from L_PWM instead).
// Diaphragm pumps usually pump the same direction either way, but a pump with
// an internal diode only runs with the right polarity.
static const bool P1_REVERSE = false;
static const bool P2_REVERSE = true; // must stay true: R_PWM2 has no trace, so PWM goes out on L_PWM2.
                                     // Swap P-02's two motor wires on the IBT_2 output if it runs backwards.
static const bool P3_REVERSE = false;

// Alternative P-02 wiring: jumper the IBT_2's RPWM pin (pin 1) to its VCC pin
// (pin 7, +5 V) instead of GND, then set this true. M+ is then held at +12 V
// and the PWM on L_PWM2 pulls M- low, so the code inverts P-02's duty
// (motor voltage = 12 V x duty). Pump red wire -> M+, green -> M-.
static const bool P2_RPWM_TIED_HIGH = true;

// P-02 speed through its ENABLE pins. Use this while the L_PWM2 signal does
// not reach the IBT_2 (P-02 ran at full speed at any duty, 2026-10-11).
// Wiring: IBT_2 RPWM (pin 1) jumpered to VCC (pin 7, 5 V), LPWM left as is.
// L_EN2 (GPIO16) is held HIGH while P-02 runs and R_EN2 (GPIO17) carries the
// PWM: R_EN high = M+ at 12 V, R_EN low = M+ off, so the pump sees 12 V x duty.
// The enable inputs switch slower than the PWM inputs, hence the lower frequency
// (audible whine is normal). Pump red wire -> M+, green -> M-.
static const bool P2_SPEED_ON_ENABLE = true;
static const uint32_t EN_PWM_FREQ_HZ = 1000;

// Sequence timing (seconds). CASE1 and BV02 can be changed at runtime.
static uint32_t CASE1_DURATION_S = 60; // how long Case 1 runs before flushing
static uint32_t BV02_WINDOW_S = 5;     // time to open BV-02 before P-03 starts

// Float switch: level reads LOW when the float closes the contact to GND.
// If your float is mounted so the contact OPENS when the sump is full, set HIGH.
static const int FLOAT_HIGH_LEVEL = LOW;
static const uint32_t FLOAT_DEBOUNCE_MS = 300; // reject ripples on the surface
static const uint32_t FLOAT_CLEAR_MS = 3000;   // level must stay low this long before P-01 restarts

// ON/OFF tact switch pulls the input to GND when pressed.
static const uint32_t BUTTON_DEBOUNCE_MS = 50;

// FT-01 (YF-S401). Datasheet: F[Hz] = 98 * Q[L/min] (5880 pulses per litre).
// Q = CAL_A * f + CAL_B. Replace with your fitted values (or use "fit save").
static const float DEFAULT_CAL_A = 0.009213f; // fitted, 5 trials, 2026-10-11
static const float DEFAULT_CAL_B = -0.0672f;
static const uint32_t FLOW_GLITCH_US = 300; // 6 L/min is ~590 Hz (1.7 ms period)

// Motor PWM: BTS7960 accepts up to 25 kHz; 20 kHz is above hearing.
static const uint32_t PWM_FREQ_HZ = 20000;
static const uint8_t PWM_BITS = 10;
static const uint32_t SOFTSTART_MS = 600; // ramp time 0 -> 100 % (limits SMPS inrush)

// Logging / sampling
static uint32_t LOG_MS = 500;

// Optional flow-hold PI on P-02 (Task 1.2 / "maintain 3 L/min"). Tune on the rig.
static const float PI_KP = 8.0f; // % duty per L/min of error
static const float PI_KI = 6.0f; // % duty per (L/min * s)
static const float TARGET_FLOW_LPM = 3.0f;

// ============================================================================
// Pumps
// ============================================================================
struct Pump
{
  const char *name;
  uint8_t rpwm, lpwm, ren, len;
  bool reverse;
  uint8_t channel; // LEDC channel (core 2.x)
  float target;    // % requested
  float actual;    // % after soft-start ramp
};

static Pump pumps[3] = {
    {"P-01", PIN_P1_RPWM, PIN_P1_LPWM, PIN_P1_REN, PIN_P1_LEN, P1_REVERSE, 0, 0, 0},
    {"P-02", PIN_P2_RPWM, PIN_P2_LPWM, PIN_P2_REN, PIN_P2_LEN, P2_REVERSE, 2, 0, 0},
    {"P-03", PIN_P3_RPWM, PIN_P3_LPWM, PIN_P3_REN, PIN_P3_LEN, P3_REVERSE, 4, 0, 0},
};
enum
{
  P1 = 0,
  P2 = 1,
  P3 = 2
};

static inline uint8_t pwmPin(const Pump &p) { return p.reverse ? p.lpwm : p.rpwm; }
static inline uint8_t idlePin(const Pump &p) { return p.reverse ? p.rpwm : p.lpwm; }
static inline bool speedOnEnable(const Pump &p) { return P2_SPEED_ON_ENABLE && &p == &pumps[P2]; }
// Pin that carries the speed PWM: R_EN for P-02 in enable mode, else R_PWM/L_PWM.
static inline uint8_t ledcPin(const Pump &p) { return speedOnEnable(p) ? p.ren : pwmPin(p); }

static void pwmWrite(Pump &p, float pct)
{
  uint32_t maxDuty = (1u << PWM_BITS) - 1;
  pct = constrain(pct, 0.0f, 100.0f);
  if (P2_RPWM_TIED_HIGH && &p == &pumps[1] && !speedOnEnable(p))
    pct = 100.0f - pct; // L_PWM2 low is what drives the motor in this wiring
  uint32_t duty = (uint32_t)(pct * maxDuty / 100.0f + 0.5f);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(ledcPin(p), duty);
#else
  ledcWrite(p.channel, duty);
#endif
}

static void pumpInit(Pump &p)
{
  pinMode(p.ren, OUTPUT);
  pinMode(p.len, OUTPUT);
  digitalWrite(p.ren, LOW);
  digitalWrite(p.len, LOW);
  if (speedOnEnable(p))
  {
    // Both PWM inputs are plain LOW outputs; the speed PWM goes out on R_EN.
    pinMode(p.rpwm, OUTPUT);
    digitalWrite(p.rpwm, LOW);
    pinMode(p.lpwm, OUTPUT);
    digitalWrite(p.lpwm, LOW);
  }
  else
  {
    pinMode(idlePin(p), OUTPUT);
    digitalWrite(idlePin(p), LOW); // the other half-bridge stays on its low side
  }
  uint32_t freq = speedOnEnable(p) ? EN_PWM_FREQ_HZ : PWM_FREQ_HZ;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(ledcPin(p), freq, PWM_BITS);
#else
  ledcSetup(p.channel, freq, PWM_BITS);
  ledcAttachPin(ledcPin(p), p.channel);
#endif
  pwmWrite(p, 0);
}

static void pumpSet(int i, float pct) { pumps[i].target = constrain(pct, 0.0f, 100.0f); }

static void allPumpsOffNow()
{
  for (auto &p : pumps)
  {
    p.target = p.actual = 0;
    pwmWrite(p, 0);
    if (!speedOnEnable(p))
      digitalWrite(p.ren, LOW); // in enable mode R_EN is the PWM pin, set by pwmWrite
    digitalWrite(p.len, LOW);
  }
}

// Called every loop: ramps actual duty toward target and drives the enables.
static void pumpsUpdate()
{
  static uint32_t last = millis();
  uint32_t now = millis();
  float step = 100.0f * (now - last) / SOFTSTART_MS;
  last = now;
  for (auto &p : pumps)
  {
    if (p.target <= 0)
    {
      p.actual = 0; // stopping is immediate
    }
    else if (p.actual < p.target)
    {
      p.actual = min(p.target, p.actual + step);
    }
    else
    {
      p.actual = p.target;
    }
    bool en = p.actual > 0;
    if (!speedOnEnable(p))
      digitalWrite(p.ren, en);
    digitalWrite(p.len, en);
    pwmWrite(p, p.actual);
  }
}

// ============================================================================
// Solenoid valve SOV-01 (12 V DC, via D4184 MOSFET module, HIGH = energised)
// ============================================================================
static bool sovOpen = false;
static void setSolenoid(bool open)
{
  sovOpen = open;
  digitalWrite(PIN_SOLENOID, open ? HIGH : LOW);
}

// ============================================================================
// FT-01 flow transmitter
// ============================================================================
static volatile uint32_t isrPulses = 0;
static volatile uint32_t isrLastUs = 0;
static volatile uint32_t isrRawEdges = 0; // every falling edge, before the glitch filter

// "fsraw 1": diagnostic readout of the FT-01 pin itself
static bool fsRaw = false;
static uint32_t fsHighSamples = 0, fsLowSamples = 0, fsLastRaw = 0;

static void IRAM_ATTR onFlowPulse()
{
  uint32_t now = micros();
  isrRawEdges++;
  if (now - isrLastUs >= FLOW_GLITCH_US)
  {
    isrPulses++;
    isrLastUs = now;
  }
}

static float calA = DEFAULT_CAL_A, calB = DEFAULT_CAL_B;
static uint32_t pulsesTotal = 0; // since start of the run / last "reset"
static float freqHz = 0, flowLpm = 0, totalL = 0;
static uint32_t lastSampleMs = 0, lastSamplePulses = 0;

static uint32_t readPulses()
{
  noInterrupts();
  uint32_t n = isrPulses;
  interrupts();
  return n;
}

static float flowFromFreq(float f)
{
  if (f <= 0.0f)
    return 0.0f; // no pulses means no flow, whatever the offset b
  return max(0.0f, calA * f + calB);
}

// Computes f, Q and the totaliser over the interval since the previous call.
static void sampleFlow()
{
  uint32_t now = millis();
  uint32_t n = readPulses();
  uint32_t dn = n - lastSamplePulses;
  float dt = (now - lastSampleMs) / 1000.0f;
  if (dt <= 0)
    return;
  lastSamplePulses = n;
  lastSampleMs = now;
  pulsesTotal += dn;
  freqHz = dn / dt;
  flowLpm = flowFromFreq(freqHz);
  totalL += flowLpm * dt / 60.0f;
}

// ============================================================================
// Inputs: ON/OFF button and float switch (both have 10k pull-ups on the PCB;
// GPIO34/35 are input-only and have no internal pull-ups)
// ============================================================================
static bool buttonPressedEvent()
{
  static int stable = HIGH, lastRead = HIGH;
  static uint32_t changedAt = 0;
  int r = digitalRead(PIN_ONOFF_SW);
  if (r != lastRead)
  {
    lastRead = r;
    changedAt = millis();
  }
  if (millis() - changedAt >= BUTTON_DEBOUNCE_MS && r != stable)
  {
    stable = r;
    if (stable == LOW)
      return true; // falling edge = press
  }
  return false;
}

static bool floatHigh = false; // debounced "sump level high"
static void updateFloat()
{
  static int lastRead = -1;
  static uint32_t changedAt = 0;
  int r = digitalRead(PIN_FLOAT_SW);
  if (r != lastRead)
  {
    lastRead = r;
    changedAt = millis();
  }
  if (millis() - changedAt >= FLOAT_DEBOUNCE_MS)
    floatHigh = (r == FLOAT_HIGH_LEVEL);
}

// ============================================================================
// Sequence state machine
// ============================================================================
enum State
{
  IDLE,
  CASE1,
  FLUSH_PREP,
  FLUSH,
  DONE,
  TEST
};
static const char *STATE_NAMES[] = {"IDLE", "CASE1", "FLUSH_PREP", "FLUSH", "DONE", "TEST"};
static State state = IDLE;
static uint32_t stateSince = 0;

static bool p1HeldByFloat = false; // P-01 (and P-03 in flush) stopped by sump high
static uint32_t floatClearSince = 0;
static bool holdFlowCase1 = false; // "hold 1": PI keeps P-02 at TARGET_FLOW_LPM in Case 1

// Flow PI on P-02 (test mode "flow <L/min>", or Case 1 with "hold 1")
static bool piActive = false;
static float piSetpoint = TARGET_FLOW_LPM, piInteg = 0;

static uint32_t secondsInState() { return (millis() - stateSince) / 1000; }

static void enterState(State s)
{
  state = s;
  stateSince = millis();
  Serial.printf("# STATE -> %s\n", STATE_NAMES[s]);
  switch (s)
  {
  case IDLE:
  case DONE:
    allPumpsOffNow();
    setSolenoid(false);
    piActive = false;
    if (s == DONE)
      Serial.printf("# Sequence complete. Total transferred: %.3f L (%lu pulses). "
                    "Press ON/OFF to run again.\n",
                    totalL, (unsigned long)pulsesTotal);
    break;
  case CASE1:
    // Totaliser starts from zero for each run.
    totalL = 0;
    pulsesTotal = 0;
    setSolenoid(true);
    delay(300); // let SOV-01 open before the pumps pull on it
    p1HeldByFloat = false;
    piInteg = 0;
    piSetpoint = TARGET_FLOW_LPM;
    pumpSet(P2, CASE1_DUTY_P2);
    pumpSet(P1, CASE1_DUTY_P1);
    pumpSet(P3, 0);
    Serial.printf("# CASE 1: SOV-01 open, P-01 %u%%, P-02 %u%%, runs %lus\n",
                  CASE1_DUTY_P1, CASE1_DUTY_P2, (unsigned long)CASE1_DURATION_S);
    break;
  case FLUSH_PREP:
    setSolenoid(false); // isolate the 10 L process tank
    pumpSet(P1, 0);     // nothing to suck until P-03 runs
    pumpSet(P3, 0);
    pumpSet(P2, 100); // keep emptying the sump through FT-01
    Serial.printf("# CASE 2: SOV-01 closed. OPEN BALL VALVE BV-02 NOW. "
                  "P-03 starts in %lus (type 'next' to start now)\n",
                  (unsigned long)BV02_WINDOW_S);
    break;
  case FLUSH:
    p1HeldByFloat = false;
    pumpSet(P3, FLUSH_DUTY_P3);
    pumpSet(P1, FLUSH_DUTY_P1);
    pumpSet(P2, FLUSH_DUTY_P2);
    Serial.println("# CASE 2: flushing, all three pumps ON. Press ON/OFF to stop.");
    break;
  case TEST:
    allPumpsOffNow();
    Serial.println("# TEST mode: use p1/p2/p3 <0-100>, sov 0|1, flow <L/min>|off. 'stop' to exit.");
    break;
  }
}

// Sump-high protection: stops P-01 (and P-03 while flushing, since it only
// feeds P-01) and restarts them once the level has stayed low long enough.
static void handleFloat(uint8_t dutyP1, uint8_t dutyP3)
{
  if (floatHigh)
  {
    floatClearSince = 0;
    if (!p1HeldByFloat)
    {
      p1HeldByFloat = true;
      pumpSet(P1, 0);
      if (dutyP3)
        pumpSet(P3, 0);
      Serial.println("# SUMP HIGH: P-01 stopped, P-02 draining");
    }
  }
  else if (p1HeldByFloat)
  {
    if (floatClearSince == 0)
      floatClearSince = millis();
    if (millis() - floatClearSince >= FLOAT_CLEAR_MS)
    {
      p1HeldByFloat = false;
      pumpSet(P1, dutyP1);
      if (dutyP3)
        pumpSet(P3, dutyP3);
      Serial.println("# Sump level normal: P-01 restarted");
    }
  }
}

static void runFlowPI(float dt)
{
  float err = piSetpoint - flowLpm;
  float ff = piSetpoint / 6.0f * 100.0f; // rough feed-forward: 6 L/min ~ 100 %
  float out = ff + PI_KP * err + PI_KI * (piInteg + err * dt);
  if (out > 0 && out < 100)
    piInteg += err * dt; // anti-windup: integrate only when not saturated
  pumpSet(P2, constrain(out, 0.0f, 100.0f));
}

static void runStateMachine()
{
  switch (state)
  {
  case CASE1:
    handleFloat(CASE1_DUTY_P1, 0);
    if (secondsInState() >= CASE1_DURATION_S)
      enterState(FLUSH_PREP);
    break;
  case FLUSH_PREP:
  {
    static uint32_t lastCount = 0;
    uint32_t left = BV02_WINDOW_S > secondsInState() ? BV02_WINDOW_S - secondsInState() : 0;
    if (left != lastCount)
    {
      lastCount = left;
      Serial.printf("# P-03 starts in %lus - open BV-02\n", (unsigned long)left);
    }
    if (left == 0)
      enterState(FLUSH);
    break;
  }
  case FLUSH:
    handleFloat(FLUSH_DUTY_P1, FLUSH_DUTY_P3); // runs until the button is pressed
    break;
  default:
    break;
  }
}

static void statusLed()
{
  uint32_t t = millis();
  bool on;
  switch (state)
  {
  case IDLE:
    on = (t % 2000) < 100;
    break; // short blink: waiting
  case CASE1:
    on = true;
    break;
  case FLUSH_PREP:
    on = (t % 200) < 100;
    break; // fast blink: open BV-02
  case FLUSH:
    on = (t % 1000) < 500;
    break;
  case DONE:
    on = (t % 3000) < 1500;
    break;
  default:
    on = (t % 500) < 250;
    break;
  }
  digitalWrite(PIN_STATUS_LED, on);
}

// ============================================================================
// Calibration trials (Q2.2 / Q2.3)
// ============================================================================
struct Trial
{
  uint32_t pulses;
  float seconds;
  float volumeL;
};
static const int MAX_TRIALS = 12;
static Trial trials[MAX_TRIALS];
static int nTrials = 0;
static bool trialRunning = false;
static uint32_t trialStartPulses = 0, trialStartMs = 0;
static Trial pending = {0, 0, -1}; // stopped, waiting for its volume

static void printTrials()
{
  Serial.println("# Trial, Pulses, Time(s), Frequency(Hz), Volume(L), ActualFlow(L/min)");
  for (int i = 0; i < nTrials; i++)
  {
    const Trial &t = trials[i];
    Serial.printf("# %d, %lu, %.2f, %.2f, %.3f, %.3f\n", i + 1, (unsigned long)t.pulses,
                  t.seconds, t.pulses / t.seconds, t.volumeL, t.volumeL / t.seconds * 60.0f);
  }
}

// Least-squares fit of Q = a f + b over the recorded trials.
static bool fitTrials(float &a, float &b)
{
  if (nTrials < 2)
  {
    Serial.println("# Need at least 2 trials (5 recommended)");
    return false;
  }
  double sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (int i = 0; i < nTrials; i++)
  {
    double f = trials[i].pulses / trials[i].seconds;
    double q = trials[i].volumeL / trials[i].seconds * 60.0;
    sx += f;
    sy += q;
    sxx += f * f;
    sxy += f * q;
  }
  double n = nTrials, den = n * sxx - sx * sx;
  if (fabs(den) < 1e-9)
  {
    Serial.println("# Trials all at the same frequency, cannot fit");
    return false;
  }
  a = (n * sxy - sx * sy) / den;
  b = (sy - a * sx) / n;
  double mean = sy / n, ssRes = 0, ssTot = 0, maxErr = 0, sumErr = 0;
  for (int i = 0; i < nTrials; i++)
  {
    double f = trials[i].pulses / trials[i].seconds;
    double q = trials[i].volumeL / trials[i].seconds * 60.0;
    double e = q - (a * f + b);
    ssRes += e * e;
    ssTot += (q - mean) * (q - mean);
    double pct = fabs(e) / q * 100.0;
    maxErr = max(maxErr, pct);
    sumErr += pct;
  }
  double r2 = ssTot > 0 ? 1.0 - ssRes / ssTot : 1.0;
  Serial.printf("# FIT: a = %.6f L/min/Hz, b = %.4f L/min, R^2 = %.5f\n", a, b, r2);
  Serial.printf("# Max calibration error = %.2f %%, average = %.2f %%\n", maxErr, sumErr / n);
  Serial.printf("# (K = %.1f pulses/L, f = %.2f * Q)\n", 60.0 / a, 1.0 / a);
  return true;
}

// ============================================================================
// Serial commands
// ============================================================================
static Preferences prefs;
static float fitA = 0, fitB = 0;
static bool fitValid = false;

static void printHelp()
{
  Serial.println(
      "# Commands:\n"
      "#  start | stop | next        run sequence / stop all / skip to next phase\n"
      "#  status                     show state, flow, calibration\n"
      "#  c1time <s> | bvtime <s>    Case 1 duration / BV-02 window\n"
      "#  fsraw 1|0                  FT-01 diagnostic: pin level and raw edge count\n"
      "#  hold 0|1                   Case 1: P-02 at 100% (0) or PI to 3 L/min (1)\n"
      "#  log <ms>                   logging period\n"
      "#  reset                      zero the totaliser\n"
      "#  test                       manual mode (from IDLE):\n"
      "#    p1|p2|p3 <0-100>, sov 0|1, flow <L/min> | flow off\n"
      "#  cal                        show a, b\n"
      "#  seta <a> | setb <b>        set calibration (RAM), calsave to keep\n"
      "#  calsave | caldefault       store in flash / back to datasheet values\n"
      "#  trial start | trial stop | trial vol <L> | trial list | trial clear\n"
      "#  fit | fit save             least-squares Q = a f + b from trials");
}

static void handleCommand(String line)
{
  line.trim();
  if (!line.length())
    return;
  int sp = line.indexOf(' ');
  String cmd = sp < 0 ? line : line.substring(0, sp);
  String arg = sp < 0 ? "" : line.substring(sp + 1);
  arg.trim();
  cmd.toLowerCase();

  if (cmd == "help")
    printHelp();
  else if (cmd == "start")
  {
    if (state == IDLE || state == DONE)
      enterState(CASE1);
  }
  else if (cmd == "stop")
    enterState(IDLE);
  else if (cmd == "next")
  {
    if (state == CASE1)
      enterState(FLUSH_PREP);
    else if (state == FLUSH_PREP)
      enterState(FLUSH);
    else if (state == FLUSH)
      enterState(DONE);
  }
  else if (cmd == "status")
  {
    Serial.printf("# state=%s t=%lus flow=%.3f L/min total=%.3f L float=%s sov=%d "
                  "P1=%.0f P2=%.0f P3=%.0f a=%.6f b=%.4f c1time=%lu bvtime=%lu hold=%d\n",
                  STATE_NAMES[state], (unsigned long)secondsInState(), flowLpm, totalL,
                  floatHigh ? "HIGH" : "normal", sovOpen, pumps[0].actual, pumps[1].actual,
                  pumps[2].actual, calA, calB, (unsigned long)CASE1_DURATION_S,
                  (unsigned long)BV02_WINDOW_S, holdFlowCase1);
  }
  else if (cmd == "c1time")
    CASE1_DURATION_S = arg.toInt();
  else if (cmd == "bvtime")
    BV02_WINDOW_S = arg.toInt();
  else if (cmd == "fsraw")
  {
    fsRaw = arg.toInt() != 0;
    Serial.println(fsRaw ? "# FT-01 raw pin readout ON (one '# FS' line per log line)"
                         : "# FT-01 raw pin readout OFF");
  }
  else if (cmd == "hold")
  {
    holdFlowCase1 = arg.toInt() != 0;
    piInteg = 0;
    piSetpoint = TARGET_FLOW_LPM;
    if (!holdFlowCase1 && state == CASE1)
      pumpSet(P2, CASE1_DUTY_P2);
  }
  else if (cmd == "log")
    LOG_MS = max(100L, arg.toInt());
  else if (cmd == "reset")
  {
    totalL = 0;
    pulsesTotal = 0;
  }
  else if (cmd == "test")
  {
    if (state == IDLE || state == DONE)
      enterState(TEST);
  }
  else if (state == TEST && (cmd == "p1" || cmd == "p2" || cmd == "p3"))
  {
    int i = cmd[1] - '1';
    if (i == P2)
      piActive = false;
    pumpSet(i, arg.toFloat());
  }
  else if (state == TEST && cmd == "sov")
    setSolenoid(arg.toInt() != 0);
  else if (state == TEST && cmd == "flow")
  {
    if (arg == "off")
    {
      piActive = false;
      pumpSet(P2, 0);
    }
    else
    {
      piSetpoint = arg.toFloat();
      piInteg = 0;
      piActive = true;
    }
  }
  else if (cmd == "cal")
    Serial.printf("# Q = %.6f * f + %.4f\n", calA, calB);
  else if (cmd == "seta")
    calA = arg.toFloat();
  else if (cmd == "setb")
    calB = arg.toFloat();
  else if (cmd == "calsave")
  {
    prefs.putFloat("a", calA);
    prefs.putFloat("b", calB);
    Serial.println("# calibration saved");
  }
  else if (cmd == "caldefault")
  {
    calA = DEFAULT_CAL_A;
    calB = DEFAULT_CAL_B;
    prefs.clear();
    Serial.println("# calibration reset to datasheet values");
  }
  else if (cmd == "trial")
  {
    if (arg == "start")
    {
      trialStartPulses = readPulses();
      trialStartMs = millis();
      trialRunning = true;
      Serial.println("# trial started: put the return line into the measuring jug");
    }
    else if (arg == "stop" && trialRunning)
    {
      trialRunning = false;
      pending.pulses = readPulses() - trialStartPulses;
      pending.seconds = (millis() - trialStartMs) / 1000.0f;
      pending.volumeL = -1;
      Serial.printf("# trial stopped: N = %lu pulses, t = %.2f s, f = %.2f Hz. "
                    "Now enter: trial vol <litres collected>\n",
                    (unsigned long)pending.pulses, pending.seconds, pending.pulses / pending.seconds);
    }
    else if (arg.startsWith("vol") && pending.seconds > 0)
    {
      if (nTrials >= MAX_TRIALS)
      {
        Serial.println("# trial table full");
        return;
      }
      pending.volumeL = arg.substring(3).toFloat();
      trials[nTrials++] = pending;
      pending.seconds = 0;
      printTrials();
    }
    else if (arg == "list")
      printTrials();
    else if (arg == "clear")
    {
      nTrials = 0;
      Serial.println("# trials cleared");
    }
  }
  else if (cmd == "fit")
  {
    if (arg == "save")
    {
      if (!fitValid)
      {
        Serial.println("# run 'fit' first");
        return;
      }
      calA = fitA;
      calB = fitB;
      prefs.putFloat("a", calA);
      prefs.putFloat("b", calB);
      Serial.println("# fitted calibration applied and saved");
    }
    else
    {
      fitValid = fitTrials(fitA, fitB);
    }
  }
  else
  {
    Serial.printf("# unknown command '%s' (type help)\n", line.c_str());
  }
}

static void pollSerial()
{
  static String buf;
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n' || c == '\r')
    {
      handleCommand(buf);
      buf = "";
    }
    else if (buf.length() < 80)
    {
      buf += c;
    }
  }
}

// ============================================================================
// setup / loop
// ============================================================================
void setup()
{
  // Outputs to a safe state first: everything off, SOV-01 closed.
  pinMode(PIN_SOLENOID, OUTPUT);
  setSolenoid(false);
  for (auto &p : pumps)
    pumpInit(p);
  pinMode(PIN_STATUS_LED, OUTPUT);

  pinMode(PIN_ONOFF_SW, INPUT); // external 10k pull-up (R2)
  pinMode(PIN_FLOAT_SW, INPUT); // external 10k pull-up (R10)
  pinMode(PIN_FLOW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW), onFlowPulse, FALLING);

  Serial.begin(115200);
  delay(200);

  prefs.begin("ft01", false);
  calA = prefs.getFloat("a", DEFAULT_CAL_A);
  calB = prefs.getFloat("b", DEFAULT_CAL_B);

  Serial.println("# Lam Stage 2 flow skid controller");
  Serial.printf("# FT-01 calibration: Q = %.6f * f + %.4f\n", calA, calB);
  Serial.println("# ON/OFF: start Case 1, press for Case 2, press again to stop. Type 'help' for commands.");
  Serial.println("H,t_ms,state,pulses,f_Hz,Q_Lmin,Q_mLs,total_L,float_high,p1,p2,p3,sov");

  lastSampleMs = millis();
  lastSamplePulses = readPulses();
  enterState(IDLE);
}

void loop()
{
  pollSerial();

  if (buttonPressedEvent())
  {
    Serial.println("# Button pressed");
    if (state == IDLE || state == DONE)
      enterState(CASE1);
    else if (state == CASE1)
      enterState(FLUSH_PREP); // Case 1 -> Case 2 now, whatever time is left
    else if (state == FLUSH_PREP || state == FLUSH)
      enterState(DONE); // Case 2 -> stop
    else
      enterState(IDLE); // TEST: leave test mode
  }

  updateFloat();

  if (fsRaw)
  {
    if (digitalRead(PIN_FLOW))
      fsHighSamples++;
    else
      fsLowSamples++;
  }

  uint32_t now = millis();
  if (now - lastSampleMs >= LOG_MS)
  {
    float dt = (now - lastSampleMs) / 1000.0f;
    sampleFlow();
    if (state == TEST && piActive)
    {
      runFlowPI(dt);
    }
    else if (state == CASE1 && holdFlowCase1)
    {
      if (p1HeldByFloat)
        pumpSet(P2, 100); // sump high: drain as fast as possible
      else
        runFlowPI(dt);
    }
    Serial.printf("D,%lu,%s,%lu,%.2f,%.3f,%.2f,%.4f,%d,%.0f,%.0f,%.0f,%d\n",
                  (unsigned long)now, STATE_NAMES[state], (unsigned long)pulsesTotal, freqHz,
                  flowLpm, flowLpm * 1000.0f / 60.0f, totalL, floatHigh, pumps[0].actual,
                  pumps[1].actual, pumps[2].actual, sovOpen);
    if (fsRaw)
    {
      noInterrupts();
      uint32_t raw = isrRawEdges;
      interrupts();
      uint32_t samples = fsHighSamples + fsLowSamples;
      Serial.printf("# FS pin=%d high=%lu%% raw_edges=+%lu (total %lu) counted=%lu\n",
                    digitalRead(PIN_FLOW),
                    (unsigned long)(samples ? 100UL * fsHighSamples / samples : 0),
                    (unsigned long)(raw - fsLastRaw), (unsigned long)raw,
                    (unsigned long)readPulses());
      fsLastRaw = raw;
      fsHighSamples = fsLowSamples = 0;
    }
  }

  runStateMachine();
  pumpsUpdate();
  statusLed();
}
