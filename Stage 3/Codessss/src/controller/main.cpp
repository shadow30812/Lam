// ESP32 #1 (U2) · controller board
//
// Drives SV1..SV4 and pumps P1..P4, reads the START switch, receives flow-sensor counts from
// ESP32 #2 (U7) over UART and runs the whole batch:
//     dose line by line (fast, then slow for the last part, stop early by the learned overrun)
//  -> water chase into the mixing tank -> report -> wait for the hand valves to be switched
//  -> second START press: water flush to the 2 L jug.
//
// Line numbers used everywhere: 1 = Red (P1, FT1, SV1), 2 = Yellow (P2, FT2, SV2), 3 = Green (P3, FT3, SV3),
// 4 = Flush water (P4, SV4). The header outlet is split by two HAND ball valves the firmware cannot see:
//     BV-M (header -> mixing tank) and BV-J (header -> flush jug).
//     Batch and chase: BV-M open, BV-J shut.   Flush, prime, calibration: BV-M shut, BV-J open.
// Type "help" in the serial monitor (115200 baud, newline line ending) for every command.

#include <Arduino.h>
#include <Preferences.h>
#include "link.h"

// ---------------------------------------------------------------- pins (PCB sheets 2-5)
static const uint8_t SV_PIN[4]  = {23, 22, 21, 19};          // SV1..SV4, HIGH = coil on = valve open (NC valves)
static const uint8_t PWM_PIN[4] = {32, 33, 25, 26};          // PWM_1..PWM_4 -> MDD10A DRV1 ch1/ch2, DRV2 ch1/ch2
static const uint8_t START_PIN  = 14;                        // ON/OFF_SWITCH1 to SGND, internal pull-up

static const uint32_t PWM_FREQ = 20000;   // MDD10A accepts up to 20 kHz; inaudible
static const uint8_t  PWM_BITS = 10;

static const char *LINE_NAME[4] = {"RED", "YELLOW", "GREEN", "FLUSH"};
enum { SV_FLUSH = 3 };
static const char *NEED_JUG = "to the JUG: BV-M shut, BV-J open";
static const char *NEED_MIX = "to the MIXING TANK: BV-M open, BV-J shut";

// ---------------------------------------------------------------- settings (saved in flash)
struct LineCfg {
  float k;        // pulses per mL, from calibration. 0 = not calibrated yet
  float target;   // mL for one batch
  uint8_t fast;   // % duty in the fast phase
  uint8_t slow;   // % duty in the slow phase (must still pump reliably)
  float overrun;  // mL that still arrive after the stop command; learned each dose
};
struct Config {
  LineCfg line[3];
  uint8_t order[3];     // dose order, 0-based line index
  float bandPct;        // slow phase covers the last bandPct % of the dose ...
  float bandMinMl;      // ... but never less than this
  uint8_t p4;           // flush pump duty %
  float p4Rate;         // mL/s the flush pump delivers at p4 duty (measured in the dry run)
  uint32_t chaseMs;     // water chase into the mixing tank after the last colour
  uint32_t flushMs;     // water flush to the jug
  bool learn;           // update overrun automatically after every dose
};
static Config cfg;
static Preferences prefs;

static void setDefaults() {
  // k = 0 forces a calibration before any metered dose.
  cfg.line[0] = {0.0f, 1800.0f, 100, 35, 3.0f};  // Red 60 %
  cfg.line[1] = {0.0f, 300.0f, 100, 35, 3.0f};   // Yellow 10 %
  cfg.line[2] = {0.0f, 900.0f, 100, 35, 3.0f};   // Green 30 %
  cfg.order[0] = 1; cfg.order[1] = 2; cfg.order[2] = 0;  // Yellow -> Green -> Red
  cfg.bandPct = 10.0f;
  cfg.bandMinMl = 20.0f;
  cfg.p4 = 60;
  cfg.p4Rate = 30.0f;
  cfg.chaseMs = 1500;
  cfg.flushMs = 6000;
  cfg.learn = true;
}
static void saveConfig() {
  prefs.begin("choc", false);
  prefs.putBytes("cfg", &cfg, sizeof(cfg));
  prefs.end();
  Serial.println(F("saved"));
}
static void loadConfig() {
  prefs.begin("choc", true);
  size_t n = prefs.getBytesLength("cfg");
  if (n == sizeof(cfg)) prefs.getBytes("cfg", &cfg, sizeof(cfg));
  else setDefaults();
  prefs.end();
}

// ---------------------------------------------------------------- outputs with interlocks
static bool valveOpen[4] = {false};
static uint8_t pumpPct[4] = {0};

static void writePump(int p, uint8_t pct) {
  pumpPct[p] = pct;
  ledcWrite(p, (uint32_t)pct * ((1u << PWM_BITS) - 1) / 100);
}
static void allOff() {
  for (int p = 0; p < 4; p++) writePump(p, 0);
  delay(100);  // let pumps spin down before valves shut (no deadheading)
  for (int v = 0; v < 4; v++) { digitalWrite(SV_PIN[v], LOW); valveOpen[v] = false; }
}
// A pump may only run when its own line valve is open. The outlet hand valves are the operator's job.
static bool pathOk(int p) { return valveOpen[p]; }

static bool setPump(int p, uint8_t pct) {
  if (pct > 100) pct = 100;
  if (pct > 0 && !pathOk(p)) {
    Serial.printf("! interlock: P%d needs SV%d open\n", p + 1, p + 1);
    return false;
  }
  writePump(p, pct);
  return true;
}
static bool setValve(int v, bool open) {
  if (!open && pumpPct[v]) {
    // closing a line valve first stops its pump, so it never runs against a closed valve
    writePump(v, 0);
    delay(100);
  }
  digitalWrite(SV_PIN[v], open ? HIGH : LOW);
  valveOpen[v] = open;
  return true;
}

// ---------------------------------------------------------------- UART link from U7
static uint32_t counts[3] = {0, 0, 0};
static uint32_t lastFrameMs = 0, framesOk = 0, framesBad = 0;
static bool linkOk() { return framesOk > 0 && millis() - lastFrameMs < 150; }

static void pollLink() {
  static char buf[80];
  static uint8_t n = 0;
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      buf[n] = 0;
      char *star = strchr(buf, '*');
      unsigned long seq, a, b, d;
      if (buf[0] == 'F' && star && sscanf(buf, "F,%lu,%lu,%lu,%lu", &seq, &a, &b, &d) == 4 &&
          (uint8_t)strtoul(star + 1, nullptr, 16) == linkChecksum(buf, star - buf + 1)) {
        counts[0] = a; counts[1] = b; counts[2] = d;
        lastFrameMs = millis();
        framesOk++;
      } else {
        framesBad++;
      }
      n = 0;
    } else if (c != '\r') {
      if (n < sizeof(buf) - 1) buf[n++] = c; else n = 0;
    }
  }
}

// ---------------------------------------------------------------- dose engine (one line at a time)
enum Phase { D_IDLE, D_OPEN, D_FAST, D_SLOW, D_STOP, D_SETTLE, D_DONE, D_FAULT };
static const char *PHASE_NAME[] = {"idle", "open", "fast", "slow", "stop", "settle", "done", "FAULT"};

static const uint32_t VALVE_MS = 200;        // valve fully open before pump starts
static const uint32_t STOP_TO_CLOSE_MS = 150;  // pump stops, then line valve closes
static const uint32_t SETTLE_MS = 800;       // count late pulses after valve closes
static const uint32_t START_STALL_MS = 4000; // no pulses at all this long after start = fault
static const uint32_t STALL_MS = 2000;       // pulses stop mid-dose this long = fault
static const uint32_t MAX_DOSE_MS = 240000;

struct Dose {
  Phase ph = D_IDLE;
  int line = 0;
  float target = 0;       // mL; ignored in timed mode
  bool timed = false;     // calibration run: fixed duty for runMs, no metering
  uint32_t runMs = 0;
  uint8_t duty = 0;
  bool toMix = false;     // only for messages: the hand valves decide where it really goes
  uint32_t t0 = 0, tPh = 0, startCount = 0, stopCount = 0, lastPulseMs = 0, lastCount = 0;
  float mlAtStop = 0, mlFinal = 0;
  uint32_t pulsesFinal = 0;
  char fault[64] = "";
} dose;

static uint32_t lastPulses[3] = {0, 0, 0};  // pulses of the latest finished dose/calrun per line (for "k")

static float mlOf(int line, uint32_t pulses) {
  return cfg.line[line].k > 0 ? pulses / cfg.line[line].k : 0;
}
static uint32_t dosePulses() { return counts[dose.line] - dose.startCount; }

static void doseFault(const char *why) {
  allOff();
  dose.ph = D_FAULT;
  strncpy(dose.fault, why, sizeof(dose.fault) - 1);
  Serial.printf("! FAULT on %s: %s. All pumps off, all valves closed.\n", LINE_NAME[dose.line], why);
}

static bool doseBegin(int line, float target, bool toMix, bool timed, uint32_t runMs, uint8_t duty) {
  if (!linkOk()) { Serial.println(F("! no data from sensor board U7, check UART wiring and that U7 is powered")); return false; }
  if (!timed && cfg.line[line].k <= 0) { Serial.printf("! %s not calibrated (k=0). Run calrun/k first.\n", LINE_NAME[line]); return false; }
  dose = Dose();
  dose.line = line; dose.target = target; dose.timed = timed; dose.runMs = runMs;
  dose.duty = duty; dose.toMix = toMix;
  setValve(line, true);
  dose.startCount = dose.lastCount = counts[line];
  dose.t0 = dose.tPh = dose.lastPulseMs = millis();
  dose.ph = D_OPEN;
  if (timed) Serial.printf("> run %s %lu ms at %u%% (%s)\n", LINE_NAME[line], (unsigned long)runMs, duty, NEED_JUG);
  else Serial.printf("> dose %s %.1f mL (%s)\n", LINE_NAME[line], target, toMix ? NEED_MIX : NEED_JUG);
  return true;
}

static void doseTelemetry(uint32_t now) {
  static uint32_t nextT = 0, prevCount = 0, prevMs = 0;
  if (dose.ph < D_FAST || dose.ph > D_SETTLE || (int32_t)(now - nextT) < 0) return;
  nextT = now + 250;
  uint32_t p = dosePulses();
  float rate = (prevMs && now > prevMs && p >= prevCount) ? mlOf(dose.line, p - prevCount) * 1000.0f / (now - prevMs) : 0;
  prevCount = p; prevMs = now;
  // CSV: D,t_ms,line,phase,duty%,pulses,mL,mL_per_s
  Serial.printf("D,%lu,%d,%s,%u,%lu,%.1f,%.1f\n", (unsigned long)(now - dose.t0), dose.line + 1, PHASE_NAME[dose.ph],
                pumpPct[dose.line], (unsigned long)p, mlOf(dose.line, p), rate);
}

static void doseStep() {
  if (dose.ph == D_IDLE || dose.ph == D_DONE || dose.ph == D_FAULT) return;
  uint32_t now = millis();
  LineCfg &L = cfg.line[dose.line];
  uint32_t c = counts[dose.line];
  if (c != dose.lastCount) { dose.lastCount = c; dose.lastPulseMs = now; }
  uint32_t p = dosePulses();
  float ml = mlOf(dose.line, p);

  if (dose.ph <= D_SLOW && !linkOk()) { doseFault("lost sensor link"); return; }
  if (now - dose.t0 > MAX_DOSE_MS) { doseFault("dose took too long"); return; }
  if (dose.ph == D_FAST || dose.ph == D_SLOW) {
    uint32_t limit = (p == 0) ? START_STALL_MS : STALL_MS;
    if (now - dose.lastPulseMs > limit) { doseFault(p == 0 ? "no flow after start (empty tank, air, valve shut?)" : "flow stopped (tank empty or pump stalled)"); return; }
  }
  doseTelemetry(now);

  float band = max(dose.target * cfg.bandPct / 100.0f, cfg.bandMinMl);
  float stopAt = dose.target - L.overrun;

  switch (dose.ph) {
    case D_OPEN:
      if (now - dose.tPh >= VALVE_MS) {
        uint8_t d = dose.timed ? dose.duty : L.fast;
        if (!dose.timed && ml >= dose.target - band) d = L.slow;  // tiny dose: go straight to slow
        if (!setPump(dose.line, d)) { doseFault("interlock refused pump start"); return; }
        dose.ph = (d == L.slow && !dose.timed) ? D_SLOW : D_FAST;
        dose.tPh = now; dose.lastPulseMs = now;
      }
      break;
    case D_FAST:
      if (dose.timed) {
        if (now - dose.tPh >= dose.runMs) goto stop;
      } else if (ml >= stopAt) {
        goto stop;
      } else if (ml >= dose.target - band) {
        writePump(dose.line, L.slow);
        dose.ph = D_SLOW; dose.tPh = now;
      }
      break;
    case D_SLOW:
      if (ml >= stopAt) goto stop;
      break;
    case D_STOP:
      if (now - dose.tPh >= STOP_TO_CLOSE_MS) {
        setValve(dose.line, false);
        dose.ph = D_SETTLE; dose.tPh = now;
      }
      break;
    case D_SETTLE:
      if (now - dose.tPh >= SETTLE_MS && now - dose.lastPulseMs >= 300) {
        dose.pulsesFinal = p;
        dose.mlFinal = ml;
        lastPulses[dose.line] = p;
        if (dose.timed) {
          Serial.printf("= calrun %s: %lu pulses in %lu ms (%.1f Hz). Now weigh the jug and type: k %d <mL>\n",
                        LINE_NAME[dose.line], (unsigned long)p, (unsigned long)dose.runMs,
                        p * 1000.0f / dose.runMs, dose.line + 1);
        } else {
          float coast = dose.mlFinal - dose.mlAtStop;
          if (cfg.learn && coast >= 0 && coast < 50) L.overrun = 0.5f * L.overrun + 0.5f * coast;
          Serial.printf("= %s target %.1f mL, metered %.1f mL (error %+.1f mL, %+.2f %%), coast %.1f mL, next overrun %.1f mL, pulses %lu\n",
                        LINE_NAME[dose.line], dose.target, dose.mlFinal, dose.mlFinal - dose.target,
                        (dose.mlFinal - dose.target) * 100.0f / dose.target, coast, L.overrun, (unsigned long)p);
        }
        dose.ph = D_DONE;
      }
      break;
    default:
      break;
  }
  return;
stop:
  writePump(dose.line, 0);
  dose.mlAtStop = ml;
  dose.stopCount = c;
  dose.ph = D_STOP; dose.tPh = now;
}

// ---------------------------------------------------------------- simple timed flush-pump runs
struct Timed { bool on = false; uint32_t t0 = 0, ms = 0; int phase = 0; bool toMix = false; } p4run;
static float jugMl = 0;  // estimated water in the jug since it was last emptied

static bool p4Begin(bool toMix, uint32_t ms) {
  if (!toMix && jugMl + cfg.p4Rate * ms / 1000.0f > 1800.0f) {
    Serial.printf("! jug would overflow (now ~%.0f mL). Empty it, type 'jug 0', then 'flush'.\n", jugMl);
    return false;
  }
  setValve(SV_FLUSH, true);
  p4run = Timed();
  p4run.on = true; p4run.t0 = millis(); p4run.ms = ms; p4run.toMix = toMix;
  Serial.printf("> P4 water %lu ms, %s (%s)\n", (unsigned long)ms, toMix ? "chase" : "flush", toMix ? NEED_MIX : NEED_JUG);
  return true;
}
static void p4Step() {
  if (!p4run.on) return;
  uint32_t now = millis();
  switch (p4run.phase) {
    case 0:
      if (now - p4run.t0 >= VALVE_MS) {
        if (!setPump(3, cfg.p4)) { allOff(); p4run.on = false; return; }
        p4run.phase = 1; p4run.t0 = now;
      }
      break;
    case 1:
      if (now - p4run.t0 >= p4run.ms) { writePump(3, 0); p4run.phase = 2; p4run.t0 = now; }
      break;
    case 2:
      if (now - p4run.t0 >= STOP_TO_CLOSE_MS) {
        setValve(SV_FLUSH, false);
        if (!p4run.toMix) jugMl += cfg.p4Rate * p4run.ms / 1000.0f;
        p4run.on = false;
        Serial.printf("= P4 done. Jug estimate ~%.0f mL\n", jugMl);
      }
      break;
  }
}

// ---------------------------------------------------------------- batch sequence
enum Batch { B_IDLE, B_LINE, B_CHASE, B_WAIT_FLUSH, B_FLUSH, B_DONE, B_ABORTED };
static Batch batch = B_IDLE;
static int batchIdx = 0;
static float result[3];
static uint32_t batchT0 = 0;

static bool busy() { return batch == B_LINE || batch == B_CHASE || batch == B_FLUSH || p4run.on ||
                            (dose.ph != D_IDLE && dose.ph != D_DONE && dose.ph != D_FAULT); }

static void batchAbort(const char *why) {
  allOff();
  p4run.on = false;
  if (dose.ph != D_DONE && dose.ph != D_IDLE) dose.ph = D_FAULT;
  batch = B_ABORTED;
  Serial.printf("! BATCH ABORTED: %s\n", why);
}

static void batchStart() {
  if (busy()) { Serial.println(F("! busy")); return; }
  for (int i = 0; i < 3; i++)
    if (cfg.line[i].k <= 0) { Serial.printf("! %s not calibrated, batch refused\n", LINE_NAME[i]); return; }
  if (!linkOk()) { Serial.println(F("! no data from sensor board U7, batch refused")); return; }
  allOff();
  Serial.printf("\n>>> BATCH START (BV-M open, BV-J shut)  order %s -> %s -> %s, then chase %lu ms, flush %lu ms\n",
                LINE_NAME[cfg.order[0]], LINE_NAME[cfg.order[1]], LINE_NAME[cfg.order[2]],
                (unsigned long)cfg.chaseMs, (unsigned long)cfg.flushMs);
  batchT0 = millis();
  batchIdx = 0;
  int l = cfg.order[0];
  if (!doseBegin(l, cfg.line[l].target, true, false, 0, 0)) { batchAbort("first dose would not start"); return; }
  batch = B_LINE;
}

static void batchReport() {
  Serial.println(F("\n===== BATCH REPORT ====="));
  float tot = 0;
  for (int i = 0; i < 3; i++) {
    int l = cfg.order[i];
    tot += result[l];
    Serial.printf("%-7s target %7.1f mL  metered %7.1f mL  error %+6.1f mL (%+.2f %%)\n", LINE_NAME[l],
                  cfg.line[l].target, result[l], result[l] - cfg.line[l].target,
                  (result[l] - cfg.line[l].target) * 100.0f / cfg.line[l].target);
  }
  Serial.printf("colour total %.1f mL, + chase water ~%.0f mL, time %.1f s\n", tot,
                cfg.p4Rate * cfg.chaseMs / 1000.0f, (millis() - batchT0) / 1000.0f);
  for (int i = 0; i < 3; i++) {
    int l = cfg.order[i];
    Serial.printf("ratio %s %.1f %%  ", LINE_NAME[l], tot > 0 ? result[l] * 100.0f / tot : 0);
  }
  Serial.println(F("\n========================"));
}

static void batchStep() {
  switch (batch) {
    case B_LINE:
      if (dose.ph == D_FAULT) { batchAbort(dose.fault); break; }
      if (dose.ph == D_DONE) {
        result[dose.line] = dose.mlFinal;
        if (++batchIdx < 3) {
          int l = cfg.order[batchIdx];
          if (!doseBegin(l, cfg.line[l].target, true, false, 0, 0)) batchAbort("next dose would not start");
        } else {
          // BV-M is still open: push the last colour out of the header into the mixing tank
          if (cfg.chaseMs == 0) batch = B_CHASE;
          else if (p4Begin(true, cfg.chaseMs)) batch = B_CHASE;
          else batchAbort("chase would not start");
        }
      }
      break;
    case B_CHASE:
      if (!p4run.on) {
        batchReport();
        batch = B_WAIT_FLUSH;
        Serial.println(F(">>> BATCH COMPLETE. To flush: SHUT BV-M, OPEN BV-J, then press START (or type flush)."));
      }
      break;
    case B_FLUSH:
      if (!p4run.on) {
        batch = B_DONE;
        Serial.println(F(">>> FLUSH DONE. Before the next batch: SHUT BV-J, OPEN BV-M. Empty the jug when near 2 L."));
      }
      break;
    default:
      break;
  }
}

static void batchFlush() {
  if (busy()) { Serial.println(F("! busy")); return; }
  if (cfg.flushMs == 0 || !p4Begin(false, cfg.flushMs)) { Serial.println(F("! flush not started")); return; }
  batch = B_FLUSH;
}

// ---------------------------------------------------------------- start switch
static void pollStart() {
  static bool last = true, stable = true;
  static uint32_t tChange = 0;
  bool r = digitalRead(START_PIN);
  if (r != last) { last = r; tChange = millis(); }
  if (millis() - tChange > 40 && r != stable) {
    stable = r;
    if (!stable) {  // pressed (pulled to SGND)
      if (busy()) batchAbort("START pressed during run (used as stop)");
      else if (batch == B_WAIT_FLUSH) batchFlush();
      else batchStart();
    }
  }
}

// ---------------------------------------------------------------- serial commands
static void printConfig() {
  Serial.println(F("line    k(p/mL)  target  fast%  slow%  overrun"));
  for (int i = 0; i < 3; i++)
    Serial.printf("%d %-6s %7.3f %7.1f %5u %6u %8.1f\n", i + 1, LINE_NAME[i], cfg.line[i].k, cfg.line[i].target,
                  cfg.line[i].fast, cfg.line[i].slow, cfg.line[i].overrun);
  Serial.printf("order %d%d%d  band %.1f %% (min %.0f mL)  learn %d\n", cfg.order[0] + 1, cfg.order[1] + 1, cfg.order[2] + 1,
                cfg.bandPct, cfg.bandMinMl, cfg.learn);
  Serial.printf("p4 %u %%  p4rate %.1f mL/s  chase %lu ms  flush %lu ms  jug ~%.0f mL\n", cfg.p4, cfg.p4Rate,
                (unsigned long)cfg.chaseMs, (unsigned long)cfg.flushMs, jugMl);
}
static void printStatus() {
  Serial.printf("link %s (ok %lu, bad %lu)  counts %lu %lu %lu  start=%s\n", linkOk() ? "OK" : "DOWN",
                (unsigned long)framesOk, (unsigned long)framesBad, (unsigned long)counts[0], (unsigned long)counts[1],
                (unsigned long)counts[2], digitalRead(START_PIN) ? "up" : "PRESSED");
  Serial.print(F("valves "));
  for (int v = 0; v < 4; v++) Serial.printf("SV%d=%d ", v + 1, valveOpen[v]);
  Serial.print(F(" pumps "));
  for (int p = 0; p < 4; p++) Serial.printf("P%d=%u%% ", p + 1, pumpPct[p]);
  static const char *BN[] = {"idle", "dosing", "chase", "WAITING FOR FLUSH (shut BV-M, open BV-J, press START)", "flush", "done", "aborted"};
  Serial.printf("\nbatch %s  dose %s\n", BN[batch], PHASE_NAME[dose.ph]);
}
static void help() {
  Serial.println(F(
      "--- Team 13 Chocolate controller (line 1=RED 2=YELLOW 3=GREEN 4=FLUSH) ---\n"
      "start | abort          run / stop the full batch (same as the START switch)\n"
      "                       after a batch, the next START press runs the flush\n"
      "  hand valves: batch/chase = BV-M open, BV-J shut; everything else = BV-M shut, BV-J open\n"
      "status | show          live I/O state | settings\n"
      "prime <1-4> <sec>      run a line into the JUG at fast duty (fill lines, purge air)\n"
      "calrun <1-3> <sec> [%] timed run into the JUG, counts pulses (calibration)\n"
      "k <1-3> <mL>           set k = pulses of last calrun/dose / mL weighed\n"
      "dose <1-3> <mL> [mix]  metered dose with full fast/slow profile (default to JUG)\n"
      "chase | flush          water into mixing tank | water flush to jug\n"
      "jug <mL>               set jug estimate (jug 0 after emptying)\n"
      "valve <1-4> <0|1>      manual valve, idle only\n"
      "pump <1-4> <%>         manual pump, idle only, interlocked\n"
      "off                    everything off\n"
      "set <key> <value>      keys: k1..3 tgt1..3 fast1..3 slow1..3 ovr1..3 order(e.g. 231)\n"
      "                       band bandmin p4 p4rate chase flush learn\n"
      "save | load | defaults settings in flash"));
}

static void handleCommand(String line) {
  line.trim();
  if (!line.length()) return;
  char buf[96];
  line.toCharArray(buf, sizeof(buf));
  char *cmd = strtok(buf, " ");
  char *a1 = strtok(nullptr, " ");
  char *a2 = strtok(nullptr, " ");
  char *a3 = strtok(nullptr, " ");
  String c(cmd);
  c.toLowerCase();
  int n = a1 ? atoi(a1) : 0;

  if (c == "help" || c == "?") help();
  else if (c == "status") printStatus();
  else if (c == "show") printConfig();
  else if (c == "start") { if (batch == B_WAIT_FLUSH) batchFlush(); else batchStart(); }
  else if (c == "abort" || c == "stop") batchAbort("abort command");
  else if (c == "off") { allOff(); p4run.on = false; dose.ph = D_IDLE; batch = B_IDLE; Serial.println(F("all off")); }
  else if (busy()) Serial.println(F("! busy, type abort first"));
  else if (c == "prime" && n >= 1 && n <= 4 && a2) {
    uint32_t ms = (uint32_t)(atof(a2) * 1000);
    if (n == 4) p4Begin(false, ms);
    else doseBegin(n - 1, 0, false, true, ms, cfg.line[n - 1].fast);
  } else if (c == "calrun" && n >= 1 && n <= 3 && a2) {
    uint8_t duty = a3 ? atoi(a3) : cfg.line[n - 1].fast;
    doseBegin(n - 1, 0, false, true, (uint32_t)(atof(a2) * 1000), duty);
  } else if (c == "k" && n >= 1 && n <= 3 && a2) {
    float ml = atof(a2);
    if (ml <= 0 || lastPulses[n - 1] == 0) Serial.println(F("! need a calrun or dose on that line first, and mL > 0"));
    else {
      cfg.line[n - 1].k = lastPulses[n - 1] / ml;
      Serial.printf("k%d = %lu / %.1f = %.4f pulses/mL (type save to keep)\n", n, (unsigned long)lastPulses[n - 1], ml, cfg.line[n - 1].k);
    }
  } else if (c == "dose" && n >= 1 && n <= 3 && a2) {
    bool mix = a3 && String(a3) == "mix";
    doseBegin(n - 1, atof(a2), mix, false, 0, 0);
  } else if (c == "chase") p4Begin(true, cfg.chaseMs);
  else if (c == "flush") { if (batch == B_WAIT_FLUSH) batchFlush(); else p4Begin(false, cfg.flushMs); }
  else if (c == "jug" && a1) { jugMl = atof(a1); Serial.printf("jug ~%.0f mL\n", jugMl); }
  else if (c == "valve" && n >= 1 && n <= 4 && a2) { setValve(n - 1, atoi(a2)); printStatus(); }
  else if (c == "pump" && n >= 1 && n <= 4 && a2) { setPump(n - 1, atoi(a2)); printStatus(); }
  else if (c == "save") saveConfig();
  else if (c == "load") { loadConfig(); printConfig(); }
  else if (c == "defaults") { setDefaults(); printConfig(); Serial.println(F("(not saved yet)")); }
  else if (c == "set" && a1 && a2) {
    String k(a1);
    k.toLowerCase();
    float v = atof(a2);
    int i = k.length() > 0 ? k.charAt(k.length() - 1) - '1' : -1;
    String base = k.substring(0, k.length() - 1);
    bool ok = true;
    if (i >= 0 && i < 3 && base == "k") cfg.line[i].k = v;
    else if (i >= 0 && i < 3 && base == "tgt") cfg.line[i].target = v;
    else if (i >= 0 && i < 3 && base == "fast") cfg.line[i].fast = constrain((int)v, 0, 100);
    else if (i >= 0 && i < 3 && base == "slow") cfg.line[i].slow = constrain((int)v, 0, 100);
    else if (i >= 0 && i < 3 && base == "ovr") cfg.line[i].overrun = v;
    else if (k == "order" && strlen(a2) == 3) {
      uint8_t o[3]; bool seen[3] = {false, false, false};
      for (int j = 0; j < 3; j++) { o[j] = a2[j] - '1'; if (o[j] > 2 || seen[o[j]]) ok = false; else seen[o[j]] = true; }
      if (ok) memcpy(cfg.order, o, 3);
    } else if (k == "band") cfg.bandPct = v;
    else if (k == "bandmin") cfg.bandMinMl = v;
    else if (k == "p4") cfg.p4 = constrain((int)v, 0, 100);
    else if (k == "p4rate") cfg.p4Rate = v;
    else if (k == "chase") cfg.chaseMs = (uint32_t)v;
    else if (k == "flush") cfg.flushMs = (uint32_t)v;
    else if (k == "learn") cfg.learn = v != 0;
    else ok = false;
    if (ok) { printConfig(); Serial.println(F("(type save to keep)")); }
    else Serial.println(F("! unknown key or bad value"));
  } else Serial.println(F("? type help"));
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  // Valves first: NC valves stay shut while the board boots.
  for (int v = 0; v < 4; v++) { pinMode(SV_PIN[v], OUTPUT); digitalWrite(SV_PIN[v], LOW); }
  for (int p = 0; p < 4; p++) { ledcSetup(p, PWM_FREQ, PWM_BITS); ledcAttachPin(PWM_PIN[p], p); ledcWrite(p, 0); }
  pinMode(START_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  Serial2.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  loadConfig();
  delay(300);
  Serial.println(F("\n[U2 controller] Team 13 Chocolate. Type help.\nFor a batch: BV-M OPEN, BV-J SHUT, then press START."));
  printConfig();
}

void loop() {
  pollLink();
  pollStart();
  doseStep();
  p4Step();
  batchStep();
  if (Serial.available()) handleCommand(Serial.readStringUntil('\n'));
}
