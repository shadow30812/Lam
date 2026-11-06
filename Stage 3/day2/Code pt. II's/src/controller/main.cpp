#include <Arduino.h>
#include <Preferences.h>
#include "link.h"

static const bool SETTINGS_FROM_CODE = true;

static const float TUNE_K[4] = {1.8719f, 9.6364f, 1.9033f, 2.4310f};
static const float TUNE_TARGET[4] = {1800.0f, 300.0f, 900.0f, 2000.0f};
static const uint8_t TUNE_FAST[4] = {100, 100, 100, 100};
static const uint8_t TUNE_SLOW[4] = {70, 70, 70, 70};
static const float TUNE_OVERRUN[4] = {3.0f, 3.0f, 3.0f, 3.0f};
static const uint8_t TUNE_ORDER[3] = {2, 3, 1};
static const float TUNE_BAND_PCT = 10.0f;
static const float TUNE_BAND_MIN = 20.0f;
static const bool TUNE_LEARN = true;
static const float TUNE_RED_HOLDUP_ML = 0.0f;

static const uint32_t TUNE_PAUSE_S = 10;
static const float TUNE_DT_TARGET = 10.0f;
static const float TUNE_DT_MARGIN = 0.3f;
static const uint8_t TUNE_DT_CONFIRM = 3;
static const uint32_t TUNE_DT_TIMEOUT_MIN = 60;
static const uint8_t TUNE_LOOP_DUTY = 100;
static const uint32_t TUNE_LOOP_CHECK_S = 180;
static const float TUNE_LOOP_MIN_DROP = 0.3f;
static const uint32_t TUNE_TLOG_MS = 2000;
static const float TUNE_TS_OFFSET[2] = {0.00f, 0.00f};
static const uint32_t TUNE_TEMP_STALE_MS = 5000;

static const float TUNE_MIX_MAX_ML = 9000.0f;
static const float TUNE_JUG_MAX_ML = 2100.0f;
static const bool TUNE_FLOAT_LOW[4] = {false, false, false, false};
static const bool TUNE_FLOAT_MIX_FULL = false;
static const uint8_t TUNE_FLOAT_ALARM_LEVEL = LOW;
static const uint32_t TUNE_START_LOCKOUT_MS = 3000;

static const uint32_t TUNE_VALVE_MS = 200;
static const uint32_t TUNE_STOP_TO_CLOSE_MS = 150;
static const uint32_t TUNE_SETTLE_MS = 800;
static const uint32_t TUNE_SETTLE_QUIET_MS = 300;
static const uint32_t TUNE_START_STALL_MS = 4000;
static const uint32_t TUNE_STALL_MS = 2000;
static const uint32_t TUNE_MAX_DOSE_MS = 600000;
static const uint32_t TUNE_SLOW_BOOST_MS = 700;
static const uint8_t TUNE_SLOW_BOOST_STEP = 10;
static const float TUNE_COAST_MAX_ML = 50.0f;
static const uint32_t TUNE_LINK_TIMEOUT_MS = 150;

static const float TUNE_TCAL_MAX_SPREAD = 0.25f;
static const float TUNE_TCAL_ICE_RANGE = 2.0f;

static const uint8_t SV_PIN[5] = {23, 22, 21, 19, 18};
static const uint8_t PWM_PIN[4] = {32, 33, 25, 26};
static const uint8_t FSW_PIN[4] = {36, 39, 34, 35};
static const uint8_t START_PIN = 14;

static const uint32_t PWM_FREQ = 20000;
static const uint8_t PWM_BITS = 10;

enum
{
  SV_RED_SRC = 0,
  SV_YEL = 1,
  SV_GRN = 2,
  SV_WATER_SRC = 3,
  SV_LOOP = 4,
  NSV = 5
};
enum
{
  L_RED = 0,
  L_YELLOW = 1,
  L_GREEN = 2,
  L_WATER = 3,
  NLINES = 4
};
enum
{
  P_RED = 0,
  P_YEL = 1,
  P_GRN = 2,
  P_LOOP = 3
};

struct LineDef
{
  const char *name;
  uint8_t pump;
  uint8_t sensor;
  int8_t valveA;
  int8_t valveB;
};
static const LineDef LD[NLINES] = {
    {"RED", P_RED, 0, SV_RED_SRC, -1},
    {"YELLOW", P_YEL, 1, SV_YEL, -1},
    {"GREEN", P_GRN, 2, SV_GRN, -1},
    {"WATER", P_RED, 0, SV_WATER_SRC, -1},
};

struct LineCfg
{
  float k;
  float target;
  uint8_t fast;
  uint8_t slow;
  float overrun;
};
struct Config
{
  LineCfg line[NLINES];
  uint8_t order[3];
  float bandPct;
  float bandMinMl;
  bool learn;
  float dtTarget;
  float dtMargin;
  uint8_t dtConfirm;
  uint32_t dtTimeoutMin;
  uint8_t loopDuty;
  uint32_t loopCheckS;
  float loopMinDrop;
  float mixMaxMl;
  float jugMaxMl;
  float tsOffset[2];
  float redHoldupMl;
  uint32_t pauseS;
};
static Config cfg;
static Preferences prefs;

static void setDefaults()
{
  for (int i = 0; i < NLINES; i++)
    cfg.line[i] = {TUNE_K[i], TUNE_TARGET[i], TUNE_FAST[i], TUNE_SLOW[i], TUNE_OVERRUN[i]};
  for (int i = 0; i < 3; i++)
    cfg.order[i] = TUNE_ORDER[i] - 1;
  cfg.bandPct = TUNE_BAND_PCT;
  cfg.bandMinMl = TUNE_BAND_MIN;
  cfg.learn = TUNE_LEARN;
  cfg.dtTarget = TUNE_DT_TARGET;
  cfg.dtMargin = TUNE_DT_MARGIN;
  cfg.dtConfirm = TUNE_DT_CONFIRM;
  cfg.dtTimeoutMin = TUNE_DT_TIMEOUT_MIN;
  cfg.loopDuty = TUNE_LOOP_DUTY;
  cfg.loopCheckS = TUNE_LOOP_CHECK_S;
  cfg.loopMinDrop = TUNE_LOOP_MIN_DROP;
  cfg.mixMaxMl = TUNE_MIX_MAX_ML;
  cfg.jugMaxMl = TUNE_JUG_MAX_ML;
  cfg.tsOffset[0] = TUNE_TS_OFFSET[0];
  cfg.tsOffset[1] = TUNE_TS_OFFSET[1];
  cfg.redHoldupMl = TUNE_RED_HOLDUP_ML;
  cfg.pauseS = TUNE_PAUSE_S;
}

static void printCode()
{
  const LineCfg *L = cfg.line;
  Serial.println(F("---- TUNING values (lines RED, YELLOW, GREEN, WATER) ----"));
  Serial.printf("static const float TUNE_K[4] = {%.4ff, %.4ff, %.4ff, %.4ff};\n", L[0].k, L[1].k, L[2].k, L[3].k);
  Serial.printf("static const float TUNE_TARGET[4] = {%.1ff, %.1ff, %.1ff, %.1ff};\n", L[0].target, L[1].target, L[2].target, L[3].target);
  Serial.printf("static const uint8_t TUNE_FAST[4] = {%u, %u, %u, %u};\n", L[0].fast, L[1].fast, L[2].fast, L[3].fast);
  Serial.printf("static const uint8_t TUNE_SLOW[4] = {%u, %u, %u, %u};\n", L[0].slow, L[1].slow, L[2].slow, L[3].slow);
  Serial.printf("static const float TUNE_OVERRUN[4] = {%.1ff, %.1ff, %.1ff, %.1ff};\n", L[0].overrun, L[1].overrun, L[2].overrun, L[3].overrun);
  Serial.printf("static const uint8_t TUNE_ORDER[3] = {%d, %d, %d};\n", cfg.order[0] + 1, cfg.order[1] + 1, cfg.order[2] + 1);
  Serial.printf("static const float TUNE_BAND_PCT = %.1ff;\n", cfg.bandPct);
  Serial.printf("static const float TUNE_BAND_MIN = %.1ff;\n", cfg.bandMinMl);
  Serial.printf("static const bool TUNE_LEARN = %s;\n", cfg.learn ? "true" : "false");
  Serial.printf("static const float TUNE_RED_HOLDUP_ML = %.1ff;\n", cfg.redHoldupMl);
  Serial.printf("static const uint32_t TUNE_PAUSE_S = %lu;\n", (unsigned long)cfg.pauseS);
  Serial.printf("static const float TUNE_DT_TARGET = %.1ff;\n", cfg.dtTarget);
  Serial.printf("static const float TUNE_DT_MARGIN = %.1ff;\n", cfg.dtMargin);
  Serial.printf("static const uint8_t TUNE_DT_CONFIRM = %u;\n", cfg.dtConfirm);
  Serial.printf("static const uint32_t TUNE_DT_TIMEOUT_MIN = %lu;\n", (unsigned long)cfg.dtTimeoutMin);
  Serial.printf("static const uint8_t TUNE_LOOP_DUTY = %u;\n", cfg.loopDuty);
  Serial.printf("static const uint32_t TUNE_LOOP_CHECK_S = %lu;\n", (unsigned long)cfg.loopCheckS);
  Serial.printf("static const float TUNE_LOOP_MIN_DROP = %.1ff;\n", cfg.loopMinDrop);
  Serial.printf("static const float TUNE_MIX_MAX_ML = %.1ff;\n", cfg.mixMaxMl);
  Serial.printf("static const float TUNE_JUG_MAX_ML = %.1ff;\n", cfg.jugMaxMl);
  Serial.printf("static const float TUNE_TS_OFFSET[2] = {%.2ff, %.2ff};\n", cfg.tsOffset[0], cfg.tsOffset[1]);
}
static void saveConfig()
{
  prefs.begin("choc2", false);
  prefs.putBytes("cfg", &cfg, sizeof(cfg));
  prefs.end();
  Serial.println(SETTINGS_FROM_CODE
                     ? F("saved to flash, but SETTINGS_FROM_CODE is true, so the code values win at the next boot. Type code and paste it into the TUNING block.")
                     : F("saved"));
}
static void loadConfig()
{
  if (SETTINGS_FROM_CODE)
  {
    setDefaults();
    return;
  }
  prefs.begin("choc2", true);
  size_t n = prefs.getBytesLength("cfg");
  if (n == sizeof(cfg))
    prefs.getBytes("cfg", &cfg, sizeof(cfg));
  else
    setDefaults();
  prefs.end();
}

static bool valveOpen[NSV] = {false};
static uint8_t pumpPct[4] = {0};
static uint32_t pumpOnMs[4] = {0};

static void writePump(int p, uint8_t pct)
{
  if (pct && !pumpPct[p])
    pumpOnMs[p] = millis();
  pumpPct[p] = pct;
  ledcWrite(p, (uint32_t)pct * ((1u << PWM_BITS) - 1) / 100);
}
static void writeValve(int v, bool open)
{
  digitalWrite(SV_PIN[v], open ? HIGH : LOW);
  valveOpen[v] = open;
}
static void pumpsOff()
{
  for (int p = 0; p < 4; p++)
    writePump(p, 0);
}
static void allOff()
{
  pumpsOff();
  delay(100);
  for (int v = 0; v < NSV; v++)
    writeValve(v, false);
}
static bool anyPump()
{
  for (int p = 0; p < 4; p++)
    if (pumpPct[p])
      return true;
  return false;
}

static bool pathOk(int p)
{
  if (p == P_RED)
    return valveOpen[SV_RED_SRC] != valveOpen[SV_WATER_SRC];
  if (p == P_YEL)
    return valveOpen[SV_YEL];
  if (p == P_GRN)
    return valveOpen[SV_GRN];
  return valveOpen[SV_LOOP];
}
static int pumpOfValve(int v)
{
  if (v == SV_YEL)
    return P_YEL;
  if (v == SV_GRN)
    return P_GRN;
  if (v == SV_LOOP)
    return P_LOOP;
  return P_RED;
}

static bool setPump(int p, uint8_t pct)
{
  if (pct > 100)
    pct = 100;
  if (pct > 0 && !pathOk(p))
  {
    if (p == P_RED)
      Serial.println(F("! interlock: P1 needs exactly one of SV1 (red) / SV4 (water) open"));
    else
      Serial.printf("! interlock: P%d needs SV%d open\n", p + 1, p == P_LOOP ? 5 : p + 1);
    return false;
  }
  writePump(p, pct);
  return true;
}
static bool setValve(int v, bool open)
{
  if (open && ((v == SV_RED_SRC && valveOpen[SV_WATER_SRC]) || (v == SV_WATER_SRC && valveOpen[SV_RED_SRC])))
  {
    Serial.println(F("! interlock: SV1 and SV4 may never be open together (water would run into the red tank)"));
    return false;
  }
  int p = pumpOfValve(v);
  if (!open && pumpPct[p])
  {
    writePump(p, 0);
    delay(100);
  }
  writeValve(v, open);
  return true;
}

static uint32_t counts[3] = {0, 0, 0};
static uint32_t lastCountChangeMs[3] = {0, 0, 0};
static uint8_t fswU7 = 3;
static uint32_t lastFrameMs = 0, framesOk = 0, framesBad = 0;
static bool linkOk() { return framesOk > 0 && millis() - lastFrameMs < TUNE_LINK_TIMEOUT_MS; }

static float tempC[2] = {NAN, NAN};
static float tempRaw[2] = {NAN, NAN};
static const int TAVG = 8;
static float rawHist[2][TAVG];
static int nHist = 0, iHist = 0;
static uint32_t lastTempMs = 0, tempFrames = 0, badTempFrames = 0;
static uint32_t lastValidMs[2] = {0, 0};
static bool probeOk(int i) { return lastValidMs[i] && millis() - lastValidMs[i] < TUNE_TEMP_STALE_MS && !isnan(tempC[i]); }
static bool tempsOk() { return tempFrames > 0 && probeOk(0) && probeOk(1); }
static float deltaT() { return fabsf(tempC[0] - tempC[1]); }

static void pollLink()
{
  static char buf[96];
  static uint8_t n = 0;
  while (Serial2.available())
  {
    char c = Serial2.read();
    if (c == '\n')
    {
      buf[n] = 0;
      unsigned long seq, a, b, d;
      int f, h;
      long t1, t2;
      if (buf[0] == 'F' && linkCheck(buf) && sscanf(buf, "F,%lu,%lu,%lu,%lu,%d", &seq, &a, &b, &d, &f) == 5)
      {
        uint32_t nc[3] = {(uint32_t)a, (uint32_t)b, (uint32_t)d};
        uint32_t now = millis();
        for (int i = 0; i < 3; i++)
        {
          if (nc[i] != counts[i])
            lastCountChangeMs[i] = now;
          counts[i] = nc[i];
        }
        fswU7 = (uint8_t)f;
        lastFrameMs = now;
        framesOk++;
      }
      else if (buf[0] == 'T' && linkCheck(buf) && sscanf(buf, "T,%lu,%ld,%ld,%d", &seq, &t1, &t2, &h) == 4)
      {
        tempRaw[0] = t1 <= TEMP_INVALID ? NAN : t1 / 100.0f;
        tempRaw[1] = t2 <= TEMP_INVALID ? NAN : t2 / 100.0f;
        uint32_t nowT = millis();
        for (int i = 0; i < 2; i++)
        {
          if (isnan(tempRaw[i]))
          {
            badTempFrames++;
            continue;
          }
          tempC[i] = tempRaw[i] + cfg.tsOffset[i];
          lastValidMs[i] = nowT;
        }
        rawHist[0][iHist] = tempRaw[0];
        rawHist[1][iHist] = tempRaw[1];
        iHist = (iHist + 1) % TAVG;
        if (nHist < TAVG)
          nHist++;
        lastTempMs = nowT;
        tempFrames++;
      }
      else
      {
        framesBad++;
      }
      n = 0;
    }
    else if (c != '\r')
    {
      if (n < sizeof(buf) - 1)
        buf[n++] = c;
      else
        n = 0;
    }
  }
}

struct TPoint
{
  float t, a, b;
  uint8_t h;
};
struct DPoint
{
  float t;
  uint8_t line, duty;
  float ml, rate;
};
struct Result
{
  bool done;
  float target, ml, secs;
  uint32_t pulses;
};
struct TCalRow
{
  uint8_t probe;
  char method;
  float raw, ref, offset;
};
static TCalRow qlog[8];
static int nQ = 0;

struct CalRow
{
  uint8_t line;
  uint32_t pulses, ms;
  float ml, k;
};
static const int MAX_T = 900, MAX_D = 2400, MAX_C = 16;
static TPoint tlog[MAX_T];
static DPoint dlog[MAX_D];
static CalRow clog[MAX_C];
static int nT = 0, nD = 0, nC = 0;
static uint32_t tStride = 1, tTick = 0;
static Result res[NLINES];
static float dtReached = NAN, dtSecs = 0;
static uint32_t runNo = 0, runT0 = 0;
static bool runLogging = false;
static bool tlogIdle = false;
static float runSecs() { return (millis() - runT0) / 1000.0f; }

static void logTemp(bool force)
{
  static uint32_t next = 0;
  uint32_t now = millis();
  if (!force && (int32_t)(now - next) < 0)
    return;
  next = now + TUNE_TLOG_MS;
  if (!runLogging && !tlogIdle)
    return;
  float t = runLogging ? runSecs() : now / 1000.0f;
  uint8_t loopOn = pumpPct[P_LOOP] ? 1 : 0;
  Serial.printf("T,%.1f,%.2f,%.2f,%.2f,%d\n", t, tempC[0], tempC[1], deltaT(), loopOn);
  if (!runLogging)
    return;
  if (tTick++ % tStride)
    return;
  if (nT >= MAX_T)
  {
    for (int i = 0; i < MAX_T / 2; i++)
      tlog[i] = tlog[i * 2];
    nT = MAX_T / 2;
    tStride *= 2;
  }
  tlog[nT++] = {t, tempC[0], tempC[1], loopOn};
}

enum Phase
{
  D_IDLE,
  D_OPEN,
  D_FAST,
  D_SLOW,
  D_STOP,
  D_SETTLE,
  D_DONE,
  D_FAULT
};
static const char *PHASE_NAME[] = {"idle", "open", "fast", "slow", "stop", "settle", "done", "FAULT"};

struct Dose
{
  Phase ph = D_IDLE;
  int line = 0;
  float target = 0;
  bool timed = false;
  uint32_t runMs = 0;
  uint8_t duty = 0;
  bool toJug = false;
  uint32_t t0 = 0, tPh = 0, startCount = 0, lastPulseMs = 0, lastCount = 0;
  float mlAtStop = 0, mlFinal = 0;
  uint32_t pulsesFinal = 0;
  char fault[72] = "";
} dose;

static uint32_t lastPulses[NLINES] = {0, 0, 0, 0};
static uint32_t lastRunMs[NLINES] = {0, 0, 0, 0};
static float jugMl = 0, mixMl = 0;

static int sensorOf(int line) { return LD[line].sensor; }
static int pumpOf(int line) { return LD[line].pump; }
static float mlOf(int line, uint32_t pulses) { return cfg.line[line].k > 0 ? pulses / cfg.line[line].k : 0; }
static uint32_t dosePulses() { return counts[sensorOf(dose.line)] - dose.startCount; }
static bool doseActive() { return dose.ph != D_IDLE && dose.ph != D_DONE && dose.ph != D_FAULT; }

enum Seq
{
  S_IDLE,
  S_FLUSH,
  S_PAUSE,
  S_DT,
  S_DOSE,
  S_DONE,
  S_FAULT
};
static const char *SEQ_NAME[] = {"idle", "STEP 1 flush", "pause before STEP 2", "STEP 2 dT (cooling loop)", "STEP 3 dose", "done", "FAULT"};
static Seq seq = S_IDLE;
static char tripWhy[80] = "";

static void printReport();

static void trip(const char *why)
{
  pumpsOff();
  delay(100);
  for (int v = 0; v < NSV; v++)
    writeValve(v, false);
  if (doseActive())
  {
    dose.ph = D_FAULT;
    strncpy(dose.fault, why, sizeof(dose.fault) - 1);
  }
  strncpy(tripWhy, why, sizeof(tripWhy) - 1);
  bool wasRun = seq != S_IDLE && seq != S_DONE && seq != S_FAULT;
  if (wasRun)
    seq = S_FAULT;
  Serial.printf("!,TRIP,%.1f,%s\n", runLogging ? runSecs() : millis() / 1000.0f, why);
  Serial.printf("! INTERLOCK TRIP: %s. All pumps off, all valves closed.\n", why);
  if (wasRun)
  {
    Serial.printf("S,%.1f,FAULT,%s\n", runSecs(), why);
    runLogging = false;
    printReport();
  }
}

static bool loopStart()
{
  for (int v = 0; v < NSV; v++)
    if (valveOpen[v] && v != SV_LOOP)
      setValve(v, false);
  setValve(SV_LOOP, true);
  delay(TUNE_VALVE_MS);
  return setPump(P_LOOP, cfg.loopDuty);
}
static void loopStop()
{
  writePump(P_LOOP, 0);
  delay(TUNE_STOP_TO_CLOSE_MS);
  setValve(SV_LOOP, false);
}

static bool doseBegin(int line, float target, bool toJug, bool timed, uint32_t runMs, uint8_t duty)
{
  pollLink();
  if (!linkOk())
  {
    Serial.println(F("! no data from sensor board U7, check UART wiring and that U7 is powered"));
    return false;
  }
  if (!timed && cfg.line[line].k <= 0)
  {
    Serial.printf("! %s not calibrated (k=0). Run calrun/k first.\n", LD[line].name);
    return false;
  }
  if (toJug && (jugMl + (timed ? 0 : target) > cfg.jugMaxMl))
  {
    Serial.printf("! jug would overflow (now ~%.0f mL). Empty it and type: jug 0\n", jugMl);
    return false;
  }
  if (!toJug && !timed && mixMl + target > cfg.mixMaxMl)
  {
    Serial.printf("! mixing tank would pass %.0f mL (now ~%.0f mL). Empty it and type: mix 0\n", cfg.mixMaxMl, mixMl);
    return false;
  }
  if (pumpPct[P_LOOP])
    loopStop();
  for (int v = 0; v < NSV; v++)
    if (valveOpen[v])
      setValve(v, false);
  dose = Dose();
  dose.line = line;
  dose.target = target;
  dose.timed = timed;
  dose.runMs = runMs;
  dose.duty = duty;
  dose.toJug = toJug;
  setValve(LD[line].valveA, true);
  dose.startCount = dose.lastCount = counts[sensorOf(line)];
  dose.t0 = dose.tPh = dose.lastPulseMs = millis();
  dose.ph = D_OPEN;
  const char *where = toJug ? "into the JUG (jug must sit under the red outlet)" : "into the MIXING TANK";
  if (timed)
    Serial.printf("> run %s %lu ms at %u%% (%s)\n", LD[line].name, (unsigned long)runMs, duty, where);
  else
    Serial.printf("> dose %s %.1f mL (%s)\n", LD[line].name, target, where);
  return true;
}

static void doseTelemetry(uint32_t now)
{
  static uint32_t nextT = 0, prevCount = 0, prevMs = 0, nextStore = 0;
  if (dose.ph < D_FAST || dose.ph > D_SETTLE || (int32_t)(now - nextT) < 0)
    return;
  nextT = now + 250;
  uint32_t p = dosePulses();
  if (p < prevCount || now - prevMs > 1000)
    prevMs = 0;
  float rate = (prevMs && now > prevMs) ? mlOf(dose.line, p - prevCount) * 1000.0f / (now - prevMs) : 0;
  prevCount = p;
  prevMs = now;
  float ml = mlOf(dose.line, p);
  float t = runLogging ? runSecs() : (now - dose.t0) / 1000.0f;
  Serial.printf("D,%.2f,%d,%s,%u,%lu,%.1f,%.1f\n", t, dose.line + 1, PHASE_NAME[dose.ph],
                pumpPct[pumpOf(dose.line)], (unsigned long)p, ml, rate);
  if (runLogging && (int32_t)(now - nextStore) >= 0 && nD < MAX_D)
  {
    nextStore = now + 500;
    dlog[nD++] = {t, (uint8_t)(dose.line + 1), pumpPct[pumpOf(dose.line)], ml, rate};
  }
}

static void doseStep()
{
  if (!doseActive())
    return;
  uint32_t now = millis();
  LineCfg &L = cfg.line[dose.line];
  int pump = pumpOf(dose.line);
  uint32_t c = counts[sensorOf(dose.line)];
  if (c != dose.lastCount)
  {
    dose.lastCount = c;
    dose.lastPulseMs = now;
  }
  uint32_t p = dosePulses();
  float ml = mlOf(dose.line, p);

  if (dose.ph <= D_SLOW && !linkOk())
  {
    trip("lost sensor link");
    return;
  }
  if (now - dose.t0 > TUNE_MAX_DOSE_MS)
  {
    trip("dose took too long");
    return;
  }
  if (!dose.timed && dose.toJug && jugMl + ml > cfg.jugMaxMl)
  {
    trip("jug full");
    return;
  }
  if (!dose.timed && !dose.toJug && mixMl + ml > cfg.mixMaxMl)
  {
    trip("mixing tank full (volume count)");
    return;
  }
  if (dose.ph == D_FAST || dose.ph == D_SLOW)
  {
    uint32_t limit = (p == 0) ? TUNE_START_STALL_MS : TUNE_STALL_MS;
    if (dose.ph == D_SLOW && now - dose.lastPulseMs > TUNE_SLOW_BOOST_MS && pumpPct[pump] < L.fast)
    {
      int up = pumpPct[pump] + TUNE_SLOW_BOOST_STEP;
      uint8_t d = up > L.fast ? L.fast : (uint8_t)up;
      writePump(pump, d);
      L.slow = d;
      dose.lastPulseMs = now;
      Serial.printf("~ %s slow duty stalled the pump, raised to %u%% (now slow%d = %u; type code to keep it)\n",
                    LD[dose.line].name, d, dose.line + 1, d);
    }
    else if (now - dose.lastPulseMs > limit)
    {
      char why[72];
      snprintf(why, sizeof(why), "%s: %s", LD[dose.line].name,
               p == 0 ? "no flow after start (empty tank, air, valve shut?)" : "flow stopped (tank empty or pump stalled)");
      trip(why);
      return;
    }
  }
  doseTelemetry(now);

  float band = max(dose.target * cfg.bandPct / 100.0f, cfg.bandMinMl);
  float stopAt = dose.target - L.overrun;

  switch (dose.ph)
  {
  case D_OPEN:
    if (now - dose.tPh >= TUNE_VALVE_MS)
    {
      uint8_t d = dose.timed ? dose.duty : L.fast;
      if (!dose.timed && ml >= dose.target - band)
        d = L.slow;
      if (!setPump(pump, d))
      {
        trip("interlock refused pump start");
        return;
      }
      dose.ph = (d == L.slow && !dose.timed) ? D_SLOW : D_FAST;
      dose.tPh = now;
      dose.lastPulseMs = now;
    }
    break;
  case D_FAST:
    if (dose.timed)
    {
      if (now - dose.tPh >= dose.runMs)
        goto stop;
    }
    else if (ml >= stopAt)
    {
      goto stop;
    }
    else if (ml >= dose.target - band)
    {
      writePump(pump, L.slow);
      dose.ph = D_SLOW;
      dose.tPh = now;
    }
    break;
  case D_SLOW:
    if (ml >= stopAt)
      goto stop;
    break;
  case D_STOP:
    if (now - dose.tPh >= TUNE_STOP_TO_CLOSE_MS)
    {
      setValve(LD[dose.line].valveA, false);
      dose.ph = D_SETTLE;
      dose.tPh = now;
    }
    break;
  case D_SETTLE:
    if (now - dose.tPh >= TUNE_SETTLE_MS && now - dose.lastPulseMs >= TUNE_SETTLE_QUIET_MS)
    {
      dose.pulsesFinal = p;
      dose.mlFinal = ml;
      lastPulses[dose.line] = p;
      lastRunMs[dose.line] = dose.timed ? dose.runMs : now - dose.t0;
      if (dose.toJug)
        jugMl += ml;
      else
        mixMl += ml;
      if (dose.timed)
      {
        Serial.printf("= calrun %s: %lu pulses in %lu ms (%.1f Hz). Now weigh the collected liquid and type: k %d <mL>\n",
                      LD[dose.line].name, (unsigned long)p, (unsigned long)dose.runMs,
                      p * 1000.0f / dose.runMs, dose.line + 1);
      }
      else
      {
        float coast = dose.mlFinal - dose.mlAtStop;
        if (cfg.learn && coast >= 0 && coast < TUNE_COAST_MAX_ML)
          L.overrun = 0.5f * L.overrun + 0.5f * coast;
        Serial.printf("= %s target %.1f mL, metered %.1f mL (error %+.1f mL, %+.2f %%), coast %.1f mL, next overrun %.1f mL, pulses %lu, %.1f s\n",
                      LD[dose.line].name, dose.target, dose.mlFinal, dose.mlFinal - dose.target,
                      (dose.mlFinal - dose.target) * 100.0f / dose.target, coast, L.overrun, (unsigned long)p,
                      (now - dose.t0) / 1000.0f);
      }
      dose.ph = D_DONE;
    }
    break;
  default:
    break;
  }
  return;
stop:
  writePump(pump, 0);
  dose.mlAtStop = ml;
  dose.ph = D_STOP;
  dose.tPh = now;
}

static bool floatAlarm(uint8_t level) { return level == TUNE_FLOAT_ALARM_LEVEL; }

static void interlocks()
{
  uint32_t now = millis();
  if (anyPump() && !linkOk())
  {
    trip("lost sensor link while a pump was running");
    return;
  }
  for (int i = 0; i < 4; i++)
  {
    if (!TUNE_FLOAT_LOW[i] || !floatAlarm(digitalRead(FSW_PIN[i])))
      continue;
    bool feeding = (i == 0 && pumpPct[P_RED] && valveOpen[SV_RED_SRC]) || (i == 1 && (pumpPct[P_YEL] || pumpPct[P_LOOP])) ||
                   (i == 2 && pumpPct[P_GRN]) || (i == 3 && pumpPct[P_RED] && valveOpen[SV_WATER_SRC]);
    if (feeding)
    {
      char why[48];
      snprintf(why, sizeof(why), "low level float FSW%d (%s tank empty)", i + 1, LD[i].name);
      trip(why);
      return;
    }
  }
  if (TUNE_FLOAT_MIX_FULL && floatAlarm((fswU7 & 1) ? HIGH : LOW) && anyPump())
  {
    trip("mixing tank full (float FSW5)");
    return;
  }
  if (!doseActive())
  {
    for (int p = 0; p < 3; p++)
    {
      if (!pumpPct[p])
        continue;
      int s = p;
      uint32_t since = max(pumpOnMs[p], lastCountChangeMs[s]);
      uint32_t lim = lastCountChangeMs[s] > pumpOnMs[p] ? TUNE_STALL_MS : TUNE_START_STALL_MS;
      if (now - since > lim)
      {
        char why[48];
        snprintf(why, sizeof(why), "no flow on P%d (manual run)", p + 1);
        trip(why);
        return;
      }
    }
  }
}

static uint8_t dtConfirmN = 0;
static uint32_t dtT0 = 0, lastTempSeen = 0, pauseT0 = 0;
static float loopRefT = NAN;
static uint32_t loopRefMs = 0;
static uint16_t loopChecks = 0;
static int doseIdx = 0;

static void logStep(const char *what) { Serial.printf("S,%.1f,%s\n", runSecs(), what); }

static void recordResult(int line)
{
  float hold = (line == L_RED && seq == S_DOSE) ? cfg.redHoldupMl : 0;
  res[line] = {true, dose.target - hold, dose.mlFinal - hold, (millis() - dose.t0) / 1000.0f, dose.pulsesFinal};
  if (hold > 0)
    Serial.printf("H,%d,%s,%.1f,%.1f\n", line + 1, LD[line].name, hold, dose.mlFinal);
  Serial.printf("R,%d,%s,%.1f,%.1f,%+.1f,%lu,%.1f\n", line + 1, LD[line].name, res[line].target, res[line].ml,
                res[line].ml - res[line].target, (unsigned long)dose.pulsesFinal, res[line].secs);
}

static bool startDoseIdx()
{
  int l = cfg.order[doseIdx];
  float extra = l == L_RED ? cfg.redHoldupMl : 0;
  if (extra > 0)
    Serial.printf("> RED line still holds ~%.1f mL of flush water from Step 1, metering %.1f mL so %.1f mL of red reaches the tank\n",
                  extra, cfg.line[l].target + extra, cfg.line[l].target);
  return doseBegin(l, cfg.line[l].target + extra, false, false, 0, 0);
}

static void finishRun()
{
  allOff();
  seq = S_DONE;
  logStep("DONE");
  float tot = 0;
  for (int i = 0; i < 3; i++)
    tot += res[i].ml;
  Serial.println(F("\n===== SEQUENCE COMPLETE ====="));
  Serial.printf("Step 1 flush: %.1f mL of %.1f mL to the jug in %.1f s\n", res[L_WATER].ml, res[L_WATER].target, res[L_WATER].secs);
  Serial.printf("Step 2 dT: %.2f C reached after %.1f s\n", dtReached, dtSecs);
  for (int i = 0; i < 3; i++)
  {
    int l = cfg.order[i];
    Serial.printf("Step 3 %-7s target %7.1f mL  metered %7.1f mL  error %+6.1f mL (%+.2f %%)  ratio %.1f %%\n", LD[l].name,
                  res[l].target, res[l].ml, res[l].ml - res[l].target, (res[l].ml - res[l].target) * 100.0f / res[l].target,
                  tot > 0 ? res[l].ml * 100.0f / tot : 0);
  }
  Serial.printf("Total colour %.1f mL, run time %.1f s\n", tot, runSecs());
  Serial.print(F("SAFE STATE: "));
  for (int p = 0; p < 4; p++)
    Serial.printf("P%d=%u%% ", p + 1, pumpPct[p]);
  for (int v = 0; v < NSV; v++)
    Serial.printf("SV%d=%s ", v + 1, valveOpen[v] ? "OPEN" : "closed");
  Serial.println();
  Serial.println(F("=============================="));
  logTemp(true);
  runLogging = false;
  printReport();
  Serial.println(F(">>> All pumps OFF, all valves CLOSED. Press START for a new run (empty the jug first)."));
}

static void enterDose()
{
  if (pumpPct[P_LOOP] || valveOpen[SV_LOOP])
    loopStop();
  seq = S_DOSE;
  logStep("STEP3_START");
  doseIdx = 0;
  if (!startDoseIdx())
    trip("first colour dose would not start");
}

static void enterDt()
{
  seq = S_DT;
  dtT0 = millis();
  dtConfirmN = 0;
  lastTempSeen = tempFrames;
  loopRefT = tempC[0];
  loopRefMs = dtT0;
  loopChecks = 0;
  logStep("STEP2_START");
  if (cfg.dtTarget <= 0)
  {
    Serial.println(F("> STEP 2 skipped (dt set to 0, test mode)"));
    logStep("STEP2_SKIPPED");
    enterDose();
    return;
  }
  if (tempsOk() && deltaT() >= cfg.dtTarget + cfg.dtMargin)
    Serial.printf("> STEP 2: dT is already %.2f C, confirming over %u readings\n", deltaT(), cfg.dtConfirm);
  if (!loopStart())
  {
    trip("cooling loop would not start");
    return;
  }
  Serial.printf("> STEP 2: SV5 open, P4 %u%% circulating YELLOW through the ice-bath coil until |TS1 - TS2| >= %.1f C\n",
                cfg.loopDuty, cfg.dtTarget + cfg.dtMargin);
}

static void seqStep()
{
  uint32_t now = millis();
  switch (seq)
  {
  case S_FLUSH:
    if (dose.ph == D_FAULT)
      break;
    if (dose.ph == D_DONE)
    {
      recordResult(L_WATER);
      logStep("STEP1_DONE");
      Serial.printf("> STEP 1 DONE: %.1f mL flushed to the jug, P1 off, SV4 closed\n", dose.mlFinal);
      Serial.println(F(">>> LIFT THE JUG OUT OF THE MIXING TANK NOW <<<"));
      if (cfg.pauseS)
      {
        seq = S_PAUSE;
        pauseT0 = now;
        logStep("PAUSE_START");
        Serial.printf("> pause %lu s, Step 2 starts by itself\n", (unsigned long)cfg.pauseS);
      }
      else
        enterDt();
    }
    break;
  case S_PAUSE:
    if (now - pauseT0 >= cfg.pauseS * 1000UL)
    {
      logStep("PAUSE_DONE");
      enterDt();
    }
    break;
  case S_DT:
  {
    if (!tempsOk())
    {
      if (now - dtT0 > TUNE_TEMP_STALE_MS)
        trip("temperature sensor missing or not reporting");
      break;
    }
    if (isnan(loopRefT))
      loopRefT = tempC[0];
    if (cfg.loopCheckS && now - loopRefMs > cfg.loopCheckS * 1000UL)
    {
      float drop = loopRefT - tempC[0];
      Serial.printf("~ cooling check %u: TS1 dropped %.2f C in the last %lu s (need %.2f)\n", ++loopChecks, drop,
                    (unsigned long)cfg.loopCheckS, cfg.loopMinDrop);
      if (drop < cfg.loopMinDrop && deltaT() < cfg.dtTarget + cfg.dtMargin)
      {
        trip(loopChecks == 1 ? "no circulation in the cooling loop (TS1 not dropping)" : "cooling stopped (TS1 not dropping: loop blocked or ice used up)");
        break;
      }
      loopRefT = tempC[0];
      loopRefMs = now;
    }
    if (tempFrames != lastTempSeen && lastValidMs[0] == lastTempMs && lastValidMs[1] == lastTempMs)
    {
      lastTempSeen = tempFrames;
      if (deltaT() >= cfg.dtTarget + cfg.dtMargin)
        dtConfirmN++;
      else
        dtConfirmN = 0;
      if (dtConfirmN >= cfg.dtConfirm)
      {
        dtReached = deltaT();
        dtSecs = (now - dtT0) / 1000.0f;
        loopStop();
        logTemp(true);
        Serial.printf("> STEP 2 DONE: dT = %.2f C (TS1 %.2f, TS2 %.2f) after %.1f s, P4 off, SV5 closed\n", dtReached, tempC[0], tempC[1], dtSecs);
        logStep("STEP2_DONE");
        enterDose();
        break;
      }
    }
    if (cfg.dtTimeoutMin && now - dtT0 > cfg.dtTimeoutMin * 60000UL)
      trip("dT not reached in time");
    break;
  }
  case S_DOSE:
    if (dose.ph == D_FAULT)
      break;
    if (dose.ph == D_DONE)
    {
      recordResult(dose.line);
      if (++doseIdx < 3)
      {
        if (!startDoseIdx())
          trip("next colour dose would not start");
      }
      else
      {
        logStep("STEP3_DONE");
        finishRun();
      }
    }
    break;
  default:
    break;
  }
}

static bool running() { return seq == S_FLUSH || seq == S_PAUSE || seq == S_DT || seq == S_DOSE || doseActive(); }

static void runStart()
{
  if (running())
  {
    Serial.println(F("! busy"));
    return;
  }
  for (int i = 0; i < NLINES; i++)
    if (cfg.line[i].k <= 0)
    {
      Serial.printf("! %s not calibrated (k=0), run refused\n", LD[i].name);
      return;
    }
  if (!linkOk())
  {
    Serial.println(F("! no data from sensor board U7, run refused"));
    return;
  }
  if (cfg.dtTarget > 0 && !tempsOk())
  {
    Serial.println(F("! temperature sensors TS1/TS2 not both reading, run refused (type temps)"));
    return;
  }
  float need = cfg.line[0].target + cfg.line[1].target + cfg.line[2].target + cfg.redHoldupMl;
  if (mixMl + need > cfg.mixMaxMl)
  {
    Serial.printf("! mixing tank count is %.0f mL, + %.0f mL would pass %.0f mL. Empty it and type mix 0.\n", mixMl, need, cfg.mixMaxMl);
    return;
  }
  allOff();
  jugMl = 0;
  nT = nD = 0;
  tStride = 1;
  tTick = 0;
  for (int i = 0; i < NLINES; i++)
    res[i] = Result();
  dtReached = NAN;
  dtSecs = 0;
  tripWhy[0] = 0;
  runNo++;
  runT0 = millis();
  runLogging = true;
  Serial.printf("\n#RUN,%lu\n", (unsigned long)runNo);
  Serial.printf(">>> RUN %lu START. Step 1 flush %.0f mL water -> jug, Step 2 dT %.1f C by cooling, Step 3 %s %.0f -> %s %.0f -> %s %.0f mL\n",
                (unsigned long)runNo, cfg.line[L_WATER].target, cfg.dtTarget,
                LD[cfg.order[0]].name, cfg.line[cfg.order[0]].target, LD[cfg.order[1]].name, cfg.line[cfg.order[1]].target,
                LD[cfg.order[2]].name, cfg.line[cfg.order[2]].target);
  logStep("STEP1_START");
  logTemp(true);
  seq = S_FLUSH;
  if (!doseBegin(L_WATER, cfg.line[L_WATER].target, true, false, 0, 0))
    trip("flush would not start");
}

static bool rawAverage(float out[2], float spread[2])
{
  if (nHist < TAVG)
    return false;
  for (int p = 0; p < 2; p++)
  {
    float sum = 0, lo = 1e9f, hi = -1e9f;
    for (int i = 0; i < TAVG; i++)
    {
      float v = rawHist[p][i];
      if (isnan(v))
        return false;
      sum += v;
      lo = min(lo, v);
      hi = max(hi, v);
    }
    out[p] = sum / TAVG;
    spread[p] = hi - lo;
  }
  return true;
}

static void addQ(int probe, char method, float raw, float ref, float off)
{
  if (nQ < 8)
    qlog[nQ++] = {(uint8_t)probe, method, raw, ref, off};
  Serial.printf("Q,%d,TS%d,%s,%.3f,%.3f,%+.3f\n", probe + 1, probe + 1, method == 'i' ? "ice-point" : "match-TS1", raw, ref, off);
}

static void tempCal(const char *how)
{
  float avg[2], spread[2];
  if (!rawAverage(avg, spread))
  {
    Serial.println(F("! need 8 good readings from both probes first (about 7 s), try again"));
    return;
  }
  if (spread[0] > TUNE_TCAL_MAX_SPREAD || spread[1] > TUNE_TCAL_MAX_SPREAD)
  {
    Serial.printf("! readings still moving (TS1 spread %.2f C, TS2 %.2f C). Stir, wait 1 minute, try again.\n", spread[0], spread[1]);
    return;
  }
  String h(how);
  if (h == "ice")
  {
    if (fabsf(avg[0]) > TUNE_TCAL_ICE_RANGE || fabsf(avg[1]) > TUNE_TCAL_ICE_RANGE)
    {
      Serial.printf("! probes read %.2f / %.2f C, not near 0 C. Put both in stirred ice water (more ice than water).\n", avg[0], avg[1]);
      return;
    }
    for (int p = 0; p < 2; p++)
    {
      cfg.tsOffset[p] = -avg[p];
      addQ(p, 'i', avg[p], 0.0f, cfg.tsOffset[p]);
    }
  }
  else if (h == "match")
  {
    float target = avg[0] + cfg.tsOffset[0];
    cfg.tsOffset[1] = target - avg[1];
    addQ(1, 'm', avg[1], target, cfg.tsOffset[1]);
  }
  else if (h == "clear")
  {
    cfg.tsOffset[0] = cfg.tsOffset[1] = 0;
    Serial.println(F("probe offsets cleared"));
    return;
  }
  else
  {
    Serial.println(F("usage: tcal ice | tcal match | tcal clear"));
    return;
  }
  Serial.printf("offsets now TS1 %+.2f C, TS2 %+.2f C. Type code and paste TUNE_TS_OFFSET to keep them.\n", cfg.tsOffset[0], cfg.tsOffset[1]);
}

static void printCal()
{
  Serial.println(F("K,line,name,k_pulses_per_mL,sensor"));
  for (int i = 0; i < NLINES; i++)
    Serial.printf("K,%d,%s,%.4f,FT%d\n", i + 1, LD[i].name, cfg.line[i].k, sensorOf(i) + 1);
  for (int i = 0; i < nC; i++)
    Serial.printf("C,%d,%s,%lu,%lu,%.1f,%.4f\n", clog[i].line + 1, LD[clog[i].line].name, (unsigned long)clog[i].pulses,
                  (unsigned long)clog[i].ms, clog[i].ml, clog[i].k);
  Serial.printf("O,TS1,%+.3f\nO,TS2,%+.3f\n", cfg.tsOffset[0], cfg.tsOffset[1]);
  for (int i = 0; i < nQ; i++)
    Serial.printf("Q,%d,TS%d,%s,%.3f,%.3f,%+.3f\n", qlog[i].probe + 1, qlog[i].probe + 1, qlog[i].method == 'i' ? "ice-point" : "match-TS1",
                  qlog[i].raw, qlog[i].ref, qlog[i].offset);
}

static void printReport()
{
  Serial.printf("#REPORT BEGIN,%lu\n", (unsigned long)runNo);
  Serial.printf("P,dt_target,%.1f\nP,mode,cooling\n", cfg.dtTarget);
  for (int i = 0; i < NLINES; i++)
    Serial.printf("P,target%d,%.1f\n", i + 1, cfg.line[i].target);
  printCal();
  for (int i = 0; i < NLINES; i++)
    if (res[i].done)
      Serial.printf("R,%d,%s,%.1f,%.1f,%+.1f,%lu,%.1f\n", i + 1, LD[i].name, res[i].target, res[i].ml, res[i].ml - res[i].target,
                    (unsigned long)res[i].pulses, res[i].secs);
  if (!isnan(dtReached))
    Serial.printf("X,dt_reached,%.2f,%.1f\n", dtReached, dtSecs);
  if (tripWhy[0])
    Serial.printf("X,trip,%s\n", tripWhy);
  for (int i = 0; i < nT; i++)
    Serial.printf("T,%.1f,%.2f,%.2f,%.2f,%u\n", tlog[i].t, tlog[i].a, tlog[i].b, fabsf(tlog[i].a - tlog[i].b), tlog[i].h);
  for (int i = 0; i < nD; i++)
    Serial.printf("D,%.2f,%u,log,%u,0,%.1f,%.1f\n", dlog[i].t, dlog[i].line, dlog[i].duty, dlog[i].ml, dlog[i].rate);
  Serial.println(F("#REPORT END"));
}

static void pollStart()
{
  static bool last = true, stable = true;
  static uint32_t tChange = 0;
  bool r = digitalRead(START_PIN);
  if (r != last)
  {
    last = r;
    tChange = millis();
  }
  if (millis() - tChange > 40 && r != stable)
  {
    stable = r;
    if (!stable)
    {
      if (running() && millis() - runT0 < TUNE_START_LOCKOUT_MS)
        Serial.println(F("~ START ignored (pressed again right after the start)"));
      else if (running())
        trip("START pressed during run (used as stop)");
      else
        runStart();
    }
  }
}

static void printConfig()
{
  Serial.println(F("line      k(p/mL)  target  fast%  slow%  overrun  route"));
  static const char *ROUTE[] = {"SV1>TA>P1>FT1>mix", "TC>P2>FT2>SV2>mix", "P3>FT3>SV3>mix", "SV4>TA>P1>FT1>jug"};
  for (int i = 0; i < NLINES; i++)
    Serial.printf("%d %-6s %8.4f %7.1f %5u %6u %8.1f  %s\n", i + 1, LD[i].name, cfg.line[i].k, cfg.line[i].target,
                  cfg.line[i].fast, cfg.line[i].slow, cfg.line[i].overrun, ROUTE[i]);
  Serial.printf("order %d%d%d  band %.1f %% (min %.0f mL)  learn %d  red hold-up %.1f mL\n", cfg.order[0] + 1, cfg.order[1] + 1,
                cfg.order[2] + 1, cfg.bandPct, cfg.bandMinMl, cfg.learn, cfg.redHoldupMl);
  Serial.printf("pause after Step 1 %lu s  ", (unsigned long)cfg.pauseS);
  Serial.printf("dT target %.1f C (+%.1f margin, %u readings)  timeout %lu min  loop P4 %u%%  check %.1f C drop in %lu s\n",
                cfg.dtTarget, cfg.dtMargin, cfg.dtConfirm, (unsigned long)cfg.dtTimeoutMin, cfg.loopDuty, cfg.loopMinDrop,
                (unsigned long)cfg.loopCheckS);
  Serial.printf("jug ~%.0f mL (max %.0f)  mixing tank ~%.0f mL (max %.0f)\n", jugMl, cfg.jugMaxMl, mixMl, cfg.mixMaxMl);
}
static void printTemps()
{
  if (tempFrames == 0)
  {
    Serial.println(F("temps: nothing from U7 yet"));
    return;
  }
  Serial.printf("TS1 (yellow, cooled) %.2f C  TS2 (reference) %.2f C  dT %.2f C  loop P4 %s  age %lu ms\n", tempC[0], tempC[1],
                deltaT(), pumpPct[P_LOOP] ? "ON" : "off", (unsigned long)(millis() - lastTempMs));
  Serial.printf("  raw %.2f / %.2f C, offsets %+.2f / %+.2f C, bad readings so far %lu\n", tempRaw[0], tempRaw[1], cfg.tsOffset[0],
                cfg.tsOffset[1], (unsigned long)badTempFrames);
}
static void printStatus()
{
  Serial.printf("link %s (ok %lu, bad %lu)  counts %lu %lu %lu  start=%s\n", linkOk() ? "OK" : "DOWN",
                (unsigned long)framesOk, (unsigned long)framesBad, (unsigned long)counts[0], (unsigned long)counts[1],
                (unsigned long)counts[2], digitalRead(START_PIN) ? "up" : "PRESSED");
  Serial.print(F("valves "));
  for (int v = 0; v < NSV; v++)
    Serial.printf("SV%d=%d ", v + 1, valveOpen[v]);
  Serial.print(F(" pumps "));
  for (int p = 0; p < 4; p++)
    Serial.printf("P%d=%u%% ", p + 1, pumpPct[p]);
  Serial.printf("\nfloats FSW1-4 %d%d%d%d FSW5 %d FSW6 %d\n", digitalRead(FSW_PIN[0]), digitalRead(FSW_PIN[1]),
                digitalRead(FSW_PIN[2]), digitalRead(FSW_PIN[3]), fswU7 & 1, (fswU7 >> 1) & 1);
  printTemps();
  Serial.printf("sequence %s  dose %s%s%s\n", SEQ_NAME[seq], PHASE_NAME[dose.ph], tripWhy[0] ? "  last trip: " : "", tripWhy);
}
static void help()
{
  Serial.println(F(
      "--- Team 13 Chocolate Day 2 controller, ESP32 #1 (U2) ---\n"
      "lines: 1=RED 2=YELLOW 3=GREEN 4=WATER (water runs SV4 > TA > P1 > FT1 > jug in the mixing tank)\n"
      "valves: SV1 red source  SV2 yellow  SV3 green  SV4 water source  SV5 cooling loop\n"
      "start | abort          run Step 1 flush > Step 2 dT (cooling) > Step 3 dose (same as START switch) | trip everything\n"
      "status | show | temps  live I/O | settings | temperatures\n"
      "prime <1-4> <sec>      fill a line at fast duty (1 and 4 counted as jug)\n"
      "calrun <1-4> <sec> [%] timed run, counts pulses\n"
      "k <1-4> <mL>           set k = pulses of last calrun/dose / mL weighed (adds a calibration row)\n"
      "cal                    print the calibration table (flow k and probe offsets)\n"
      "tcal ice | match | clear  probe calibration: both probes in stirred ice water | in the same water | reset\n"
      "dose <1-4> <mL> [jug]  metered dose (default counted into the mixing tank)\n"
      "loop <0|1>             cooling loop P4 + SV5 by hand, idle only\n"
      "tlog <0|1>             log temperatures every 2 s while idle\n"
      "report                 print the stored data of the last run (flush, cal, dT trend, doses)\n"
      "jug <mL> | mix <mL>    set the jug / mixing tank volume count (0 after emptying)\n"
      "valve <1-5> <0|1>      manual valve, idle only\n"
      "pump <1-4> <%>         manual pump, idle only, interlocked\n"
      "off                    everything off\n"
      "set <key> <value>      k1..4 tgt1..4 fast1..4 slow1..4 ovr1..4 order(e.g. 231) band bandmin learn\n"
      "                       dt dtmargin dtconfirm dttimeout loopduty loopcheck loopdrop mixmax jugmax ts1off ts2off holdup pause\n"
      "save | load | defaults | code"));
}

static void handleCommand(String line)
{
  line.trim();
  if (!line.length())
    return;
  char buf[96];
  line.toCharArray(buf, sizeof(buf));
  char *cmd = strtok(buf, " ");
  char *a1 = strtok(nullptr, " ");
  char *a2 = strtok(nullptr, " ");
  char *a3 = strtok(nullptr, " ");
  String c(cmd);
  c.toLowerCase();
  int n = a1 ? atoi(a1) : 0;
  bool busy = running();

  if (c == "help" || c == "?")
    help();
  else if (c == "status")
    printStatus();
  else if (c == "show")
    printConfig();
  else if (c == "temps")
    printTemps();
  else if (c == "cal")
    printCal();
  else if (c == "report")
    printReport();
  else if (c == "tcal" && !busy)
    tempCal(a1 ? a1 : "");
  else if (c == "start")
    runStart();
  else if (c == "abort" || c == "stop")
    trip("abort command");
  else if (c == "off")
  {
    allOff();
    dose.ph = D_IDLE;
    seq = S_IDLE;
    runLogging = false;
    Serial.println(F("all off"));
  }
  else if (c == "tlog" && a1)
  {
    tlogIdle = n != 0;
    Serial.printf("idle temperature log %s\n", tlogIdle ? "on" : "off");
  }
  else if (busy)
    Serial.println(F("! busy, type abort first"));
  else if (c == "prime" && n >= 1 && n <= 4 && a2)
    doseBegin(n - 1, 0, n == 1 || n == 4, true, (uint32_t)(atof(a2) * 1000), cfg.line[n - 1].fast);
  else if (c == "calrun" && n >= 1 && n <= 4 && a2)
  {
    uint8_t duty = a3 ? atoi(a3) : cfg.line[n - 1].fast;
    doseBegin(n - 1, 0, n == 1 || n == 4, true, (uint32_t)(atof(a2) * 1000), duty);
  }
  else if (c == "k" && n >= 1 && n <= 4 && a2)
  {
    float ml = atof(a2);
    if (ml <= 0 || lastPulses[n - 1] == 0)
      Serial.println(F("! need a calrun or dose on that line first, and mL > 0"));
    else
    {
      cfg.line[n - 1].k = lastPulses[n - 1] / ml;
      if (nC < MAX_C)
        clog[nC++] = {(uint8_t)(n - 1), lastPulses[n - 1], lastRunMs[n - 1], ml, cfg.line[n - 1].k};
      Serial.printf("C,%d,%s,%lu,%lu,%.1f,%.4f\n", n, LD[n - 1].name, (unsigned long)lastPulses[n - 1],
                    (unsigned long)lastRunMs[n - 1], ml, cfg.line[n - 1].k);
      Serial.printf("k%d = %lu / %.1f = %.4f pulses/mL (type code and paste TUNE_K to keep it)\n", n,
                    (unsigned long)lastPulses[n - 1], ml, cfg.line[n - 1].k);
    }
  }
  else if (c == "dose" && n >= 1 && n <= 4 && a2)
  {
    bool jug = a3 && String(a3) == "jug";
    doseBegin(n - 1, atof(a2), jug, false, 0, 0);
  }
  else if (c == "loop" && a1)
  {
    if (n)
    {
      if (loopStart())
        Serial.println(F("cooling loop ON (SV5 open, P4 running). No flow sensor on this loop: watch the coil outlet."));
    }
    else
    {
      loopStop();
      Serial.println(F("cooling loop off"));
    }
  }
  else if (c == "jug" && a1)
  {
    jugMl = atof(a1);
    Serial.printf("jug ~%.0f mL\n", jugMl);
  }
  else if (c == "mix" && a1)
  {
    mixMl = atof(a1);
    Serial.printf("mixing tank ~%.0f mL\n", mixMl);
  }
  else if (c == "valve" && n >= 1 && n <= NSV && a2)
  {
    setValve(n - 1, atoi(a2));
    printStatus();
  }
  else if (c == "pump" && n >= 1 && n <= 4 && a2)
  {
    setPump(n - 1, atoi(a2));
    printStatus();
  }
  else if (c == "save")
    saveConfig();
  else if (c == "code")
    printCode();
  else if (c == "load")
  {
    loadConfig();
    printConfig();
  }
  else if (c == "defaults")
  {
    setDefaults();
    printConfig();
    Serial.println(F("(not saved yet)"));
  }
  else if (c == "set" && a1 && a2)
  {
    String k(a1);
    k.toLowerCase();
    float v = atof(a2);
    int i = k.length() > 0 ? k.charAt(k.length() - 1) - '1' : -1;
    String base = k.substring(0, k.length() - 1);
    bool ok = true;
    if (i >= 0 && i < NLINES && base == "k")
      cfg.line[i].k = v;
    else if (i >= 0 && i < NLINES && base == "tgt")
      cfg.line[i].target = v;
    else if (i >= 0 && i < NLINES && base == "fast")
      cfg.line[i].fast = constrain((int)v, 0, 100);
    else if (i >= 0 && i < NLINES && base == "slow")
      cfg.line[i].slow = constrain((int)v, 0, 100);
    else if (i >= 0 && i < NLINES && base == "ovr")
      cfg.line[i].overrun = v;
    else if (k == "order" && strlen(a2) == 3)
    {
      uint8_t o[3];
      bool seen[3] = {false, false, false};
      for (int j = 0; j < 3; j++)
      {
        o[j] = a2[j] - '1';
        if (o[j] > 2 || seen[o[j]])
          ok = false;
        else
          seen[o[j]] = true;
      }
      if (ok)
        memcpy(cfg.order, o, 3);
    }
    else if (k == "band")
      cfg.bandPct = v;
    else if (k == "bandmin")
      cfg.bandMinMl = v;
    else if (k == "learn")
      cfg.learn = v != 0;
    else if (k == "dt")
      cfg.dtTarget = v;
    else if (k == "dtmargin")
      cfg.dtMargin = v;
    else if (k == "dtconfirm")
      cfg.dtConfirm = constrain((int)v, 1, 50);
    else if (k == "dttimeout")
      cfg.dtTimeoutMin = (uint32_t)v;
    else if (k == "loopduty")
      cfg.loopDuty = constrain((int)v, 0, 100);
    else if (k == "loopcheck")
      cfg.loopCheckS = (uint32_t)v;
    else if (k == "loopdrop")
      cfg.loopMinDrop = v;
    else if (k == "mixmax")
      cfg.mixMaxMl = v;
    else if (k == "jugmax")
      cfg.jugMaxMl = v;
    else if (k == "ts1off")
      cfg.tsOffset[0] = v;
    else if (k == "ts2off")
      cfg.tsOffset[1] = v;
    else if (k == "pause")
      cfg.pauseS = constrain((int)v, 0, 3600);
    else if (k == "holdup")
      cfg.redHoldupMl = constrain(v, 0.0f, 200.0f);
    else
      ok = false;
    if (ok)
    {
      printConfig();
      Serial.println(F("(type code to get the TUNING block)"));
    }
    else
      Serial.println(F("! unknown key or bad value"));
  }
  else
    Serial.println(F("? type help"));
}

void setup()
{
  for (int v = 0; v < NSV; v++)
  {
    pinMode(SV_PIN[v], OUTPUT);
    digitalWrite(SV_PIN[v], LOW);
  }
  for (int p = 0; p < 4; p++)
  {
    ledcSetup(p, PWM_FREQ, PWM_BITS);
    ledcAttachPin(PWM_PIN[p], p);
    ledcWrite(p, 0);
  }
  for (int i = 0; i < 4; i++)
    pinMode(FSW_PIN[i], INPUT);
  pinMode(START_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  Serial2.setRxBufferSize(1024);
  Serial2.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  loadConfig();
  delay(300);
  Serial.println(F("\n[U2 controller] Team 13 Chocolate, Day 2: Flush > dT 10 C (cooling) > Dose. Type help.\nEmpty jug in the mixing tank under the red outlet, then press START."));
  printConfig();
}

void loop()
{
  pollLink();
  pollStart();
  interlocks();
  doseStep();
  seqStep();
  logTemp(false);

  static String rx;
  while (Serial.available())
  {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r')
    {
      if (rx.length())
      {
        pollLink();
        handleCommand(rx);
        rx = "";
      }
    }
    else if (rx.length() < 90)
    {
      rx += ch;
    }
  }
}
