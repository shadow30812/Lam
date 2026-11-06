#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "link.h"

static const uint8_t FS_PIN[3] = {33, 25, 26};
static const uint8_t TS_PIN[2] = {32, 13};
static const uint8_t FSW_PIN[2] = {34, 35};
static const uint8_t H1_PIN = 23;
static const uint32_t TEMP_CONV_MS = 800;
static const uint8_t TS_RESOLUTION = 12;
static const uint32_t MIN_EDGE_US = 300;

static volatile uint32_t pulses[3] = {0, 0, 0};
static volatile uint32_t lastEdgeUs[3] = {0, 0, 0};

static void IRAM_ATTR countEdge(uint8_t i) {
  uint32_t now = micros();
  if (now - lastEdgeUs[i] >= MIN_EDGE_US) {
    pulses[i]++;
    lastEdgeUs[i] = now;
  }
}
static void IRAM_ATTR isr0() { countEdge(0); }
static void IRAM_ATTR isr1() { countEdge(1); }
static void IRAM_ATTR isr2() { countEdge(2); }

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static void snapshot(uint32_t out[3]) {
  portENTER_CRITICAL(&mux);
  for (int i = 0; i < 3; i++) out[i] = pulses[i];
  portEXIT_CRITICAL(&mux);
}

static OneWire bus1(TS_PIN[0]);
static OneWire bus2(TS_PIN[1]);
static DallasTemperature ds1(&bus1);
static DallasTemperature ds2(&bus2);
static int found1 = 0, found2 = 0;
static DeviceAddress addr[2];
static bool have[2] = {false, false};
static DallasTemperature *busOf[2] = {&ds1, &ds2};
static float tempC[2] = {NAN, NAN};
static uint32_t tempSeq = 0;


static uint32_t seq = 0;
static bool watch = false;

static bool validT(float t) { return !isnan(t) && t > -55.0f && t < 125.0f && t != 85.0f; }

static void scanSensors() {
  ds1.begin();
  ds2.begin();
  found1 = ds1.getDeviceCount();
  found2 = ds2.getDeviceCount();
  have[0] = found1 > 0 && ds1.getAddress(addr[0], 0);
  busOf[0] = &ds1;
  if (found2 > 0) {
    have[1] = ds2.getAddress(addr[1], 0);
    busOf[1] = &ds2;
  } else {
    have[1] = found1 > 1 && ds1.getAddress(addr[1], 1);
    busOf[1] = &ds1;
  }
  ds1.setResolution(TS_RESOLUTION);
  ds2.setResolution(TS_RESOLUTION);
  ds1.setWaitForConversion(false);
  ds2.setWaitForConversion(false);
  ds1.requestTemperatures();
  ds2.requestTemperatures();
}

static const char *familyName(uint8_t f) {
  switch (f) {
    case 0x28: return "DS18B20";
    case 0x10: return "DS18S20";
    case 0x22: return "DS1822";
    case 0x3B: return "MAX31850";
    default: return "unknown";
  }
}

static void printProbes() {
  for (int i = 0; i < 2; i++) {
    Serial.printf("TS%d: ", i + 1);
    if (!have[i]) {
      Serial.println(F("no sensor found (check red VDD, black GND, brown DATA and the 4.7k pull-up)"));
      continue;
    }
    for (int b = 0; b < 8; b++) Serial.printf("%02X", addr[i][b]);
    bool crcOk = OneWire::crc8(addr[i], 7) == addr[i][7];
    Serial.printf("  family 0x%02X = %s  CRC %s  bus %s  parasite %s\n", addr[i][0], familyName(addr[i][0]),
                  crcOk ? "ok" : "BAD", busOf[i] == &ds1 ? "G32" : "G13", busOf[i]->isParasitePowerMode() ? "yes" : "no");
  }
}

static void readTemps() {
  float a = have[0] ? busOf[0]->getTempC(addr[0]) : NAN;
  float b = have[1] ? busOf[1]->getTempC(addr[1]) : NAN;
  tempC[0] = validT(a) ? a : NAN;
  tempC[1] = validT(b) ? b : NAN;
  tempSeq++;
  ds1.requestTemperatures();
  ds2.requestTemperatures();
}

static void sendTemps() {
  char buf[80];
  long a = validT(tempC[0]) ? lroundf(tempC[0] * 100) : TEMP_INVALID;
  long b = validT(tempC[1]) ? lroundf(tempC[1] * 100) : TEMP_INVALID;
  int n = snprintf(buf, sizeof(buf), "T,%lu,%ld,%ld,0*", (unsigned long)tempSeq, a, b);
  linkSeal(buf, sizeof(buf), n);
  Serial2.print(buf);
}

static void printTemps() {
  Serial.printf("TS1 (G32, %d found) %s  TS2 (G13, %d found) %s  dT %s\n", found1,
                validT(tempC[0]) ? String(tempC[0], 2).c_str() : "--", found2,
                validT(tempC[1]) ? String(tempC[1], 2).c_str() : "--",
                validT(tempC[0]) && validT(tempC[1]) ? String(fabsf(tempC[0] - tempC[1]), 2).c_str() : "--");
}

void setup() {
  pinMode(H1_PIN, OUTPUT);
  digitalWrite(H1_PIN, LOW);
  Serial.begin(115200);
  Serial2.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  for (int i = 0; i < 3; i++) pinMode(FS_PIN[i], INPUT_PULLUP);
  for (int i = 0; i < 2; i++) pinMode(FSW_PIN[i], INPUT);
  attachInterrupt(digitalPinToInterrupt(FS_PIN[0]), isr0, FALLING);
  attachInterrupt(digitalPinToInterrupt(FS_PIN[1]), isr1, FALLING);
  attachInterrupt(digitalPinToInterrupt(FS_PIN[2]), isr2, FALLING);
  scanSensors();
  Serial.printf("\n[U7 sensor, Day 2] FS1=G33 FS2=G25 FS3=G26  TS1=G32 (%d on bus)  TS2=G13 (%d on bus)\n", found1, found2);
  printProbes();
  Serial.println(F("Commands: show, watch, temps, scan"));
}

void loop() {
  static uint32_t nextFrame = 0, nextWatch = 0, nextTemp = TEMP_CONV_MS;
  static uint32_t prevWatch[3] = {0, 0, 0};
  uint32_t now = millis();

  if ((int32_t)(now - nextTemp) >= 0) {
    nextTemp = now + TEMP_CONV_MS;
    readTemps();
    sendTemps();
  }

  if ((int32_t)(now - nextFrame) >= 0) {
    nextFrame = now + LINK_PERIOD_MS;
    uint32_t c[3];
    snapshot(c);
    int fsw = (digitalRead(FSW_PIN[0]) ? 1 : 0) | (digitalRead(FSW_PIN[1]) ? 2 : 0);
    char buf[80];
    int n = snprintf(buf, sizeof(buf), "F,%lu,%lu,%lu,%lu,%d*", (unsigned long)seq++,
                     (unsigned long)c[0], (unsigned long)c[1], (unsigned long)c[2], fsw);
    linkSeal(buf, sizeof(buf), n);
    Serial2.print(buf);
  }

  if (watch && (int32_t)(now - nextWatch) >= 0) {
    nextWatch = now + 500;
    uint32_t c[3];
    snapshot(c);
    Serial.printf("FS1 %lu (%lu Hz)  FS2 %lu (%lu Hz)  FS3 %lu (%lu Hz)  ",
                  (unsigned long)c[0], (unsigned long)((c[0] - prevWatch[0]) * 2),
                  (unsigned long)c[1], (unsigned long)((c[1] - prevWatch[1]) * 2),
                  (unsigned long)c[2], (unsigned long)((c[2] - prevWatch[2]) * 2));
    printTemps();
    for (int i = 0; i < 3; i++) prevWatch[i] = c[i];
  }

  static String rx;
  bool gotLine = false;
  while (Serial.available() && !gotLine) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') gotLine = rx.length() > 0;
    else if (rx.length() < 40) rx += ch;
  }
  if (gotLine) {
    String cmd = rx;
    rx = "";
    cmd.trim();
    if (cmd == "show") {
      uint32_t c[3];
      snapshot(c);
      Serial.printf("FS1 %lu  FS2 %lu  FS3 %lu  FSW5 %d  FSW6 %d\n", (unsigned long)c[0], (unsigned long)c[1],
                    (unsigned long)c[2], digitalRead(FSW_PIN[0]), digitalRead(FSW_PIN[1]));
    } else if (cmd == "watch") {
      watch = !watch;
      Serial.printf("watch %s\n", watch ? "on" : "off");
    } else if (cmd == "temps") {
      printTemps();
    } else if (cmd == "scan") {
      scanSensors();
      Serial.printf("TS1 bus: %d sensor(s), TS2 bus: %d sensor(s)\n", found1, found2);
      printProbes();
    } else if (cmd.length()) {
      Serial.println(F("commands: show, watch, temps, scan"));
    }
  }
}
