// ESP32 #2 (U7) · sensor board
// Counts pulses from flow sensors FS1..FS3 and streams the running totals to ESP32 #1 over UART.
// It never drives anything, so it is safe to leave running all the time.
//
// USB serial commands (115200):
//   show      print the three running totals once
//   watch     toggle a 2 Hz print of totals and pulse rates (handy when wiring/priming)

#include <Arduino.h>
#include "link.h"

// PCB sheet 3: FLOW_SENSOR1..3 signal pins on ESP32 #2. (FS4 = G14, FS5 = G27 are unused.)
static const uint8_t FS_PIN[3] = {33, 25, 26};

static volatile uint32_t pulses[3] = {0, 0, 0};
static volatile uint32_t lastEdgeUs[3] = {0, 0, 0};
// Ignore edges closer than this (contact bounce / noise). 300 us = 3.3 kHz, far above any real flow.
static const uint32_t MIN_EDGE_US = 300;

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

static uint32_t seq = 0;
static bool watch = false;

void setup() {
  Serial.begin(115200);
  Serial2.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  for (int i = 0; i < 3; i++) pinMode(FS_PIN[i], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FS_PIN[0]), isr0, FALLING);
  attachInterrupt(digitalPinToInterrupt(FS_PIN[1]), isr1, FALLING);
  attachInterrupt(digitalPinToInterrupt(FS_PIN[2]), isr2, FALLING);
  Serial.println(F("\n[U7 sensor] FS1=G33 FS2=G25 FS3=G26 -> UART G17 @115200. Commands: show, watch"));
}

void loop() {
  static uint32_t nextFrame = 0, nextWatch = 0;
  static uint32_t prevWatch[3] = {0, 0, 0};
  uint32_t now = millis();

  if ((int32_t)(now - nextFrame) >= 0) {
    nextFrame = now + LINK_PERIOD_MS;
    uint32_t c[3];
    snapshot(c);
    char buf[64];
    int n = snprintf(buf, sizeof(buf), "F,%lu,%lu,%lu,%lu*", (unsigned long)seq++,
                     (unsigned long)c[0], (unsigned long)c[1], (unsigned long)c[2]);
    uint8_t x = linkChecksum(buf, n);
    snprintf(buf + n, sizeof(buf) - n, "%02X\n", x);
    Serial2.print(buf);
  }

  if (watch && (int32_t)(now - nextWatch) >= 0) {
    nextWatch = now + 500;
    uint32_t c[3];
    snapshot(c);
    Serial.printf("FS1 %lu (%lu Hz)  FS2 %lu (%lu Hz)  FS3 %lu (%lu Hz)\n",
                  (unsigned long)c[0], (unsigned long)((c[0] - prevWatch[0]) * 2),
                  (unsigned long)c[1], (unsigned long)((c[1] - prevWatch[1]) * 2),
                  (unsigned long)c[2], (unsigned long)((c[2] - prevWatch[2]) * 2));
    for (int i = 0; i < 3; i++) prevWatch[i] = c[i];
  }

  // Non-blocking command reader: frames to U2 keep flowing every 10 ms while you type.
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
      Serial.printf("FS1 %lu  FS2 %lu  FS3 %lu\n", (unsigned long)c[0], (unsigned long)c[1], (unsigned long)c[2]);
    } else if (cmd == "watch") {
      watch = !watch;
      Serial.printf("watch %s\n", watch ? "on" : "off");
    } else if (cmd.length()) {
      Serial.println(F("commands: show, watch"));
    }
  }
}
