// Shared definitions for the UART link between ESP32 #2 (sensor, U7) and ESP32 #1 (controller, U2).
//
// U7 sends one line every LINK_PERIOD_MS:
//     F,<seq>,<count1>,<count2>,<count3>*<xx>\n
// count1..3 are the running pulse totals of FS1..FS3 since U7 booted (uint32, they never reset).
// <xx> is the XOR of every character between 'F' and '*' inclusive of 'F', as two hex digits.
// U2 keeps its own baseline, so a lost frame never loses pulses: the next frame carries the total.
#pragma once
#include <Arduino.h>

static const uint32_t LINK_BAUD      = 115200;
static const uint8_t  LINK_RX_PIN    = 16;   // both boards: RX on G16
static const uint8_t  LINK_TX_PIN    = 17;   // both boards: TX on G17
static const uint32_t LINK_PERIOD_MS = 10;   // 100 frames per second

inline uint8_t linkChecksum(const char *s, size_t n) {
  uint8_t x = 0;
  for (size_t i = 0; i < n; i++) x ^= (uint8_t)s[i];
  return x;
}
