#pragma once
#include <Arduino.h>

static const uint32_t LINK_BAUD      = 115200;
static const uint8_t  LINK_RX_PIN    = 16;
static const uint8_t  LINK_TX_PIN    = 17;
static const uint32_t LINK_PERIOD_MS = 10;
static const int32_t  TEMP_INVALID   = -12700;

inline uint8_t linkChecksum(const char *s, size_t n) {
  uint8_t x = 0;
  for (size_t i = 0; i < n; i++) x ^= (uint8_t)s[i];
  return x;
}

inline int linkSeal(char *buf, size_t cap, int n) {
  uint8_t x = linkChecksum(buf, n);
  return n + snprintf(buf + n, cap - n, "%02X\n", x);
}

inline bool linkCheck(const char *buf) {
  const char *star = strchr(buf, '*');
  if (!star) return false;
  return (uint8_t)strtoul(star + 1, nullptr, 16) == linkChecksum(buf, star - buf + 1);
}
