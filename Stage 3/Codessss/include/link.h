// Shared definitions for the UART link between
// ESP32 #2 (sensor, U7) and ESP32 #1 (controller, U2).

#include <Arduino.h>

static const uint32_t LINK_BAUD = 115200;
static const uint8_t LINK_RX_PIN = 16;     // RX on G16
static const uint8_t LINK_TX_PIN = 17;     // TX on G17
static const uint32_t LINK_PERIOD_MS = 10; // 100 fps

inline uint8_t linkChecksum(const char *s, size_t n)
{
  uint8_t x = 0;
  for (size_t i = 0; i < n; i++)
    x ^= (uint8_t)s[i];
  return x;
}
