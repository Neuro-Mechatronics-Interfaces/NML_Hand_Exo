#pragma once
/**
 * OLED (SSD1306) minimal, non-blocking display driver for exo state text.
 * 
 * Requirements:
 *  - Adafruit_GFX
 *  - Adafruit_SSD1306
 *  - Wire (I2C)
 * 
 * Configurable values are read from config.h:
 *   OLED_ENABLED_DEFAULT
 *   OLED_I2C_ADDR_PRIMARY
 *   OLED_I2C_ADDR_ALT
 *   OLED_SCREEN_WIDTH
 *   OLED_SCREEN_HEIGHT
 *   OLED_CENTER_TEXT
 *   OLED_UPDATE_PERIOD_MS
 */

#include <Arduino.h>
#include "config.h"

// Public exo states to render on the display.
// Extend as needed for your application.
enum ExoState : uint8_t {
  EXO_READY = 0,
  EXO_INDEX_PINCH,
  EXO_MIDDLE_PINCH,
  EXO_RING_PINCH,
  EXO_KEYGRIP_OPEN,
  EXO_KEYGRIP_CLOSE,
  EXO_GRASP_OPEN,
  EXO_GRASP_CLOSE,
  EXO_BT_CONNECTED,
  EXO_BT_DISCONNECTED
};

/** Initialize the OLED safely (fast-fail). Returns true if enabled & ready. */
bool oledInit();

/** Enable/disable the OLED feature at runtime. If enabling, attempts init. */
void oledSetEnabled(bool enabled);

/** Query whether the OLED feature is currently enabled. */
bool oledEnabled();

/** Non-blocking tick to push text updates (call once per loop()). */
void oledTick();

/** Set the current exo state. The text is drawn on the next oledTick(). */
void oledSetState(ExoState s);

/** Get the currently requested exo state. */
ExoState oledGetState();

/** Optional: get the displayable PROGMEM text for a given state. */
const __FlashStringHelper* oledGetStateText(ExoState s);

/** Run the startup animation sequence (blocking for ~3 seconds). */
void oledStartupAnimation();
