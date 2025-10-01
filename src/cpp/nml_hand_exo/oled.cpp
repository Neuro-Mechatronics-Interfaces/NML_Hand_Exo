#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "config.h"
#include "oled.h"

// ---- Internal state ----
static bool sEnabled = OLED_ENABLED_DEFAULT;
static uint8_t sAddr = 0;

static Adafruit_SSD1306 sDisplay(OLED_SCREEN_WIDTH, OLED_SCREEN_HEIGHT, &Wire, -1);

static ExoState sState = EXO_READY;
static ExoState sLastPainted = static_cast<ExoState>(0xFF);
static uint32_t sNextAllowedMs = 0;

// ---- Helpers ----
static bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

static uint8_t detectOledAddr() {
  if (i2cPresent(OLED_I2C_ADDR_PRIMARY)) return OLED_I2C_ADDR_PRIMARY;
  if (i2cPresent(OLED_I2C_ADDR_ALT))     return OLED_I2C_ADDR_ALT;
  return 0;
}

const __FlashStringHelper* oledGetStateText(ExoState s) {
  switch (s) {
    case EXO_READY:         return F("ready");
    case EXO_INDEX_PINCH:   return F("index pinch");
    case EXO_MIDDLE_PINCH:  return F("middle pinch");
    case EXO_RING_PINCH:    return F("ring pinch");
    case EXO_KEYGRIP_OPEN:  return F("keygrip open");
    case EXO_KEYGRIP_CLOSE: return F("keygrip close");
    case EXO_GRASP_OPEN:    return F("grasp open");
    case EXO_GRASP_CLOSE:   return F("grasp close");
    default:                return F("unknown");
  }
}

static void oledCenterPrint(const String& text) {
  sDisplay.clearDisplay();
#if OLED_CENTER_TEXT
  int16_t x1, y1; uint16_t w, h;
  sDisplay.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  int16_t x = (OLED_SCREEN_WIDTH  - (int)w) / 2;
  int16_t y = (OLED_SCREEN_HEIGHT - (int)h) / 2;
  if (x < 0) x = 0; if (y < 0) y = 0;
  sDisplay.setCursor(x, y);
#else
  sDisplay.setCursor(0, 0);
#endif
  sDisplay.print(text);
  sDisplay.display(); // single I2C burst
}

// ---- Public API ----
bool oledInit() {
  if (!sEnabled) return false;

  Wire.begin();
//#if defined(WIRE_HAS_TIMEOUT) || defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_ESP32)
//  Wire.setWireTimeout(2000, true);
//#endif
  Wire.setClock(400000);

  sAddr = detectOledAddr();
  if (sAddr == 0) {
    // Not present: disable to avoid blocking the app.
    sEnabled = false;
    return false;
  }

  const uint32_t t0 = millis();
  const uint32_t MAX_INIT_MS = 150;
  if (!sDisplay.begin(SSD1306_SWITCHCAPVCC, sAddr)) {
    sEnabled = false;
    return false;
  }
  if ((millis() - t0) > MAX_INIT_MS) {
    // Defensive: if init was slow, disable to protect ISR latency.
    sEnabled = false;
    return false;
  }

  sDisplay.clearDisplay();
  // 2x is readable on both 128x32 and 128x64. Tune if desired.
  sDisplay.setTextSize(2);
  sDisplay.setTextColor(SSD1306_WHITE);
  sDisplay.display();

  sLastPainted = static_cast<ExoState>(0xFF); // force first paint
  sNextAllowedMs = 0;
  return true;
}

void oledSetEnabled(bool enabled) {
  if (enabled == sEnabled) return;
  sEnabled = enabled;
  if (sEnabled) {
    if (!oledInit()) {
      // If init fails, remain disabled
      sEnabled = false;
    }
  }
}

bool oledEnabled() {
  return sEnabled;
}

void oledSetState(ExoState s) {
  sState = s;
  // Do not paint here; oledTick() will push on its schedule.
}

ExoState oledGetState() {
  return sState;
}

void oledTick() {
  if (!sEnabled) return;
  const uint32_t now = millis();
  if (now < sNextAllowedMs) return;

  if (sState != sLastPainted) {
    // Build a temporary RAM string for measuring/printing
    String txt = String(oledGetStateText(sState));
    oledCenterPrint(txt);
    sLastPainted = sState;
  }

  sNextAllowedMs = now + OLED_UPDATE_PERIOD_MS;
}
