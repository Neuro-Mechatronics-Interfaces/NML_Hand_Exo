/*******************************************************************************
 * Minimal OpenRB-150 Dynamixel motor console.
 *
 * Purpose:
 *   A tiny, read-only sketch for confirming motor communication and reading
 *   telemetry without the NML exo firmware, GUI, OLED, IMU, sync-read tests, or
 *   any motion/torque commands.
 *
 * Arduino Serial Monitor:
 *   Baud: 1000000
 *   Line ending: Newline
 *
 * Commands:
 *   help
 *   baud 1000000
 *   protocol 2
 *   ids 11 12 13 14 16 17 18 19
 *   ping
 *   cache
 *   fw
 *   rd
 *   srl
 *   setrd
 *   pos
 *   posbench 50
 *   cur
 *   vel
 *   syncpos
 *   fastsyncpos
 *   scan
 ******************************************************************************/

#include <Dynamixel2Arduino.h>

// Match ROBOTIS OpenRB examples.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8);
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2;
#elif defined(ARDUINO_SAM_DUE)
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2;
#elif defined(ARDUINO_SAM_ZERO)
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2;
#elif defined(ARDUINO_OpenCM904)
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 22;
#elif defined(ARDUINO_OpenCR)
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 84;
#elif defined(ARDUINO_OpenRB)
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2;
#endif

using namespace ControlTableItem;

constexpr long DEBUG_BAUD = 1000000;
constexpr uint8_t MAX_IDS = 18;
constexpr uint32_t READ_TIMEOUT_MS = 3;
constexpr uint16_t USER_PKT_BUF_CAP = 128;
constexpr uint16_t DXL_RX_DRAIN_MAX_BYTES = 128;
constexpr uint32_t DXL_RX_DRAIN_MAX_US = 1000;
constexpr uint8_t MAX_CONSECUTIVE_READ_ERRORS = 2;
constexpr uint16_t XC330_T288_MODEL_NUMBER = 1220;

uint32_t g_read_timeout_ms = 3;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

float g_protocol = 2.0;
long g_baud = 1000000;
uint8_t g_ids[MAX_IDS] = {11, 12, 13, 14, 16, 17, 18, 19};
uint8_t g_id_count = 8;
uint8_t g_user_pkt_buf[USER_PKT_BUF_CAP];
int32_t g_sync_positions[MAX_IDS];
DYNAMIXEL::InfoSyncReadInst_t g_sync_read_info;
DYNAMIXEL::XELInfoSyncRead_t g_sync_read_xels[MAX_IDS];

uint16_t clearDxlRx() {
  uint16_t drained = 0;
  uint32_t start = micros();
  while (DXL_SERIAL.available() > 0 &&
         drained < DXL_RX_DRAIN_MAX_BYTES &&
         (micros() - start) < DXL_RX_DRAIN_MAX_US) {
    DXL_SERIAL.read();
    ++drained;
  }
  return drained;
}

void openDxlPort() {
  // This order intentionally matches the working ROBOTIS scan example.
  dxl.setPortProtocolVersion(g_protocol);
  dxl.begin(g_baud);
}

void printHelp() {
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("OpenRB motor console - read-only");
  DEBUG_SERIAL.println("Commands:");
  DEBUG_SERIAL.println("  help");
  DEBUG_SERIAL.println("  protocol 1|2");
  DEBUG_SERIAL.println("  baud 57600|115200|1000000|2000000|3000000");
  DEBUG_SERIAL.println("  timeout MS (set read timeout, default 3)");
  DEBUG_SERIAL.println("  ids 11 12 13 14 16 17 18 19");
  DEBUG_SERIAL.println("  ping");
  DEBUG_SERIAL.println("  cache      (cache XC330 model number 1220 for selected IDs)");
  DEBUG_SERIAL.println("  fw ID      (read FIRMWARE_VERSION; X330 fast read needs v46+)");
  DEBUG_SERIAL.println("  rd ID      (read RETURN_DELAY_TIME; units are 2 us)");
  DEBUG_SERIAL.println("  srl ID     (read STATUS_RETURN_LEVEL)");
  DEBUG_SERIAL.println("  setrd ID VALUE (write RETURN_DELAY_TIME, 0-250; no motion command)");
  DEBUG_SERIAL.println("  setmotorbaud ID BAUD (EEPROM; torque off; one motor only)");
  DEBUG_SERIAL.println("  recover    (reopen DXL port and bounded-drain stale bytes)");
  DEBUG_SERIAL.println("  pos ID     (read PRESENT_POSITION only)");
  DEBUG_SERIAL.println("  posbench ID 50 5 (aggregate position-read timing; gap in ms)");
  DEBUG_SERIAL.println("  cur ID     (read PRESENT_CURRENT only)");
  DEBUG_SERIAL.println("  vel ID     (read PRESENT_VELOCITY only)");
  DEBUG_SERIAL.println("  syncpos    (official syncRead position-only test)");
  DEBUG_SERIAL.println("  fastsyncpos (official fastSyncRead position-only test)");
  DEBUG_SERIAL.println("  scan");
  DEBUG_SERIAL.println();
}

void printConfig() {
  DEBUG_SERIAL.print("CONFIG protocol=");
  DEBUG_SERIAL.print(g_protocol, 1);
  DEBUG_SERIAL.print(" baud=");
  DEBUG_SERIAL.print(g_baud);
  DEBUG_SERIAL.print(" ids:");
  for (uint8_t i = 0; i < g_id_count; ++i) {
    DEBUG_SERIAL.print(' ');
    DEBUG_SERIAL.print(g_ids[i]);
  }
  DEBUG_SERIAL.println();
}

void setProtocol(String line) {
  int space = line.indexOf(' ');
  if (space < 0) {
    printConfig();
    return;
  }
  int value = line.substring(space + 1).toInt();
  if (value != 1 && value != 2) {
    DEBUG_SERIAL.println("ERROR protocol must be 1 or 2");
    return;
  }
  g_protocol = (float)value;
  openDxlPort();
  printConfig();
}

void setBaud(String line) {
  int space = line.indexOf(' ');
  if (space < 0) {
    printConfig();
    return;
  }
  long value = line.substring(space + 1).toInt();
  if (value <= 0) {
    DEBUG_SERIAL.println("ERROR invalid baud");
    return;
  }
  g_baud = value;
  openDxlPort();
  printConfig();
}

void setReadTimeout(String line) {
  int space = line.indexOf(' ');
  if (space < 0) {
    DEBUG_SERIAL.print("TIMEOUT ms=");
    DEBUG_SERIAL.println(g_read_timeout_ms);
    return;
  }
  int value = line.substring(space + 1).toInt();
  if (value <= 0 || value > 1000) {
    DEBUG_SERIAL.println("ERROR timeout must be 1..1000 ms");
    return;
  }
  g_read_timeout_ms = (uint32_t)value;
  DEBUG_SERIAL.print("TIMEOUT ms=");
  DEBUG_SERIAL.println(g_read_timeout_ms);
}

void setIds(String line) {
  uint8_t parsed[MAX_IDS];
  uint8_t count = 0;
  int start = line.indexOf(' ');
  while (start >= 0 && count < MAX_IDS) {
    while (start < line.length() && line[start] == ' ') {
      ++start;
    }
    if (start >= line.length()) {
      break;
    }
    int end = line.indexOf(' ', start);
    String token = (end >= 0) ? line.substring(start, end) : line.substring(start);
    int value = token.toInt();
    if (value > 0 && value < 254) {
      parsed[count++] = (uint8_t)value;
    }
    start = end;
  }
  if (count == 0) {
    DEBUG_SERIAL.println("ERROR no valid IDs");
    return;
  }
  g_id_count = count;
  for (uint8_t i = 0; i < g_id_count; ++i) {
    g_ids[i] = parsed[i];
  }
  printConfig();
}

int parseCommandId(String line) {
  int space = line.indexOf(' ');
  if (space < 0) {
    return -1;
  }
  int id = line.substring(space + 1).toInt();
  return (id > 0 && id < 254) ? id : -1;
}

int parseSecondInt(String line, int fallback) {
  int first = line.indexOf(' ');
  if (first < 0) {
    return fallback;
  }
  int second = line.indexOf(' ', first + 1);
  if (second < 0) {
    return fallback;
  }
  int value = line.substring(second + 1).toInt();
  return value >= 0 ? value : fallback;
}

int parseThirdInt(String line, int fallback) {
  int first = line.indexOf(' ');
  if (first < 0) {
    return fallback;
  }
  int second = line.indexOf(' ', first + 1);
  if (second < 0) {
    return fallback;
  }
  int third = line.indexOf(' ', second + 1);
  if (third < 0) {
    return fallback;
  }
  int value = line.substring(third + 1).toInt();
  return value >= 0 ? value : fallback;
}

void runCacheModelNumbers() {
  printConfig();
  for (uint8_t i = 0; i < g_id_count; ++i) {
    uint8_t id = g_ids[i];
    bool ok = dxl.setModelNumber(id, XC330_T288_MODEL_NUMBER);
    DEBUG_SERIAL.print("CACHE id=");
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print(" model=");
    DEBUG_SERIAL.print(XC330_T288_MODEL_NUMBER);
    DEBUG_SERIAL.print(" ok=");
    DEBUG_SERIAL.println(ok ? 1 : 0);
  }
}

void runPing() {
  printConfig();
  for (uint8_t i = 0; i < g_id_count; ++i) {
    uint8_t id = g_ids[i];
    DYNAMIXEL::InfoFromPing_t info[1];
    memset(info, 0, sizeof(info));
    clearDxlRx();
    uint32_t start = micros();
    uint8_t received = dxl.ping(id, info, 1, g_read_timeout_ms);
    uint32_t elapsed = micros() - start;
    DEBUG_SERIAL.print("PING id=");
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print(" ok=");
    DEBUG_SERIAL.print(received > 0 ? 1 : 0);
    DEBUG_SERIAL.print(" elapsed_us=");
    DEBUG_SERIAL.print(elapsed);
    if (received > 0) {
      DEBUG_SERIAL.print(" model=");
      DEBUG_SERIAL.print(info[0].model_number);
      DEBUG_SERIAL.print(" firmware=");
      DEBUG_SERIAL.print(info[0].firmware_version);
    }
    DEBUG_SERIAL.print(" lib_err=");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  }
}

int32_t readCtItem(uint8_t item, uint8_t id, uint32_t& elapsed_us, int& lib_err) {
  clearDxlRx();
  uint32_t start = micros();
  int32_t value = dxl.readControlTableItem(item, id, g_read_timeout_ms);
  elapsed_us = micros() - start;
  lib_err = dxl.getLastLibErrCode();
  return value;
}

void runReadItem(String line, uint8_t item, const char* label) {
  int parsed_id = parseCommandId(line);
  if (parsed_id < 0) {
    DEBUG_SERIAL.print("ERROR ");
    DEBUG_SERIAL.print(label);
    DEBUG_SERIAL.println(" requires explicit ID, e.g. pos 11");
    return;
  }
  uint8_t id = (uint8_t)parsed_id;
  uint32_t elapsed_us = 0;
  int lib_err = 0;
  int32_t value = readCtItem(item, id, elapsed_us, lib_err);
  DEBUG_SERIAL.print(label);
  DEBUG_SERIAL.print(" id=");
  DEBUG_SERIAL.print(id);
  if (lib_err == DXL_LIB_OK) {
    DEBUG_SERIAL.print(" value=");
    DEBUG_SERIAL.print(value);
    if (item == FIRMWARE_VERSION || item == RETURN_DELAY_TIME || item == STATUS_RETURN_LEVEL) {
      DEBUG_SERIAL.print(" unsigned=");
      DEBUG_SERIAL.print((uint8_t)value);
    }
  } else {
    DEBUG_SERIAL.print(" value=INVALID");
  }
  DEBUG_SERIAL.print(" elapsed_us=");
  DEBUG_SERIAL.print(elapsed_us);
  DEBUG_SERIAL.print(" lib_err=");
  DEBUG_SERIAL.println(lib_err);
}

void runPositionBench(String line) {
  int parsed_id = parseCommandId(line);
  if (parsed_id < 0) {
    DEBUG_SERIAL.println("ERROR posbench requires explicit ID, e.g. posbench 11 50 5");
    return;
  }
  uint8_t id = (uint8_t)parsed_id;
  int samples = parseSecondInt(line, 50);
  if (samples > 1000) {
    samples = 1000;
  }
  int gap_ms = parseThirdInt(line, 2);
  if (gap_ms > 1000) {
    gap_ms = 1000;
  }

  DEBUG_SERIAL.print("POSBENCH samples=");
  DEBUG_SERIAL.print(samples);
  DEBUG_SERIAL.print(" gap_ms=");
  DEBUG_SERIAL.println(gap_ms);
  openDxlPort();
  delay(5);
  clearDxlRx();
  uint16_t ok_count = 0;
  uint16_t attempt_count = 0;
  uint32_t min_us = 0xFFFFFFFF;
  uint32_t max_us = 0;
  uint64_t total_us = 0;
  int32_t last_value = 0;
  int last_err = 0;
  uint16_t timeout_errs = 0;
  uint16_t crc_errs = 0;
  uint16_t overflow_errs = 0;
  uint16_t other_errs = 0;
  for (int sample = 0; sample < samples; ++sample) {
    uint32_t elapsed_us = 0;
    int lib_err = 0;
    int32_t value = readCtItem(PRESENT_POSITION, id, elapsed_us, lib_err);
    ++attempt_count;
    if (elapsed_us < min_us) min_us = elapsed_us;
    if (elapsed_us > max_us) max_us = elapsed_us;
    total_us += elapsed_us;
    last_err = lib_err;
    if (lib_err == DXL_LIB_OK) {
      ++ok_count;
      last_value = value;
    } else {
      if (lib_err == DXL_LIB_ERROR_TIMEOUT) {
        ++timeout_errs;
      } else if (lib_err == DXL_LIB_ERROR_CRC) {
        ++crc_errs;
      } else if (lib_err == DXL_LIB_ERROR_BUFFER_OVERFLOW) {
        ++overflow_errs;
      } else {
        ++other_errs;
      }
      delay(gap_ms + 2);
      clearDxlRx();
    }
    if (gap_ms > 0) {
      delay(gap_ms);
    }
  }
  float mean_us = attempt_count ? (float)total_us / (float)attempt_count : 0.0f;
  DEBUG_SERIAL.print("POSBENCH id=");
  DEBUG_SERIAL.print(id);
  DEBUG_SERIAL.print(" ok=");
  DEBUG_SERIAL.print(ok_count);
  DEBUG_SERIAL.print("/");
  DEBUG_SERIAL.print(attempt_count);
  DEBUG_SERIAL.print(" min_us=");
  DEBUG_SERIAL.print(min_us);
  DEBUG_SERIAL.print(" mean_us=");
  DEBUG_SERIAL.print(mean_us, 1);
  DEBUG_SERIAL.print(" max_us=");
  DEBUG_SERIAL.print(max_us);
  DEBUG_SERIAL.print(" last=");
  if (ok_count > 0) {
    DEBUG_SERIAL.print(last_value);
  } else {
    DEBUG_SERIAL.print("INVALID");
  }
  DEBUG_SERIAL.print(" last_err=");
  DEBUG_SERIAL.print(last_err);
  DEBUG_SERIAL.print(" timeout=");
  DEBUG_SERIAL.print(timeout_errs);
  DEBUG_SERIAL.print(" crc=");
  DEBUG_SERIAL.print(crc_errs);
  DEBUG_SERIAL.print(" overflow=");
  DEBUG_SERIAL.print(overflow_errs);
  DEBUG_SERIAL.print(" other=");
  DEBUG_SERIAL.println(other_errs);
}

void runRecover() {
  openDxlPort();
  uint16_t drained = clearDxlRx();
  DEBUG_SERIAL.print("RECOVER reopened baud=");
  DEBUG_SERIAL.print(g_baud);
  DEBUG_SERIAL.print(" protocol=");
  DEBUG_SERIAL.print(g_protocol, 1);
  DEBUG_SERIAL.print(" drained=");
  DEBUG_SERIAL.println(drained);
}

void runSetReturnDelayZero(String line) {
  int parsed_id = parseCommandId(line);
  if (parsed_id < 0) {
    DEBUG_SERIAL.println("ERROR setrd0 requires explicit ID, e.g. setrd0 11");
    return;
  }
  uint8_t id = (uint8_t)parsed_id;
  clearDxlRx();
  uint32_t start = micros();
  bool ok = dxl.writeControlTableItem(RETURN_DELAY_TIME, id, 0, g_read_timeout_ms);
  uint32_t elapsed_us = micros() - start;
  DEBUG_SERIAL.print("SETRD0 id=");
  DEBUG_SERIAL.print(id);
  DEBUG_SERIAL.print(" ok=");
  DEBUG_SERIAL.print(ok ? 1 : 0);
  DEBUG_SERIAL.print(" elapsed_us=");
  DEBUG_SERIAL.print(elapsed_us);
  DEBUG_SERIAL.print(" lib_err=");
  DEBUG_SERIAL.println(dxl.getLastLibErrCode());
}

void runSetReturnDelay(String line) {
  int parsed_id = parseCommandId(line);
  if (parsed_id < 0) {
    DEBUG_SERIAL.println("ERROR setrd requires explicit ID and value, e.g. setrd 11 50");
    return;
  }
  int value = parseSecondInt(line, -1);
  if (value < 0 || value > 250) {
    DEBUG_SERIAL.println("ERROR setrd value must be 0..250");
    return;
  }
  uint8_t id = (uint8_t)parsed_id;
  clearDxlRx();
  uint32_t start = micros();
  bool ok = dxl.writeControlTableItem(RETURN_DELAY_TIME, id, value, g_read_timeout_ms);
  uint32_t elapsed_us = micros() - start;
  DEBUG_SERIAL.print("SETRD id=");
  DEBUG_SERIAL.print(id);
  DEBUG_SERIAL.print(" value=");
  DEBUG_SERIAL.print(value);
  DEBUG_SERIAL.print(" ok=");
  DEBUG_SERIAL.print(ok ? 1 : 0);
  DEBUG_SERIAL.print(" elapsed_us=");
  DEBUG_SERIAL.print(elapsed_us);
  DEBUG_SERIAL.print(" lib_err=");
  DEBUG_SERIAL.println(dxl.getLastLibErrCode());
}

void runSetMotorBaud(String line) {
  int parsed_id = parseCommandId(line);
  if (parsed_id < 0) {
    DEBUG_SERIAL.println("ERROR setmotorbaud requires explicit ID and baud, e.g. setmotorbaud 11 1000000");
    return;
  }
  int baud = parseSecondInt(line, -1);
  if (!(baud == 9600 || baud == 57600 || baud == 115200 || baud == 1000000 ||
        baud == 2000000 || baud == 3000000 || baud == 4000000)) {
    DEBUG_SERIAL.println("ERROR baud must be 9600, 57600, 115200, 1000000, 2000000, 3000000, or 4000000");
    return;
  }

  uint8_t id = (uint8_t)parsed_id;
  clearDxlRx();
  bool torque_off_ok = dxl.torqueOff(id);
  delay(10);
  bool baud_ok = dxl.setBaudrate(id, (uint32_t)baud);
  DEBUG_SERIAL.print("SETMOTORBAUD id=");
  DEBUG_SERIAL.print(id);
  DEBUG_SERIAL.print(" baud=");
  DEBUG_SERIAL.print(baud);
  DEBUG_SERIAL.print(" torque_off_ok=");
  DEBUG_SERIAL.print(torque_off_ok ? 1 : 0);
  DEBUG_SERIAL.print(" baud_ok=");
  DEBUG_SERIAL.print(baud_ok ? 1 : 0);
  DEBUG_SERIAL.print(" lib_err=");
  DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  if (baud_ok) {
    g_baud = baud;
    openDxlPort();
    delay(20);
    clearDxlRx();
    DEBUG_SERIAL.print("DXL port switched to baud=");
    DEBUG_SERIAL.println(g_baud);
  }
}

void prepareSyncPositionRead() {
  memset(g_sync_positions, 0, sizeof(g_sync_positions));
  memset(g_sync_read_xels, 0, sizeof(g_sync_read_xels));
  g_sync_read_info.packet.p_buf = g_user_pkt_buf;
  g_sync_read_info.packet.buf_capacity = USER_PKT_BUF_CAP;
  g_sync_read_info.packet.gen_length = 0;
  g_sync_read_info.packet.is_completed = false;
  g_sync_read_info.addr = 132;
  g_sync_read_info.addr_length = 4;
  g_sync_read_info.p_xels = g_sync_read_xels;
  g_sync_read_info.xel_count = 0;
  for (uint8_t i = 0; i < g_id_count; ++i) {
    g_sync_read_xels[i].id = g_ids[i];
    g_sync_read_xels[i].p_recv_buf = reinterpret_cast<uint8_t*>(&g_sync_positions[i]);
    ++g_sync_read_info.xel_count;
  }
  g_sync_read_info.is_info_changed = true;
}

void runSyncPosition(bool fast) {
  printConfig();
  prepareSyncPositionRead();
  clearDxlRx();
  uint32_t start = micros();
  uint8_t received = fast ? dxl.fastSyncRead(&g_sync_read_info, g_read_timeout_ms)
                          : dxl.syncRead(&g_sync_read_info, g_read_timeout_ms);
  uint32_t elapsed = micros() - start;
  DEBUG_SERIAL.print(fast ? "FASTSYNCPOS" : "SYNCPOS");
  DEBUG_SERIAL.print(" received=");
  DEBUG_SERIAL.print(received);
  DEBUG_SERIAL.print("/");
  DEBUG_SERIAL.print(g_id_count);
  DEBUG_SERIAL.print(" elapsed_us=");
  DEBUG_SERIAL.print(elapsed);
  DEBUG_SERIAL.print(" lib_err=");
  DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  for (uint8_t i = 0; i < received; ++i) {
    DEBUG_SERIAL.print("  id=");
    DEBUG_SERIAL.print(g_sync_read_xels[i].id);
    DEBUG_SERIAL.print(" err=");
    DEBUG_SERIAL.print(g_sync_read_xels[i].error);
    DEBUG_SERIAL.print(" position=");
    DEBUG_SERIAL.println(g_sync_positions[i]);
  }
  if (received < g_id_count) {
    DEBUG_SERIAL.println("  remaining IDs did not return a valid grouped-read packet");
  }
}

void runScan() {
  const long bauds[] = {57600, 115200, 1000000, 2000000, 3000000};
  DEBUG_SERIAL.println("SCAN begin");
  uint16_t found_total = 0;
  for (int protocol = 1; protocol < 3; ++protocol) {
    g_protocol = (float)protocol;
    dxl.setPortProtocolVersion(g_protocol);
    DEBUG_SERIAL.print("SCAN PROTOCOL ");
    DEBUG_SERIAL.println(protocol);
    for (uint8_t b = 0; b < sizeof(bauds) / sizeof(bauds[0]); ++b) {
      g_baud = bauds[b];
      dxl.begin(g_baud);
      DEBUG_SERIAL.print("SCAN BAUDRATE ");
      DEBUG_SERIAL.println(g_baud);
      for (int id = 0; id < DXL_BROADCAST_ID; ++id) {
        if (dxl.ping(id)) {
          DEBUG_SERIAL.print("ID : ");
          DEBUG_SERIAL.print(id);
          DEBUG_SERIAL.print(", Model Number: ");
          DEBUG_SERIAL.println(dxl.getModelNumber(id));
          ++found_total;
        }
      }
    }
  }
  g_protocol = 2.0;
  g_baud = 1000000;
  openDxlPort();
  DEBUG_SERIAL.print("Total ");
  DEBUG_SERIAL.print(found_total);
  DEBUG_SERIAL.println(" DYNAMIXEL(s) found!");
}

void handleLine(String line) {
  line.trim();
  line.toLowerCase();
  if (line.length() == 0) {
    return;
  }
  if (line == "help" || line == "h") {
    printHelp();
  } else if (line.startsWith("protocol")) {
    setProtocol(line);
  } else if (line.startsWith("baud")) {
    setBaud(line);
  } else if (line.startsWith("timeout")) {
    setReadTimeout(line);
  } else if (line.startsWith("ids")) {
    setIds(line);
  } else if (line == "ping") {
    runPing();
  } else if (line == "cache") {
    runCacheModelNumbers();
  } else if (line.startsWith("fw")) {
    runReadItem(line, FIRMWARE_VERSION, "FW");
  } else if (line.startsWith("rd")) {
    runReadItem(line, RETURN_DELAY_TIME, "RD");
  } else if (line.startsWith("srl")) {
    runReadItem(line, STATUS_RETURN_LEVEL, "SRL");
  } else if (line.startsWith("setrd ")) {
    runSetReturnDelay(line);
  } else if (line.startsWith("setrd0")) {
    runSetReturnDelayZero(line);
  } else if (line.startsWith("setmotorbaud")) {
    runSetMotorBaud(line);
  } else if (line == "recover") {
    runRecover();
  } else if (line.startsWith("posbench")) {
    runPositionBench(line);
  } else if (line.startsWith("pos")) {
    runReadItem(line, PRESENT_POSITION, "POS");
  } else if (line.startsWith("cur")) {
    runReadItem(line, PRESENT_CURRENT, "CUR");
  } else if (line.startsWith("vel")) {
    runReadItem(line, PRESENT_VELOCITY, "VEL");
  } else if (line == "syncpos") {
    runSyncPosition(false);
  } else if (line == "fastsyncpos") {
    runSyncPosition(true);
  } else if (line == "scan") {
    runScan();
  } else {
    DEBUG_SERIAL.print("ERROR unknown command: ");
    DEBUG_SERIAL.println(line);
  }
}

void setup() {
  DEBUG_SERIAL.begin(DEBUG_BAUD);
  openDxlPort();
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("READY openrb_motor_console");
  printConfig();
  printHelp();
}

void loop() {
  if (DEBUG_SERIAL.available() > 0) {
    String line = DEBUG_SERIAL.readStringUntil('\n');
    handleLine(line);
  }
}
