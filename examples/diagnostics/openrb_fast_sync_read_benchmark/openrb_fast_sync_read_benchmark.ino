/*******************************************************************************
 * Minimal OpenRB-150 Dynamixel telemetry benchmark.
 *
 * This sketch is intentionally separate from the exo firmware. Flash it only
 * when you want to isolate raw Dynamixel bus read speed, then flash the normal
 * exo firmware back afterward.
 *
 * Serial commands at 2,000,000 baud:
 *   help
 *   baud 2000000
 *   ping
 *   scan
 *   fullscan
 *   readone
 *   benchread 200
 *   ids 11 12 13 14 15 16 17 18 19
 *   once
 *   bench 200
 ******************************************************************************/

#include <Dynamixel2Arduino.h>

#if defined(ARDUINO_OpenRB)
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2;
#endif

using namespace ControlTableItem;

constexpr long DEBUG_BAUD = 2000000;
long dxl_baud = 2000000;
float dxl_protocol = 2.0;
constexpr uint8_t MAX_IDS = 18;
constexpr uint16_t TELEM_START_ADDR = 126;  // Present Current
constexpr uint16_t TELEM_ADDR_LEN = 10;     // Current(2) + Velocity(4) + Position(4)

struct __attribute__((packed)) TelemetryRaw {
  int16_t present_current;
  int32_t present_velocity;
  int32_t present_position;
};

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

uint8_t ids[MAX_IDS] = {11, 12, 13, 14, 15, 16, 17, 18, 19};
uint8_t id_count = 9;
uint8_t packet_buf[512];
TelemetryRaw raw[MAX_IDS];
DYNAMIXEL::InfoSyncReadInst_t sr_info;
DYNAMIXEL::XELInfoSyncRead_t sr_xels[MAX_IDS];

void prepareSyncRead() {
  memset(raw, 0, sizeof(raw));
  memset(&sr_info, 0, sizeof(sr_info));
  memset(sr_xels, 0, sizeof(sr_xels));

  sr_info.packet.p_buf = packet_buf;
  sr_info.packet.buf_capacity = sizeof(packet_buf);
  sr_info.packet.is_completed = false;
  sr_info.addr = TELEM_START_ADDR;
  sr_info.addr_length = TELEM_ADDR_LEN;
  sr_info.p_xels = sr_xels;
  sr_info.xel_count = id_count;

  for (uint8_t i = 0; i < id_count; ++i) {
    sr_xels[i].id = ids[i];
    sr_xels[i].p_recv_buf = reinterpret_cast<uint8_t*>(&raw[i]);
    sr_xels[i].error = 0;
  }
  sr_info.is_info_changed = true;
}

void printHelp() {
  DEBUG_SERIAL.println("OpenRB fast sync read benchmark");
  DEBUG_SERIAL.println("Commands:");
  DEBUG_SERIAL.println("  protocol 1|2");
  DEBUG_SERIAL.println("  baud 9600|57600|115200|1000000|2000000|3000000|4000000");
  DEBUG_SERIAL.println("  ping");
  DEBUG_SERIAL.println("  scan      (all IDs at current baud/protocol)");
  DEBUG_SERIAL.println("  baudscan  (selected IDs at common bauds/protocol 2)");
  DEBUG_SERIAL.println("  fullscan  (all IDs, protocol 1+2, common bauds)");
  DEBUG_SERIAL.println("  readone   (individual readControlTableItem diagnostics)");
  DEBUG_SERIAL.println("  benchread 200");
  DEBUG_SERIAL.println("  ids 11 12 13 14 15 16 17 18 19");
  DEBUG_SERIAL.println("  once");
  DEBUG_SERIAL.println("  bench 200");
}

void beginDxl(long baud) {
  dxl_baud = baud;
  dxl.setPortProtocolVersion(dxl_protocol);
  dxl.begin(dxl_baud);
  prepareSyncRead();
  DEBUG_SERIAL.print("DXL baud set to ");
  DEBUG_SERIAL.print(dxl_baud);
  DEBUG_SERIAL.print(", protocol=");
  DEBUG_SERIAL.println(dxl_protocol, 1);
}

void setProtocolFromLine(String line) {
  int space = line.indexOf(' ');
  if (space < 0) {
    DEBUG_SERIAL.print("Current DXL protocol: ");
    DEBUG_SERIAL.println(dxl_protocol, 1);
    return;
  }
  int parsed = line.substring(space + 1).toInt();
  if (parsed != 1 && parsed != 2) {
    DEBUG_SERIAL.println("ERROR protocol must be 1 or 2");
    return;
  }
  dxl_protocol = (float)parsed;
  beginDxl(dxl_baud);
}

void printIds() {
  DEBUG_SERIAL.print("IDs:");
  for (uint8_t i = 0; i < id_count; ++i) {
    DEBUG_SERIAL.print(' ');
    DEBUG_SERIAL.print(ids[i]);
  }
  DEBUG_SERIAL.println();
}

void setIdsFromLine(String line) {
  uint8_t parsed[MAX_IDS];
  uint8_t parsed_count = 0;
  int start = line.indexOf(' ');
  while (start >= 0 && parsed_count < MAX_IDS) {
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
      parsed[parsed_count++] = static_cast<uint8_t>(value);
    }
    start = end;
  }
  if (parsed_count == 0) {
    DEBUG_SERIAL.println("ERROR no valid IDs");
    return;
  }
  id_count = parsed_count;
  for (uint8_t i = 0; i < id_count; ++i) {
    ids[i] = parsed[i];
  }
  prepareSyncRead();
  printIds();
}

void setBaudFromLine(String line) {
  int space = line.indexOf(' ');
  if (space < 0) {
    DEBUG_SERIAL.print("Current DXL baud: ");
    DEBUG_SERIAL.println(dxl_baud);
    return;
  }
  long parsed = line.substring(space + 1).toInt();
  if (parsed <= 0) {
    DEBUG_SERIAL.println("ERROR invalid baud");
    return;
  }
  beginDxl(parsed);
}

void runPing() {
  DEBUG_SERIAL.print("PING baud=");
  DEBUG_SERIAL.print(dxl_baud);
  DEBUG_SERIAL.print(" protocol=");
  DEBUG_SERIAL.print(dxl_protocol, 1);
  DEBUG_SERIAL.print(" ids=");
  DEBUG_SERIAL.println(id_count);
  for (uint8_t i = 0; i < id_count; ++i) {
    uint32_t start = micros();
    bool ok = dxl.ping(ids[i]);
    uint32_t elapsed = micros() - start;
    DEBUG_SERIAL.print("  id=");
    DEBUG_SERIAL.print(ids[i]);
    DEBUG_SERIAL.print(" ok=");
    DEBUG_SERIAL.print(ok ? 1 : 0);
    DEBUG_SERIAL.print(" elapsed_us=");
    DEBUG_SERIAL.println(elapsed);
  }
}

void runScan() {
  DEBUG_SERIAL.print("SCAN baud=");
  DEBUG_SERIAL.print(dxl_baud);
  DEBUG_SERIAL.print(" protocol=");
  DEBUG_SERIAL.println(dxl_protocol, 1);
  DEBUG_SERIAL.print("  found:");
  uint8_t found = 0;
  for (uint8_t id = 1; id < 253; ++id) {
    if (dxl.ping(id)) {
      DEBUG_SERIAL.print(' ');
      DEBUG_SERIAL.print(id);
      DEBUG_SERIAL.print("(model=");
      DEBUG_SERIAL.print(dxl.getModelNumber(id));
      DEBUG_SERIAL.print(")");
      ++found;
    }
  }
  if (found == 0) {
    DEBUG_SERIAL.print(" none");
  }
  DEBUG_SERIAL.println();
}

void runBaudScan() {
  const long bauds[] = {9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000};
  dxl_protocol = 2.0;
  DEBUG_SERIAL.println("BAUDSCAN selected IDs, protocol=2.0");
  for (uint8_t b = 0; b < sizeof(bauds) / sizeof(bauds[0]); ++b) {
    beginDxl(bauds[b]);
    DEBUG_SERIAL.print("  baud=");
    DEBUG_SERIAL.print(dxl_baud);
    DEBUG_SERIAL.print(" found:");
    uint8_t found = 0;
    for (uint8_t i = 0; i < id_count; ++i) {
      if (dxl.ping(ids[i])) {
        DEBUG_SERIAL.print(' ');
        DEBUG_SERIAL.print(ids[i]);
        ++found;
      }
    }
    if (found == 0) {
      DEBUG_SERIAL.print(" none");
    }
    DEBUG_SERIAL.println();
  }
}

void runFullScan() {
  const long bauds[] = {9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000};
  DEBUG_SERIAL.println("FULLSCAN all IDs, protocol 1 and 2");
  uint16_t total = 0;
  for (uint8_t protocol = 1; protocol <= 2; ++protocol) {
    dxl_protocol = (float)protocol;
    for (uint8_t b = 0; b < sizeof(bauds) / sizeof(bauds[0]); ++b) {
      beginDxl(bauds[b]);
      DEBUG_SERIAL.print("  protocol=");
      DEBUG_SERIAL.print(dxl_protocol, 1);
      DEBUG_SERIAL.print(" baud=");
      DEBUG_SERIAL.print(dxl_baud);
      DEBUG_SERIAL.print(" found:");
      uint8_t found = 0;
      for (uint8_t id = 1; id < 253; ++id) {
        if (dxl.ping(id)) {
          DEBUG_SERIAL.print(' ');
          DEBUG_SERIAL.print(id);
          DEBUG_SERIAL.print("(model=");
          DEBUG_SERIAL.print(dxl.getModelNumber(id));
          DEBUG_SERIAL.print(")");
          ++found;
          ++total;
        }
      }
      if (found == 0) {
        DEBUG_SERIAL.print(" none");
      }
      DEBUG_SERIAL.println();
    }
  }
  dxl_protocol = 2.0;
  beginDxl(dxl_baud);
  DEBUG_SERIAL.print("FULLSCAN total found entries=");
  DEBUG_SERIAL.println(total);
}

bool readIndividualRecord(uint8_t id, TelemetryRaw& out, uint32_t timeout_ms = 10) {
  out.present_current = (int16_t)dxl.readControlTableItem(PRESENT_CURRENT, id, timeout_ms);
  bool current_ok = (dxl.getLastLibErrCode() == DXL_LIB_OK);
  out.present_velocity = (int32_t)dxl.readControlTableItem(PRESENT_VELOCITY, id, timeout_ms);
  bool velocity_ok = (dxl.getLastLibErrCode() == DXL_LIB_OK);
  out.present_position = (int32_t)dxl.readControlTableItem(PRESENT_POSITION, id, timeout_ms);
  bool position_ok = (dxl.getLastLibErrCode() == DXL_LIB_OK);
  return current_ok && velocity_ok && position_ok;
}

void runIndividualReadOnce() {
  DEBUG_SERIAL.print("READONE baud=");
  DEBUG_SERIAL.print(dxl_baud);
  DEBUG_SERIAL.print(" protocol=");
  DEBUG_SERIAL.print(dxl_protocol, 1);
  DEBUG_SERIAL.print(" ids=");
  DEBUG_SERIAL.println(id_count);

  for (uint8_t i = 0; i < id_count; ++i) {
    uint8_t id = ids[i];
    TelemetryRaw one;
    uint32_t start = micros();
    bool ok = readIndividualRecord(id, one, 10);
    uint32_t elapsed = micros() - start;
    DEBUG_SERIAL.print("  id=");
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print(" ok=");
    DEBUG_SERIAL.print(ok ? 1 : 0);
    DEBUG_SERIAL.print(" elapsed_us=");
    DEBUG_SERIAL.print(elapsed);
    DEBUG_SERIAL.print(" current_mA=");
    DEBUG_SERIAL.print(one.present_current);
    DEBUG_SERIAL.print(" velocity_raw=");
    DEBUG_SERIAL.print(one.present_velocity);
    DEBUG_SERIAL.print(" position_ticks=");
    DEBUG_SERIAL.print(one.present_position);
    DEBUG_SERIAL.print(" last_lib_err=");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  }
}

void benchmarkIndividualRead(uint16_t samples) {
  uint16_t ok = 0;
  uint32_t min_us = 0xFFFFFFFF;
  uint32_t max_us = 0;
  uint64_t total_us = 0;

  for (uint16_t sample = 0; sample < samples; ++sample) {
    bool all_ok = true;
    uint32_t start = micros();
    for (uint8_t i = 0; i < id_count; ++i) {
      TelemetryRaw one;
      all_ok = readIndividualRecord(ids[i], one, 10) && all_ok;
    }
    uint32_t elapsed = micros() - start;
    if (all_ok) {
      ++ok;
    }
    if (elapsed < min_us) {
      min_us = elapsed;
    }
    if (elapsed > max_us) {
      max_us = elapsed;
    }
    total_us += elapsed;
  }

  float mean_us = samples ? (static_cast<float>(total_us) / samples) : 0.0f;
  DEBUG_SERIAL.print("BENCH method=individualRead ids=");
  DEBUG_SERIAL.print(id_count);
  DEBUG_SERIAL.print(" samples=");
  DEBUG_SERIAL.print(samples);
  DEBUG_SERIAL.print(" ok=");
  DEBUG_SERIAL.print(ok);
  DEBUG_SERIAL.print(" min_us=");
  DEBUG_SERIAL.print(min_us);
  DEBUG_SERIAL.print(" mean_us=");
  DEBUG_SERIAL.print(mean_us, 1);
  DEBUG_SERIAL.print(" max_us=");
  DEBUG_SERIAL.print(max_us);
  DEBUG_SERIAL.print(" mean_hz=");
  DEBUG_SERIAL.println(mean_us > 0.0f ? 1000000.0f / mean_us : 0.0f, 1);
}

uint8_t doFastSyncRead(uint32_t timeout_ms = 10) {
  return dxl.fastSyncRead(&sr_info, timeout_ms);
}

uint8_t doSyncRead(uint32_t timeout_ms = 10) {
  return dxl.syncRead(&sr_info, timeout_ms);
}

void printRecords(const char* method, uint8_t received, uint32_t elapsed_us) {
  DEBUG_SERIAL.print(method);
  DEBUG_SERIAL.print(" received=");
  DEBUG_SERIAL.print(received);
  DEBUG_SERIAL.print("/");
  DEBUG_SERIAL.print(id_count);
  DEBUG_SERIAL.print(" elapsed_us=");
  DEBUG_SERIAL.println(elapsed_us);

  for (uint8_t i = 0; i < id_count; ++i) {
    DEBUG_SERIAL.print("  id=");
    DEBUG_SERIAL.print(ids[i]);
    DEBUG_SERIAL.print(" err=");
    DEBUG_SERIAL.print(sr_xels[i].error);
    DEBUG_SERIAL.print(" current_mA=");
    DEBUG_SERIAL.print(raw[i].present_current);
    DEBUG_SERIAL.print(" velocity_raw=");
    DEBUG_SERIAL.print(raw[i].present_velocity);
    DEBUG_SERIAL.print(" position_ticks=");
    DEBUG_SERIAL.println(raw[i].present_position);
  }
}

void runOnce() {
  prepareSyncRead();
  uint32_t start = micros();
  uint8_t received = doFastSyncRead();
  uint32_t elapsed = micros() - start;
  printRecords("fastSyncRead", received, elapsed);

  prepareSyncRead();
  start = micros();
  received = doSyncRead();
  elapsed = micros() - start;
  printRecords("syncRead", received, elapsed);
}

void benchmarkMethod(const char* label, bool fast, uint16_t samples) {
  uint16_t ok = 0;
  uint32_t min_us = 0xFFFFFFFF;
  uint32_t max_us = 0;
  uint64_t total_us = 0;

  for (uint16_t i = 0; i < samples; ++i) {
    prepareSyncRead();
    uint32_t start = micros();
    uint8_t received = fast ? doFastSyncRead() : doSyncRead();
    uint32_t elapsed = micros() - start;
    if (received == id_count) {
      ++ok;
    }
    if (elapsed < min_us) {
      min_us = elapsed;
    }
    if (elapsed > max_us) {
      max_us = elapsed;
    }
    total_us += elapsed;
  }

  float mean_us = samples ? (static_cast<float>(total_us) / samples) : 0.0f;
  DEBUG_SERIAL.print("BENCH method=");
  DEBUG_SERIAL.print(label);
  DEBUG_SERIAL.print(" ids=");
  DEBUG_SERIAL.print(id_count);
  DEBUG_SERIAL.print(" samples=");
  DEBUG_SERIAL.print(samples);
  DEBUG_SERIAL.print(" ok=");
  DEBUG_SERIAL.print(ok);
  DEBUG_SERIAL.print(" min_us=");
  DEBUG_SERIAL.print(min_us);
  DEBUG_SERIAL.print(" mean_us=");
  DEBUG_SERIAL.print(mean_us, 1);
  DEBUG_SERIAL.print(" max_us=");
  DEBUG_SERIAL.print(max_us);
  DEBUG_SERIAL.print(" mean_hz=");
  DEBUG_SERIAL.println(mean_us > 0.0f ? 1000000.0f / mean_us : 0.0f, 1);
}

void runBench(String line) {
  int samples = 200;
  int space = line.indexOf(' ');
  if (space >= 0) {
    int parsed = line.substring(space + 1).toInt();
    if (parsed > 0 && parsed <= 5000) {
      samples = parsed;
    }
  }
  benchmarkMethod("fastSyncRead", true, static_cast<uint16_t>(samples));
  benchmarkMethod("syncRead", false, static_cast<uint16_t>(samples));
}

void runBenchRead(String line) {
  int samples = 200;
  int space = line.indexOf(' ');
  if (space >= 0) {
    int parsed = line.substring(space + 1).toInt();
    if (parsed > 0 && parsed <= 5000) {
      samples = parsed;
    }
  }
  benchmarkIndividualRead(static_cast<uint16_t>(samples));
}

void setup() {
  DEBUG_SERIAL.begin(DEBUG_BAUD);
  while (!DEBUG_SERIAL && millis() < 3000) {
  }

  beginDxl(dxl_baud);

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("READY openrb_fast_sync_read_benchmark");
  printIds();
  printHelp();
}

void loop() {
  if (!DEBUG_SERIAL.available()) {
    return;
  }

  String line = DEBUG_SERIAL.readStringUntil('\n');
  line.trim();
  line.toLowerCase();

  if (line == "help" || line == "h") {
    printHelp();
  } else if (line.startsWith("protocol ")) {
    setProtocolFromLine(line);
  } else if (line == "protocol") {
    setProtocolFromLine(line);
  } else if (line.startsWith("baud ")) {
    setBaudFromLine(line);
  } else if (line == "baud") {
    setBaudFromLine(line);
  } else if (line == "ping") {
    runPing();
  } else if (line == "scan") {
    runScan();
  } else if (line == "baudscan") {
    runBaudScan();
  } else if (line == "fullscan") {
    runFullScan();
  } else if (line == "readone") {
    runIndividualReadOnce();
  } else if (line.startsWith("benchread")) {
    runBenchRead(line);
  } else if (line.startsWith("ids")) {
    setIdsFromLine(line);
  } else if (line == "once") {
    runOnce();
  } else if (line.startsWith("bench")) {
    runBench(line);
  } else if (line.length() > 0) {
    DEBUG_SERIAL.print("ERROR unknown command: ");
    DEBUG_SERIAL.println(line);
  }
}
