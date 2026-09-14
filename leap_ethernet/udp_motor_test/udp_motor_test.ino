#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Dynamixel2Arduino.h>

/*
 * UDP Command Schema:
 * READ_ALL addr len                    -> OK val1 val2 ... / ERR message
 * WRITE_ALL addr len val1 val2 ...     -> OK / ERR message
 * READ addr len id1 id2 ...            -> OK val1 val2 ... / ERR message
 * WRITE addr len id1 val1 id2 val2 ... -> OK / ERR message
 * invalid command                      -> ERR unknown command
 */

#define DXL_SERIAL   Serial1
const int DXL_DIR_PIN = -1;
const int32_t BAUDRATE = 1000000;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// MAC address for the Ethernet shield
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
// Static IP address for the OpenRB-150
IPAddress ip(10, 42, 42, 50);

unsigned int localPort = 8888;

const int MAX_UDP_PAYLOAD_SIZE = 512; 
char packetBuffer[MAX_UDP_PAYLOAD_SIZE];
EthernetUDP Udp;

// --- Sync Buffers ---
const uint16_t user_pkt_buf_cap = 256;
uint8_t user_pkt_buf[user_pkt_buf_cap];

DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr[16];

DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[16];

// Backing arrays for Sync data. Max length of any table item is 4 bytes.
uint8_t sw_data_arr[16][4];
uint8_t sr_data_arr[16][4];

// --- Helper Functions ---
void sendResponse(const char* msg) {
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(msg);
  Udp.endPacket();
}

void sendError(const char* msg) {
  char errBuf[128];
  snprintf(errBuf, sizeof(errBuf), "ERR %s", msg);
  sendResponse(errBuf);
  Serial.println(errBuf);
}

// Helper to safely extract signed values based on data length
int32_t extractValue(uint8_t* buf, uint16_t item_len) {
  int32_t val = 0;
  if (item_len > 4) item_len = 4;
  memcpy(&val, buf, item_len);
  if (item_len == 1) val = *(int8_t*)buf;
  else if (item_len == 2) val = *(int16_t*)buf;
  return val;
}

// Helper to safely pack signed values into the buffer
void packValue(uint8_t* buf, int32_t val, uint16_t item_len) {
  if (item_len == 1) { int8_t v = val; memcpy(buf, &v, 1); }
  else if (item_len == 2) { int16_t v = val; memcpy(buf, &v, 2); }
  else { memcpy(buf, &val, 4); }
}

// Helper to drain any unread/stale bytes from the Dynamixel serial RX buffer
void drainDxlRx() {
  while (DXL_SERIAL.available()) {
    DXL_SERIAL.read();
  }
}

void handleRead(bool isAll, uint16_t addr, uint16_t item_len) {
  sr_infos.addr = addr;
  sr_infos.addr_length = item_len;
  
  if (isAll) {
    // Split 16 motors into two batches of 8 to keep incoming status packets
    // (~120 bytes per batch) safely within the SAMD21's 256-byte hardware serial ring buffer.
    uint8_t total_recv = 0;

    // --- Batch 1: Motors 0 to 7 ---
    sr_infos.xel_count = 8;
    for (int i = 0; i < 8; i++) {
      info_xels_sr[i].id = i;
      info_xels_sr[i].p_recv_buf = sr_data_arr[i];
      info_xels_sr[i].error = 0;
    }
    sr_infos.is_info_changed = true;
    drainDxlRx();
    uint8_t cnt1 = dxl.syncRead(&sr_infos, 50);
    total_recv += cnt1;

    // --- Batch 2: Motors 8 to 15 ---
    for (int i = 0; i < 8; i++) {
      info_xels_sr[i].id = i + 8;
      info_xels_sr[i].p_recv_buf = sr_data_arr[i + 8];
      info_xels_sr[i].error = 0;
    }
    sr_infos.is_info_changed = true;
    drainDxlRx();
    uint8_t cnt2 = dxl.syncRead(&sr_infos, 50);
    total_recv += cnt2;

    if (total_recv == 16) {
      char response[512];
      int offset = snprintf(response, sizeof(response), "OK");
      for (int i = 0; i < 16; i++) {
        int32_t val = extractValue(sr_data_arr[i], item_len);
        offset += snprintf(response + offset, sizeof(response) - offset, " %ld", (long)val);
      }
      sendResponse(response);
    } else {
      char errBuf[64];
      snprintf(errBuf, sizeof(errBuf), "syncRead failed (got %d/16 motors) LibErr: %d", total_recv, dxl.getLastLibErrCode());
      sendError(errBuf);
      drainDxlRx();
    }
  } else {
    sr_infos.xel_count = 0;
    while (char* id_str = strtok(NULL, " \r\n\t")) {
      if (sr_infos.xel_count < 16) {
        info_xels_sr[sr_infos.xel_count].id = atoi(id_str);
        info_xels_sr[sr_infos.xel_count].p_recv_buf = sr_data_arr[sr_infos.xel_count];
        info_xels_sr[sr_infos.xel_count].error = 0;
        sr_infos.xel_count++;
      }
    }

    if (sr_infos.xel_count == 0) {
      sendError("No IDs provided");
      return;
    }
    sr_infos.is_info_changed = true;
    drainDxlRx();

    uint8_t recv_cnt = dxl.syncRead(&sr_infos, 50);
    if (recv_cnt > 0) {
      char response[512];
      int offset = snprintf(response, sizeof(response), "OK");
      for (int i = 0; i < recv_cnt; i++) {
        int32_t val = extractValue(sr_data_arr[i], item_len);
        offset += snprintf(response + offset, sizeof(response) - offset, " %ld", (long)val);
      }
      sendResponse(response);
    } else {
      char errBuf[64];
      snprintf(errBuf, sizeof(errBuf), "syncRead failed LibErr: %d", dxl.getLastLibErrCode());
      sendError(errBuf);
      drainDxlRx();
    }
  }
}

void handleWrite(bool isAll, uint16_t addr, uint16_t item_len) {
  sw_infos.addr = addr;
  sw_infos.addr_length = item_len;
  sw_infos.xel_count = 0;
  
  if (isAll) {
    sw_infos.xel_count = 16;
    for (int i = 0; i < 16; i++) {
      char* val_str = strtok(NULL, " \r\n\t");
      if (!val_str) {
        sendError("Not enough values (need 16)");
        return;
      }
      packValue(sw_data_arr[i], atol(val_str), item_len);
      info_xels_sw[i].id = i;
      info_xels_sw[i].p_data = sw_data_arr[i];
    }
  } else {
    while (char* id_str = strtok(NULL, " \r\n\t")) {
      char* val_str = strtok(NULL, " \r\n\t");
      if (!val_str) {
        sendError("Mismatched ID/Value pairs");
        return;
      }
      if (sw_infos.xel_count < 16) {
        int idx = sw_infos.xel_count;
        packValue(sw_data_arr[idx], atol(val_str), item_len);
        info_xels_sw[idx].id = atoi(id_str);
        info_xels_sw[idx].p_data = sw_data_arr[idx];
        sw_infos.xel_count++;
      }
    }
  }

  if (sw_infos.xel_count == 0) {
    sendError("No IDs provided");
    return;
  }
  sw_infos.is_info_changed = true;

  if (dxl.syncWrite(&sw_infos)) {
    drainDxlRx();
    sendResponse("OK");
  } else {
    drainDxlRx();
    char errBuf[64];
    snprintf(errBuf, sizeof(errBuf), "syncWrite failed LibErr: %d", dxl.getLastLibErrCode());
    sendError(errBuf);
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  dxl.begin(BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  sr_infos.packet.p_buf = user_pkt_buf;
  sr_infos.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos.packet.is_completed = false;
  sr_infos.p_xels = info_xels_sr;
  
  sw_infos.packet.p_buf = nullptr;
  sw_infos.packet.is_completed = false;
  sw_infos.p_xels = info_xels_sw;

  for (int i = 0; i < 16; i++) {
    dxl.ping(i);
    dxl.torqueOff(i); // Start with torque off for configuration
    // dxl.setOperatingMode(i, OP_POSITION);
  }

  Serial.println("Initializing Ethernet...");
  Ethernet.begin(mac, ip);
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.");
    while (true) delay(1);
  }
  
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }
  
  Udp.begin(localPort);
  Serial.print("Listening for UDP on ");
  Serial.print(Ethernet.localIP());
  Serial.print(":");
  Serial.println(localPort);
}

void loop() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    int len = Udp.read(packetBuffer, MAX_UDP_PAYLOAD_SIZE - 1);
    if (len >= 0) packetBuffer[len] = '\0';
    
    static bool ledState = false;
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);

    char* cmd = strtok(packetBuffer, " \r\n\t");
    if (!cmd) return;

    bool is_read_all = (strcmp(cmd, "READ_ALL") == 0);
    bool is_write_all = (strcmp(cmd, "WRITE_ALL") == 0);
    bool is_read = (strcmp(cmd, "READ") == 0);
    bool is_write = (strcmp(cmd, "WRITE") == 0);

    if (!is_read_all && !is_write_all && !is_read && !is_write) {
      sendError("unknown command");
      return;
    }

    char* addr_str = strtok(NULL, " \r\n\t");
    char* len_str = strtok(NULL, " \r\n\t");
    
    if (!addr_str || !len_str) {
      sendError("Missing address or length");
      return;
    }

    uint16_t addr = atoi(addr_str);
    uint16_t item_len = atoi(len_str);

    if (item_len != 1 && item_len != 2 && item_len != 4) {
      sendError("Invalid length (must be 1, 2, or 4)");
      return;
    }

    if (is_read_all) handleRead(true, addr, item_len);
    else if (is_write_all) handleWrite(true, addr, item_len);
    else if (is_read) handleRead(false, addr, item_len);
    else if (is_write) handleWrite(false, addr, item_len);
  }
}