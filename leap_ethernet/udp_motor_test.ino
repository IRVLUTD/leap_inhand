#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Dynamixel2Arduino.h>

/*
 * UDP Binary Packet Schema:
 * Total size: 84 bytes
 * 
 * struct UdpPacket {
 *     uint8_t cmd;      // 0=READ_ALL, 1=WRITE_ALL, 2=READ, 3=WRITE, 255=ERR
 *     uint8_t addr;     // Dynamixel address (0-255)
 *     uint8_t length;   // 1, 2, or 4
 *     uint8_t count;    // Number of motors in this packet (0-16)
 *     struct {
 *         uint8_t id;   // Motor ID
 *         int32_t val;  // Value
 *     } motors[16];
 * };
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

EthernetUDP Udp;

#pragma pack(push, 1)
struct UdpPacket {
    uint8_t cmd;
    uint8_t addr;
    uint8_t length;
    uint8_t count;
    struct {
        uint8_t id;
        int32_t val;
    } motors[16];
};
#pragma pack(pop)

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

// Track torque state to prevent movement commands when torque is off
bool torque_enabled[16] = {false};

// --- Helper Functions ---
void drainDxlRx() {
  while (DXL_SERIAL.available() > 0) {
    DXL_SERIAL.read();
  }
}
void sendResponse(UdpPacket* pkt) {
  // Serial.print("SENT -> cmd:"); Serial.print(pkt->cmd);
  // Serial.print(" addr:"); Serial.print(pkt->addr);
  // Serial.print(" len:"); Serial.print(pkt->length);
  // Serial.print(" cnt:"); Serial.println(pkt->count);
  if (pkt->cmd == 255) {
    int32_t err_code = 0;
    memcpy(&err_code, &pkt->motors[0].val, sizeof(int32_t)); // Safe unaligned load
    Serial.print("  [ERR CODE]: "); Serial.println(err_code);
  }

  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write((uint8_t*)pkt, sizeof(UdpPacket));
  Udp.endPacket();
}

void sendError(UdpPacket* pkt, int32_t err_code) {
  pkt->cmd = 255;
  pkt->count = 1;
  int32_t temp = err_code;
  memcpy(&pkt->motors[0].val, &temp, sizeof(int32_t)); // Safe unaligned store
  sendResponse(pkt);
}

// Helper to safely extract signed values based on data length
int32_t extractValue(uint8_t* buf, uint8_t item_len) {
  if (item_len > 4) item_len = 4;
  int32_t val = 0;
  memcpy(&val, buf, item_len);
  if (item_len == 1) val = (int8_t)val;   // Safely sign-extend
  if (item_len == 2) val = (int16_t)val;  // Safely sign-extend
  return val;
}

// Helper to safely pack signed values into the buffer
void packValue(uint8_t* buf, int32_t val, uint8_t item_len) {
  if (item_len > 4) item_len = 4;
  memcpy(buf, &val, item_len);
}

void handleRead(UdpPacket* pkt, bool isAll) {
  sr_infos.addr = pkt->addr;
  sr_infos.addr_length = pkt->length;
  
  if (isAll) {
    // Split into two batches of 8 motors to keep serial RX traffic under the 
    // SAMD21's 256-byte hardware ring buffer limit (each 8-motor batch is only ~142 bytes)
    uint8_t total_recv = 0;
    
    // --- Batch 1: Motors 0 to 7 ---
    sr_infos.xel_count = 8;
    for (int i = 0; i < 8; i++) {
      info_xels_sr[i].id = i;
      info_xels_sr[i].p_recv_buf = sr_data_arr[i];
    }
    sr_infos.is_info_changed = true;
    drainDxlRx();
    uint8_t cnt1 = dxl.syncRead(&sr_infos);
    for (int i = 0; i < cnt1; i++) {
      pkt->motors[total_recv].id = info_xels_sr[i].id;
      int32_t temp_val = extractValue(sr_data_arr[i], pkt->length);
      memcpy(&pkt->motors[total_recv].val, &temp_val, sizeof(int32_t));
      total_recv++;
    }
    
    // --- Batch 2: Motors 8 to 15 ---
    for (int i = 0; i < 8; i++) {
      info_xels_sr[i].id = i + 8;
      info_xels_sr[i].p_recv_buf = sr_data_arr[i + 8];
    }
    sr_infos.is_info_changed = true;
    drainDxlRx();
    uint8_t cnt2 = dxl.syncRead(&sr_infos);
    for (int i = 0; i < cnt2; i++) {
      pkt->motors[total_recv].id = info_xels_sr[i].id;
      int32_t temp_val = extractValue(sr_data_arr[i + 8], pkt->length);
      memcpy(&pkt->motors[total_recv].val, &temp_val, sizeof(int32_t));
      total_recv++;
    }
    
    pkt->count = total_recv;
    for (int i = total_recv; i < 16; i++) {
      pkt->motors[i].id = 255;
      int32_t zero = 0;
      memcpy(&pkt->motors[i].val, &zero, sizeof(int32_t));
    }
    
    if (total_recv > 0) {
      sendResponse(pkt);
    } else {
      sendError(pkt, -4); // -4: syncRead timed out
    }
  } else {
    sr_infos.xel_count = 0;
    for (int i = 0; i < pkt->count && i < 16; i++) {
      info_xels_sr[i].id = pkt->motors[i].id;
      info_xels_sr[i].p_recv_buf = sr_data_arr[i];
      sr_infos.xel_count++;
    }

    if (sr_infos.xel_count == 0) {
      sendError(pkt, -1); // -1: No IDs provided
      return;
    }
    sr_infos.is_info_changed = true;

    drainDxlRx();

    uint8_t recv_cnt = dxl.syncRead(&sr_infos);
    if (recv_cnt > 0) {
      pkt->count = recv_cnt;
      for (int i = 0; i < recv_cnt; i++) {
        pkt->motors[i].id = info_xels_sr[i].id;
        int32_t temp_val = extractValue(sr_data_arr[i], pkt->length);
        memcpy(&pkt->motors[i].val, &temp_val, sizeof(int32_t)); // Safe unaligned store
      }
      // Clean out unused slots so stale/dirty values from the request aren't sent back
      for (int i = recv_cnt; i < 16; i++) {
        pkt->motors[i].id = 255;
        int32_t zero = 0;
        memcpy(&pkt->motors[i].val, &zero, sizeof(int32_t));
      }
      sendResponse(pkt);
    } else {
      sendError(pkt, dxl.getLastLibErrCode());
    }
  }
}

void handleWrite(UdpPacket* pkt, bool isAll) {
  // Serial.println("--- Entering handleWrite ---");
  sw_infos.addr = pkt->addr;
  sw_infos.addr_length = pkt->length;
  sw_infos.xel_count = 0;
  
  // Pre-calculate safety checks to save CPU cycles inside the loop
  bool is_movement_cmd = (pkt->addr == 116 || pkt->addr == 104 || pkt->addr == 102 || pkt->addr == 100);
  bool is_torque_cmd = (pkt->addr == 64);
  
  if (isAll) {
    sw_infos.xel_count = 16;
    for (int i = 0; i < 16; i++) {
      int32_t val;
      memcpy(&val, &pkt->motors[i].val, sizeof(int32_t)); // Safe unaligned load
      
      if (is_movement_cmd && !torque_enabled[i]) {
        Serial.print("BLOCKED: Torque OFF for ID "); Serial.println(i);
        sendError(pkt, -2); // -2: Torque is off
        return;
      }
      if (is_torque_cmd) torque_enabled[i] = (val != 0);
      
      packValue(sw_data_arr[i], val, pkt->length);
      info_xels_sw[i].id = i; // For WRITE_ALL, assume IDs 0-15 sequentially
      info_xels_sw[i].p_data = sw_data_arr[i];
    }
  } else {
    for (int i = 0; i < pkt->count && i < 16; i++) {
      uint8_t id = pkt->motors[i].id;
      int32_t val;
      memcpy(&val, &pkt->motors[i].val, sizeof(int32_t)); // Safe unaligned load
      
      if (is_movement_cmd && id < 16 && !torque_enabled[id]) {
        Serial.print("BLOCKED: Torque OFF for ID "); Serial.println(id);
        sendError(pkt, -2); // -2: Torque is off
        return;
      }
      if (is_torque_cmd && id < 16) torque_enabled[id] = (val != 0);
      
      packValue(sw_data_arr[i], val, pkt->length);
      info_xels_sw[i].id = id;
      info_xels_sw[i].p_data = sw_data_arr[i];
      sw_infos.xel_count++;
    }
  }

  if (sw_infos.xel_count == 0) {
    Serial.println("ERR: sw_infos.xel_count is 0");
    sendError(pkt, -1);
    return;
  }
  sw_infos.is_info_changed = true;

  // Serial.print("Executing syncWrite for "); Serial.print(sw_infos.xel_count); Serial.println(" motors...");
  if (dxl.syncWrite(&sw_infos)) {
    // Serial.println("syncWrite SUCCESS");
    drainDxlRx();
    delayMicroseconds(500);  // bus / DIR settle & motor write processing
    drainDxlRx();
    sendResponse(pkt);
  } else {
    int32_t err = dxl.getLastLibErrCode();
    // Serial.print("syncWrite FAILED, lib err: "); Serial.println(err);
    sendError(pkt, err);
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
    if (dxl.ping(i)) {
      Serial.print("Motor "); Serial.print(i); Serial.println(" OK");
    } else {
      Serial.print("WARNING: Motor "); Serial.print(i); Serial.println(" NOT FOUND!");
    }

    dxl.torqueOff(i); // Start with torque off for configuration
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
  
  // Only process if the packet is exactly the size of our UdpPacket struct (84 bytes)
  if (packetSize == sizeof(UdpPacket)) {
    UdpPacket pkt;
    Udp.read((char*)&pkt, sizeof(UdpPacket));
    
    // Serial.print("RECV -> cmd:"); Serial.print(pkt.cmd);
    // Serial.print(" addr:"); Serial.print(pkt.addr);
    // Serial.print(" len:"); Serial.print(pkt.length);
    // Serial.print(" cnt:"); Serial.println(pkt.count);
    
    static bool ledState = false;
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);

    if (pkt.cmd == 0) handleRead(&pkt, true);
    else if (pkt.cmd == 1) handleWrite(&pkt, true);
    else if (pkt.cmd == 2) handleRead(&pkt, false);
    else if (pkt.cmd == 3) handleWrite(&pkt, false);
    else sendError(&pkt, -3); // -3: Unknown command
    
  } else if (packetSize > 0) {
    // Drop invalid length packets immediately by draining all unread bytes
    while (Udp.available()) {
      Udp.read();
    }
  }
}
