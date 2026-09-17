#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Dynamixel2Arduino.h>

/*
 * UDP Binary Packet Schema:
 * Total size: 183 bytes
 * 
 * struct UdpPacket {
 *     uint8_t cmd;         // 0=READ_ALL, 1=WRITE_ALL, 2=WRITE_READ_ALL, 3=READ, 4=WRITE, 5=WRITE_READ, 254=REBOOT, 255=ERR
 *     uint8_t flags;       // 0x00=NONE, 0x01=IGNORE_ERRORS (do not bubble up / block on errors)
 *     uint8_t addr;        // Dynamixel write address (for REBOOT: 0=BOARD, 1=MOTORS, 2=ALL)
 *     uint8_t length;      // Write length (1, 2, 4, or 10)
 *     uint8_t count;       // Number of motors in this packet (0-16)
 *     uint8_t read_addr;   // Dynamixel read address for WRITE_READ commands (0-255)
 *     uint8_t read_length; // Read length (1, 2, 4, or 10)
 *     struct {
 *         uint8_t id;      // Motor ID
 *         uint8_t val[10]; // 80-bit raw value buffer
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
    uint8_t flags;
    uint8_t addr;
    uint8_t length;
    uint8_t count;
    uint8_t read_addr;
    uint8_t read_length;
    struct {
        uint8_t id;
        uint8_t val[10];
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

// Backing arrays for Sync data. Max length of any table item is 10 bytes.
uint8_t sw_data_arr[16][10];
uint8_t sr_data_arr[16][10];

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
    memcpy(&err_code, pkt->motors[0].val, sizeof(int32_t)); // Safe unaligned load
    Serial.print("  [ERR CODE]: "); Serial.println(err_code);
  }

  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write((uint8_t*)pkt, sizeof(UdpPacket));
  Udp.endPacket();
}

void sendError(UdpPacket* pkt, int32_t err_code, uint8_t motor_id = 255) {
  if (pkt->flags & 0x01) {
    // IGNORE_ERRORS flag set: do not bubble up error packet
    return;
  }
  pkt->cmd = 255;
  pkt->count = 1;
  pkt->motors[0].id = motor_id;
  memset(pkt->motors[0].val, 0, 10);
  int32_t temp = err_code;
  memcpy(pkt->motors[0].val, &temp, sizeof(int32_t)); // Safe unaligned store
  sendResponse(pkt);
}

// Check status packet error byte and report hardware error status (reg 70) if Alert bit is set
bool checkAndReportMotorError(UdpPacket* pkt, uint8_t id, uint8_t dxl_err) {
  if (dxl_err != 0) {
    if (id < 16) torque_enabled[id] = false;
    if (pkt->flags & 0x01) {
      // IGNORE_ERRORS flag set: do not report or abort
      return false;
    }
    int32_t err_code = -5;
    if (dxl_err & 0x80) {
      // Alert bit 7: Read Hardware Error Status (address 70, 1 byte)
      uint8_t hw_err = 0;
      drainDxlRx();
      dxl.read(id, 70, 1, &hw_err, 1, 10);
      err_code = -100 - hw_err;
    } else {
      err_code = -200 - dxl_err;
    }
    sendError(pkt, err_code, id);
    return true;
  }
  return false;
}

void handleRead(UdpPacket* pkt, bool isAll, bool useReadAddr = false) {
  bool ignore_errors = (pkt->flags & 0x01) != 0;
  uint8_t target_addr = useReadAddr ? pkt->read_addr : pkt->addr;
  uint8_t target_length = useReadAddr ? pkt->read_length : pkt->length;
  if (target_length > 10) target_length = 10;
  sr_infos.addr = target_addr;
  sr_infos.addr_length = target_length;
  
  if (isAll) {
    // Split into two batches of 8 motors to keep serial RX traffic under the 
    // SAMD21's 256-byte hardware ring buffer limit (each 8-motor batch of 10B is 160 bytes)
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
      if (checkAndReportMotorError(pkt, info_xels_sr[i].id, info_xels_sr[i].error)) {
        return;
      }
      pkt->motors[total_recv].id = info_xels_sr[i].id;
      memset(pkt->motors[total_recv].val, 0, 10);
      memcpy(pkt->motors[total_recv].val, sr_data_arr[i], target_length);
      total_recv++;
    }
    if (cnt1 < 8 && !ignore_errors) {
      sendError(pkt, -4, cnt1); // -4: syncRead timed out
      return;
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
      if (checkAndReportMotorError(pkt, info_xels_sr[i].id, info_xels_sr[i].error)) {
        return;
      }
      pkt->motors[total_recv].id = info_xels_sr[i].id;
      memset(pkt->motors[total_recv].val, 0, 10);
      memcpy(pkt->motors[total_recv].val, sr_data_arr[i + 8], target_length);
      total_recv++;
    }
    if (cnt2 < 8 && !ignore_errors) {
      sendError(pkt, -4, 8 + cnt2); // -4: syncRead timed out
      return;
    }
    
    pkt->count = total_recv;
    for (int i = total_recv; i < 16; i++) {
      pkt->motors[i].id = 255;
      memset(pkt->motors[i].val, 0, 10);
    }
    
    sendResponse(pkt);
  } else {
    sr_infos.xel_count = 0;
    for (int i = 0; i < pkt->count && i < 16; i++) {
      info_xels_sr[i].id = pkt->motors[i].id;
      info_xels_sr[i].p_recv_buf = sr_data_arr[i];
      sr_infos.xel_count++;
    }

    if (sr_infos.xel_count == 0) {
      if (!ignore_errors) sendError(pkt, -1); // -1: No IDs provided
      return;
    }
    sr_infos.is_info_changed = true;

    drainDxlRx();

    uint8_t recv_cnt = dxl.syncRead(&sr_infos);
    for (int i = 0; i < recv_cnt; i++) {
      if (checkAndReportMotorError(pkt, info_xels_sr[i].id, info_xels_sr[i].error)) {
        return;
      }
      pkt->motors[i].id = info_xels_sr[i].id;
      memset(pkt->motors[i].val, 0, 10);
      memcpy(pkt->motors[i].val, sr_data_arr[i], target_length);
    }
    if (recv_cnt < pkt->count && !ignore_errors) {
      sendError(pkt, -4, pkt->motors[recv_cnt].id);
      return;
    }
    pkt->count = recv_cnt;
    for (int i = recv_cnt; i < 16; i++) {
      pkt->motors[i].id = 255;
      memset(pkt->motors[i].val, 0, 10);
    }
    sendResponse(pkt);
  }
}

bool executeWrite(UdpPacket* pkt, bool isAll) {
  bool ignore_errors = (pkt->flags & 0x01) != 0;
  uint8_t write_len = pkt->length > 10 ? 10 : pkt->length;
  sw_infos.addr = pkt->addr;
  sw_infos.addr_length = write_len;
  sw_infos.xel_count = 0;
  
  // Pre-calculate safety checks to save CPU cycles inside the loop
  bool is_movement_cmd = (pkt->addr == 116 || pkt->addr == 104 || pkt->addr == 102 || pkt->addr == 100);
  bool is_torque_cmd = (pkt->addr == 64);
  
  if (isAll) {
    sw_infos.xel_count = 16;
    for (int i = 0; i < 16; i++) {
      if (is_movement_cmd && !torque_enabled[i] && !ignore_errors) {
        Serial.print("BLOCKED: Torque OFF for ID "); Serial.println(i);
        sendError(pkt, -2, i); // -2: Torque is off
        return false;
      }
      if (is_torque_cmd) torque_enabled[i] = (pkt->motors[i].val[0] != 0);
      
      memset(sw_data_arr[i], 0, 10);
      memcpy(sw_data_arr[i], pkt->motors[i].val, write_len);
      info_xels_sw[i].id = i; // For WRITE_ALL, assume IDs 0-15 sequentially
      info_xels_sw[i].p_data = sw_data_arr[i];
    }
  } else {
    for (int i = 0; i < pkt->count && i < 16; i++) {
      uint8_t id = pkt->motors[i].id;
      
      if (is_movement_cmd && id < 16 && !torque_enabled[id] && !ignore_errors) {
        Serial.print("BLOCKED: Torque OFF for ID "); Serial.println(id);
        sendError(pkt, -2, id); // -2: Torque is off
        return false;
      }
      if (is_torque_cmd && id < 16) torque_enabled[id] = (pkt->motors[i].val[0] != 0);
      
      memset(sw_data_arr[i], 0, 10);
      memcpy(sw_data_arr[i], pkt->motors[i].val, write_len);
      info_xels_sw[sw_infos.xel_count].id = id;
      info_xels_sw[sw_infos.xel_count].p_data = sw_data_arr[i];
      sw_infos.xel_count++;
    }
  }

  if (sw_infos.xel_count == 0) {
    if (!ignore_errors) sendError(pkt, -1);
    return false;
  }
  sw_infos.is_info_changed = true;

  if (dxl.syncWrite(&sw_infos)) {
    drainDxlRx();
    delayMicroseconds(500);  // bus / DIR settle & motor write processing
    drainDxlRx();
    return true;
  } else {
    int32_t err = dxl.getLastLibErrCode();
    if (!ignore_errors) sendError(pkt, err);
    return ignore_errors;
  }
}

void handleWrite(UdpPacket* pkt, bool isAll) {
  if (executeWrite(pkt, isAll)) {
    sendResponse(pkt);
  }
}

void handleWriteReadAll(UdpPacket* pkt) {
  if (executeWrite(pkt, true)) {
    handleRead(pkt, true, true);
  }
}

void handleWriteRead(UdpPacket* pkt) {
  if (executeWrite(pkt, false)) {
    handleRead(pkt, false, true);
  }
}

void handleReboot(UdpPacket* pkt) {
  uint8_t target = pkt->addr; // 0=BOARD, 1=MOTORS, 2=ALL

  if (target == 1 || target == 2) {
    if (pkt->count == 0) {
      dxl.reboot(DXL_BROADCAST_ID);
      for (int i = 0; i < 16; i++) {
        torque_enabled[i] = false;
      }
    } else {
      for (int i = 0; i < pkt->count && i < 16; i++) {
        uint8_t id = pkt->motors[i].id;
        dxl.reboot(id);
        if (id < 16) {
          torque_enabled[id] = false;
        }
      }
    }
  }

  sendResponse(pkt);

  if (target == 0 || target == 2) {
    delay(100);
    NVIC_SystemReset();
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
  
  // Only process if the packet is exactly the size of our UdpPacket struct (183 bytes)
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
    else if (pkt.cmd == 2) handleWriteReadAll(&pkt);
    else if (pkt.cmd == 3) handleRead(&pkt, false);
    else if (pkt.cmd == 4) handleWrite(&pkt, false);
    else if (pkt.cmd == 5) handleWriteRead(&pkt);
    else if (pkt.cmd == 254) handleReboot(&pkt);
    else sendError(&pkt, -3); // -3: Unknown command
    
  } else if (packetSize > 0) {
    // Drop invalid length packets immediately by draining all unread bytes
    while (Udp.available()) {
      Udp.read();
    }
  }
}
