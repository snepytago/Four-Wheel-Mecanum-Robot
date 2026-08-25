/*
 * esp32_server.ino
 * ----------------------------------------------------------------------
 * Firmware cho ESP32 "SERVER" (đặt cạnh máy tính, KHÔNG gắn trên robot).
 *
 * Vai trò:
 *   1. Nhận dòng lệnh text qua cổng Serial/USB từ script bàn phím chạy
 *      trên PC (xem pc_teleop/teleop_keyboard.py).
 *   2. Đóng gói thành gói tin nhị phân và gửi sang "ESP32 trên robot"
 *      bằng ESP-NOW (không cần router/AP WiFi, độ trễ thấp).
 *   3. Watchdog: nếu quá SERIAL_TIMEOUT_MS không nhận được dòng lệnh mới
 *      nào từ PC (script bị đóng, rút cáp USB, PC treo...) thì tự động
 *      phát liên tục gói STOP (0,0,0) sang robot cho tới khi PC gửi lại.
 *      Đây là lớp an toàn thứ 1 trong 3 lớp (lớp 2 ở ESP32 robot, lớp 3
 *      ở chính STM32).
 *
 * Kết nối: ESP32 này cắm vào máy tính bằng cáp USB như nạp code bình
 * thường — sau khi nạp xong, KHÔNG cần rút ra, cứ để cắm USB, script
 * Python sẽ mở đúng cổng COM/tty đó để gửi lệnh.
 *
 * QUAN TRỌNG: phải điền đúng địa chỉ MAC của ESP32-trên-robot vào
 * ROBOT_MAC bên dưới trước khi nạp code này. Lấy MAC bằng cách nạp
 * esp32_robot.ino trước, mở Serial Monitor, nó sẽ tự in ra MAC của nó.
 * ----------------------------------------------------------------------
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ======================= CẤU HÌNH ======================================

// !!! SỬA địa chỉ MAC dưới đây thành MAC thật của ESP32 gắn trên robot !!!
// Lấy từ dòng "[ROBOT] MAC cua ESP32 nay: ..." khi nạp esp32_robot.ino.
static uint8_t ROBOT_MAC[6] = {0x14, 0x33, 0x5C, 0x04, 0x61, 0x18};

#define ESPNOW_WIFI_CHANNEL   1     // phải giống hệt esp32_robot.ino

#define SERIAL_BAUD           115200
// Nếu quá thời gian này (ms) không có dòng lệnh mới hợp lệ từ PC
// -> coi như mất kết nối PC, tự phát STOP sang robot.
#define SERIAL_TIMEOUT_MS     300

#define LOOP_PERIOD_MS        20    // 50Hz, khớp tần số gửi của PC
#define TELEOP_MAGIC          0xA5

// Giới hạn an toàn — chặn bớt nếu script PC lỡ gửi giá trị bất thường
// (không thay thế cho việc PC tự giới hạn, chỉ là lớp phòng hờ thêm).
#define VX_MAX_MM_S     1000
#define VY_MAX_MM_S     1000
#define WZ_MAX_MRAD_S   2000

// ======================= GIAO THỨC GÓI TIN ESP-NOW ======================
// PHẢI khớp 100% với struct trong esp32_robot.ino.

typedef struct __attribute__((packed)) {
  uint8_t  magic;
  uint32_t seq;
  int16_t  vx_mm_s;
  int16_t  vy_mm_s;
  int16_t  wz_mrad_s;
  uint8_t  checksum;
} teleop_packet_t;

static uint8_t calc_checksum(const teleop_packet_t *p) {
  const uint8_t *b = (const uint8_t *)p;
  uint8_t x = 0;
  for (size_t i = 0; i < sizeof(teleop_packet_t) - sizeof(p->checksum); i++) {
    x ^= b[i];
  }
  return x;
}

// ======================= BIẾN TOÀN CỤC ===================================

static uint32_t g_seq = 0;
static uint32_t g_last_serial_ms = 0;
static int16_t  g_vx_mm_s = 0, g_vy_mm_s = 0, g_wz_mrad_s = 0;
static char     g_line_buf[64];
static uint8_t  g_line_len = 0;

// ======================= GỬI GÓI ESP-NOW =================================

static void send_packet(int16_t vx, int16_t vy, int16_t wz) {
  teleop_packet_t pkt;
  pkt.magic = TELEOP_MAGIC;
  pkt.seq = g_seq++;
  pkt.vx_mm_s = vx;
  pkt.vy_mm_s = vy;
  pkt.wz_mrad_s = wz;
  pkt.checksum = calc_checksum(&pkt);

  esp_now_send(ROBOT_MAC, (uint8_t *)&pkt, sizeof(pkt));
}

// ======================= PARSE DÒNG LỆNH TỪ PC ===========================
// Format:  V,<vx_mm>,<vy_mm>,<wz_mrad>\n   (số nguyên, có thể âm)
// Trả về true nếu parse thành công và ghi kết quả vào vx/vy/wz.

static bool parse_line(const char *line, int16_t *vx, int16_t *vy, int16_t *wz) {
  if (line[0] != 'V' || line[1] != ',') return false;

  int a, b, c;
  int n = sscanf(line + 2, "%d,%d,%d", &a, &b, &c);
  if (n != 3) return false;

  if (a < -32768 || a > 32767) return false;
  if (b < -32768 || b > 32767) return false;
  if (c < -32768 || c > 32767) return false;

  *vx = (int16_t)a;
  *vy = (int16_t)b;
  *wz = (int16_t)c;
  return true;
}

static int16_t clampi(int16_t v, int16_t lo, int16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// ======================= SETUP / LOOP ====================================

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.print("[SERVER] MAC cua ESP32 nay (khong can dung toi): ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[SERVER] LOI: esp_now_init that bai!");
    while (1) delay(1000);
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, ROBOT_MAC, 6);
  peer.channel = ESPNOW_WIFI_CHANNEL;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[SERVER] LOI: khong them duoc peer (kiem tra ROBOT_MAC)!");
  }

  Serial.println("[SERVER] San sang. Cho lenh 'V,vx,vy,wz' tu PC qua Serial...");
  g_last_serial_ms = millis();
}

void loop() {
  // ---- Đọc byte từ PC, gom thành dòng ----
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (g_line_len > 0) {
        g_line_buf[g_line_len] = '\0';
        int16_t vx, vy, wz;
        if (parse_line(g_line_buf, &vx, &vy, &wz)) {
          g_vx_mm_s   = clampi(vx, -VX_MAX_MM_S, VX_MAX_MM_S);
          g_vy_mm_s   = clampi(vy, -VY_MAX_MM_S, VY_MAX_MM_S);
          g_wz_mrad_s = clampi(wz, -WZ_MAX_MRAD_S, WZ_MAX_MRAD_S);
          g_last_serial_ms = millis();
        }
        g_line_len = 0;
      }
    } else if (g_line_len < sizeof(g_line_buf) - 1) {
      g_line_buf[g_line_len++] = c;
    } else {
      // dòng quá dài -> bỏ, chờ ký tự xuống dòng tiếp theo
      g_line_len = 0;
    }
  }

  // ---- Gửi định kỳ sang ESP32 robot (đóng vai trò heartbeat) ----
  static uint32_t last_loop_ms = 0;
  static uint32_t last_status_ms = 0;
  uint32_t now = millis();
  if (now - last_loop_ms < LOOP_PERIOD_MS) return;
  last_loop_ms = now;

  bool timed_out = (now - g_last_serial_ms) > SERIAL_TIMEOUT_MS;

  if (timed_out) {
    send_packet(0, 0, 0);
  } else {
    send_packet(g_vx_mm_s, g_vy_mm_s, g_wz_mrad_s);
  }

  if (now - last_status_ms >= 1000) {
    last_status_ms = now;
    Serial.printf("[SERVER] pc_link=%s seq=%lu v=(%d,%d,%d)\n",
                  timed_out ? "MAT" : "OK",
                  (unsigned long)g_seq,
                  g_vx_mm_s, g_vy_mm_s, g_wz_mrad_s);
  }
}
