/*
 * esp32_robot.ino
 * ----------------------------------------------------------------------
 * Firmware cho ESP32 GẮN TRÊN ROBOT (mecanum 4 bánh, STM32F401RE).
 *
 * Vai trò:
 *   1. Nhận gói tin điều khiển (vx, vy, wz) từ "ESP32 server" qua ESP-NOW.
 *   2. Chuyển tiếp lệnh xuống STM32 bằng UART (dòng text CSV), STM32 sẽ
 *      parse dòng này và gọi robot_set_velocity(vx, vy, wz).
 *   3. Có watchdog riêng: nếu quá ESPNOW_TIMEOUT_MS không nhận được gói
 *      hợp lệ nào (mất sóng, ESP32 server tắt, v.v.) thì tự động gửi
 *      lệnh STOP (0,0,0) xuống STM32 liên tục cho tới khi có tín hiệu
 *      trở lại. Đây là lớp an toàn thứ 2 (lớp 1 nằm ở ESP32 server, lớp
 *      3 nằm ở chính STM32 — xem stm32_patch/).
 *
 * Dây nối UART xuống STM32 (Nucleo-F401RE, USART6):
 *   ESP32 GPIO17 (TX2)  ->  STM32 PC7  (USART6_RX, cần thêm ở stm32_patch)
 *   ESP32 GPIO16 (RX2)  <-  STM32 PC6  (USART6_TX, đã có sẵn — dùng để
 *                                        board debug/telemetry, không bắt
 *                                        buộc phải nối nếu chưa cần đọc
 *                                        telemetry từ ESP32)
 *   ESP32 GND            -  STM32 GND  (BẮT BUỘC phải nối chung GND)
 *
 * Cả ESP32 và STM32 (Nucleo) đều dùng mức logic 3.3V -> nối thẳng,
 * KHÔNG cần mạch chia áp/level shifter.
 *
 * Baud UART xuống STM32: 115200 (khớp với usart6 hiện tại của FWMR).
 *
 * Thư viện: chỉ dùng ESP-NOW + WiFi có sẵn trong ESP32 Arduino core,
 * không cần cài thêm thư viện ngoài.
 * ----------------------------------------------------------------------
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ======================= CẤU HÌNH ======================================

// Kênh WiFi cố định dùng cho ESP-NOW — PHẢI giống hệt ESP32 server.
#define ESPNOW_WIFI_CHANNEL   1

// Chân UART nối xuống STM32 (dùng UART2 phần cứng của ESP32).
#define STM32_UART_TX_PIN     17   // -> STM32 PC7 (USART6_RX)
#define STM32_UART_RX_PIN     16   // <- STM32 PC6 (USART6_TX)
#define STM32_UART_BAUD       115200

// Nếu quá thời gian này (ms) không nhận được gói ESP-NOW hợp lệ nào
// -> coi như mất kết nối, tự gửi lệnh dừng xuống STM32.
#define ESPNOW_TIMEOUT_MS     300

// Tần số lặp vòng chính (đồng thời là tần số tối đa gửi lệnh xuống STM32).
#define LOOP_PERIOD_MS        20    // 50Hz

// Magic byte để nhận diện gói tin hợp lệ (chống nhiễu byte rác).
#define TELEOP_MAGIC          0xA5

// ======================= GIAO THỨC GÓI TIN ESP-NOW =====================
// PHẢI khớp 100% với struct trong esp32_server.ino (cùng thứ tự field,
// cùng kiểu dữ liệu, có __attribute__((packed)) để không bị compiler
// chèn padding khác nhau giữa 2 board).

typedef struct __attribute__((packed)) {
  uint8_t  magic;        // luôn = TELEOP_MAGIC
  uint32_t seq;          // số thứ tự gói, dùng để debug/đếm rớt gói
  int16_t  vx_mm_s;      // vận tốc dọc thân xe,  đơn vị mm/s
  int16_t  vy_mm_s;      // vận tốc ngang thân xe, đơn vị mm/s
  int16_t  wz_mrad_s;    // vận tốc góc quay,      đơn vị mrad/s
  uint8_t  checksum;     // XOR toàn bộ các byte phía trước
} teleop_packet_t;

static uint8_t calc_checksum(const teleop_packet_t *p) {
  const uint8_t *b = (const uint8_t *)p;
  uint8_t x = 0;
  // XOR tất cả các byte trước trường checksum
  for (size_t i = 0; i < sizeof(teleop_packet_t) - sizeof(p->checksum); i++) {
    x ^= b[i];
  }
  return x;
}

// ======================= BIẾN TOÀN CỤC ==================================

HardwareSerial STM32Serial(2);   // UART2 phần cứng của ESP32

volatile int16_t g_vx_mm_s   = 0;
volatile int16_t g_vy_mm_s   = 0;
volatile int16_t g_wz_mrad_s = 0;
volatile uint32_t g_last_rx_ms   = 0;
volatile uint32_t g_last_seq     = 0;
volatile uint32_t g_rx_count     = 0;
volatile uint32_t g_checksum_err = 0;
volatile bool     g_link_ok      = false;

// ======================= CALLBACK ESP-NOW ===============================

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
#else
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
#endif
  if (len != sizeof(teleop_packet_t)) return;

  teleop_packet_t pkt;
  memcpy(&pkt, data, sizeof(pkt));

  if (pkt.magic != TELEOP_MAGIC) return;
  if (calc_checksum(&pkt) != pkt.checksum) {
    g_checksum_err++;
    return;
  }

  g_vx_mm_s   = pkt.vx_mm_s;
  g_vy_mm_s   = pkt.vy_mm_s;
  g_wz_mrad_s = pkt.wz_mrad_s;
  g_last_seq  = pkt.seq;
  g_last_rx_ms = millis();
  g_rx_count++;
  g_link_ok = true;
}

// ======================= GỬI LỆNH XUỐNG STM32 ===========================
// Dòng text CSV, kết thúc bằng '\n'. Format:  V,<vx_mm>,<vy_mm>,<wz_mrad>
// Đây CÙNG format với dòng gửi từ PC -> ESP32 server (xem README) nên có
// thể test STM32 độc lập bằng cách gõ tay dòng này qua terminal UART.

void send_to_stm32(int16_t vx_mm, int16_t vy_mm, int16_t wz_mrad) {
  STM32Serial.printf("V,%d,%d,%d\n", vx_mm, vy_mm, wz_mrad);
}

// ======================= SETUP / LOOP ====================================

void setup() {
  Serial.begin(115200);           // debug qua USB
  delay(200);

  STM32Serial.begin(STM32_UART_BAUD, SERIAL_8N1, STM32_UART_RX_PIN, STM32_UART_TX_PIN);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.print("[ROBOT] MAC cua ESP32 nay: ");
  Serial.println(WiFi.macAddress());
  Serial.println("[ROBOT] -> copy dia chi MAC nay vao ROBOT_MAC trong esp32_server.ino");

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ROBOT] LOI: esp_now_init that bai!");
    while (1) delay(1000);
  }

  esp_now_register_recv_cb(onDataRecv);

  Serial.println("[ROBOT] San sang, cho lenh tu ESP32 server...");
}

void loop() {
  static uint32_t last_loop_ms = 0;
  static uint32_t last_status_ms = 0;
  uint32_t now = millis();

  if (now - last_loop_ms < LOOP_PERIOD_MS) return;
  last_loop_ms = now;

  bool timed_out = (now - g_last_rx_ms) > ESPNOW_TIMEOUT_MS;

  int16_t vx = 0, vy = 0, wz = 0;
  if (!timed_out && g_link_ok) {
    vx = g_vx_mm_s;
    vy = g_vy_mm_s;
    wz = g_wz_mrad_s;
  } else {
    // Mất tín hiệu ESP-NOW -> ép về 0, KHÔNG dùng lệnh cũ.
    vx = vy = wz = 0;
    g_link_ok = false;
  }

  send_to_stm32(vx, vy, wz);

  // In trạng thái debug 1 lần/giây, không spam Serial USB.
  if (now - last_status_ms >= 1000) {
    last_status_ms = now;
    Serial.printf("[ROBOT] link=%s seq=%lu rx=%lu ck_err=%lu v=(%d,%d,%d)\n",
                  timed_out ? "MAT" : "OK",
                  (unsigned long)g_last_seq,
                  (unsigned long)g_rx_count,
                  (unsigned long)g_checksum_err,
                  vx, vy, wz);
  }
}
