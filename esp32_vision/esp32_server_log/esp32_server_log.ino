/*
 * esp32_server_log.ino  (esp32Server - ESP32 CẮM USB VÀO MÁY TÍNH)
 * ----------------------------------------------------------------------
 * Nhận log STM32 mà esp32R trên robot gửi sang bằng ESP-NOW rồi in lên
 * Serial Monitor (115200). Robot chạy không cần dây, vẫn xem được log.
 *
 *   STM32 PC6 --UART--> esp32R (robot 29) --ESP-NOW--> board này --USB--> PC
 *
 * MAC board này: 94:E6:86:3A:C3:88 (phải khớp SERVER_MAC trong
 * esp32_vision/esp32_robot/esp32_robot.ino).
 * Chỉ nhận gói từ esp32R (14:33:5C:04:61:18), bỏ qua mọi nguồn khác.
 * Kênh WiFi phải giống esp32R và gateway: ESPNOW_CHANNEL = 1.
 * ----------------------------------------------------------------------
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define ESPNOW_CHANNEL     1
#define SERIAL_BAUD        115200
#define LINK_TIMEOUT_MS    3000   // quá lâu không có gói -> in cảnh báo MAT

static const uint8_t ROBOT_MAC[6] = {0x14, 0x33, 0x5C, 0x04, 0x61, 0x18};

// Callback ESP-NOW chạy ở task WiFi: chỉ chép vào hàng đợi, in ở loop().
#define QUEUE_LEN          32
#define PAYLOAD_MAX        250

static char     q_buf[QUEUE_LEN][PAYLOAD_MAX + 1];
static volatile uint16_t q_head = 0;
static volatile uint16_t q_tail = 0;

static volatile uint32_t g_rx      = 0;
static volatile uint32_t g_dropped = 0;   // hàng đợi đầy
static volatile uint32_t g_other   = 0;   // gói từ MAC khác
static volatile uint32_t g_last_ms = 0;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    const uint8_t *mac = info->src_addr;
#else
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
#endif
    if (memcmp(mac, ROBOT_MAC, 6) != 0) { g_other++; return; }
    if (len <= 0) return;
    if (len > PAYLOAD_MAX) len = PAYLOAD_MAX;

    uint16_t next = (uint16_t)((q_head + 1) % QUEUE_LEN);
    if (next == q_tail) { g_dropped++; return; }

    memcpy(q_buf[q_head], data, len);
    q_buf[q_head][len] = '\0';
    q_head = next;

    g_rx++;
    g_last_ms = millis();
}

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(300);

    WiFi.mode(WIFI_STA);
    delay(100);
    esp_wifi_set_ps(WIFI_PS_NONE);
    WiFi.disconnect();

    if (esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
        Serial.println(">>> LOI DAT WIFI CHANNEL");
    }

    if (esp_now_init() != ESP_OK) {
        Serial.println(">>> ESP-NOW INIT FAILED");
        while (true) delay(1000);
    }
    esp_now_register_recv_cb(onDataRecv);

    Serial.println();
    Serial.println("====================================");
    Serial.println("esp32Server - nhan log STM32 tu robot 29");
    Serial.println("====================================");
    Serial.print("MAC BOARD NAY: ");
    Serial.println(WiFi.macAddress());
    Serial.println("(phai la 94:E6:86:3A:C3:88)");
    Serial.println("CHO LOG TU esp32R...");
}

void loop()
{
    while (q_tail != q_head) {
        Serial.print("[STM] ");
        Serial.println(q_buf[q_tail]);
        q_tail = (uint16_t)((q_tail + 1) % QUEUE_LEN);
    }

    // Chỉ báo khi mất liên lạc, để không chen vào giữa log.
    static uint32_t last_warn_ms = 0;
    uint32_t now = millis();
    bool lost = (g_rx == 0) || ((now - g_last_ms) > LINK_TIMEOUT_MS);
    if (lost && (now - last_warn_ms) >= LINK_TIMEOUT_MS) {
        last_warn_ms = now;
        Serial.printf("[SRV] chua co log tu robot (rx=%lu drop=%lu khac=%lu)\n",
                      (unsigned long)g_rx, (unsigned long)g_dropped,
                      (unsigned long)g_other);
    }
}
