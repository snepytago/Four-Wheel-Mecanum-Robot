/*
 * esp32_robot.ino  (esp32R - ESP32 GẮN TRÊN ROBOT, robot ID 29)
 * ----------------------------------------------------------------------
 * Hai chiều, độc lập nhau:
 *
 *   XUỐNG (ESP-NOW -> UART): nhận gói text từ gateway esp32C, thêm '\n',
 *       đẩy nguyên văn xuống STM32. Cố ý giữ "câm" — không parse, không
 *       đổi đơn vị. Mọi việc hiểu nội dung do pose_link.c phía STM32 làm.
 *
 *   LÊN (UART -> USB Serial): đọc telemetry STM32 gửi ra PC6 rồi in lên
 *       Serial Monitor, để xem log của STM32 mà không cần USB-TTL riêng.
 *       Dòng STM32 được đánh dấu "[STM]" để phân biệt với "[R29]".
 *
 * Chuỗi đường truyền:
 *   PC (Python) --Serial--> esp32C (gateway, đã có sẵn)
 *        gateway cắt "<ID>;" ở đầu, gửi phần còn lại tới MAC robot ID 29
 *   --ESP-NOW--> esp32R (file này) --UART 115200--> STM32 PC7 (USART6_RX)
 *   STM32 PC6 (USART6_TX) --UART--> esp32R --USB--> Serial Monitor
 *
 * Các payload nhận được từ gateway (nguyên văn, KÈM dấu '#' cuối):
 *   510.0;60.0;3.14159#                 pose: x_cm ; y_cm ; theta_rad
 *   START#      STOP#      WPCLR#
 *   WPLIST;100.0;80.0;120.0;60.0#
 *
 * Dây nối UART với STM32 (Nucleo-F401RE, USART6):
 *   ESP32 GPIO17 (TX2)  ->  STM32 PC7  (USART6_RX)   - lệnh đi xuống
 *   ESP32 GPIO16 (RX2)  <-  STM32 PC6  (USART6_TX)   - telemetry đi lên
 *   ESP32 GND            -  STM32 GND                - BẮT BUỘC
 * Cả hai đều 3.3V -> nối thẳng, không cần level shifter.
 *
 * MAC của board này phải khớp robot29Mac trong code gateway:
 *   14:33:5C:04:61:18
 * ----------------------------------------------------------------------
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ======================= CẤU HÌNH ======================================

#define ESPNOW_CHANNEL        1      // PHẢI giống ESPNOW_CHANNEL của gateway

#define STM32_UART_TX_PIN     17     // -> STM32 PC7 (USART6_RX)
#define STM32_UART_RX_PIN     16     // <- STM32 PC6 (USART6_TX)
#define STM32_UART_BAUD       115200

#define RX_TIMEOUT_MS         500    // quá lâu không có gói -> báo MAT

// --- Hiển thị log của STM32 trên Serial Monitor ---
// STM32 gửi ~100 dòng/giây (IMU 50Hz + POSE 50Hz + PLINK 5Hz). In hết sẽ
// trôi màn hình không đọc nổi, nên lọc bớt ngay tại đây.
#define FORWARD_STM32_LOG     1      // 0 = tắt hẳn, chỉ in log của esp32R
#define LOG_SHOW_IMU          0      // 1 = in cả dòng IMU (rất nhiều)
#define LOG_POSE_EVERY_N      10     // POSE 50Hz -> in 1 trong 10 dòng (5Hz)
                                     // đặt = 1 nếu muốn xem đủ 50Hz
// PLINK và các dòng "STM32:..." luôn in đủ, vì chúng thưa và quan trọng.

// Hàng đợi vòng: callback ESP-NOW chạy ở task khác với loop(), nếu ghi UART
// thẳng trong callback thì một gói dài có thể chặn task WiFi.
#define QUEUE_LEN             8
#define PAYLOAD_MAX           250    // giới hạn payload của ESP-NOW v1.0

#define STM_LINE_MAX          192

// ======================= HÀNG ĐỢI XUỐNG =================================

static char     q_buf[QUEUE_LEN][PAYLOAD_MAX + 2];
static uint16_t q_len[QUEUE_LEN];
static volatile uint16_t q_head = 0;
static volatile uint16_t q_tail = 0;

static volatile uint32_t g_rx_count   = 0;
static volatile uint32_t g_rx_bad     = 0;   // gói rỗng/quá dài/có ký tự lạ
static volatile uint32_t g_dropped    = 0;   // hàng đợi đầy
static volatile uint32_t g_last_rx_ms = 0;
static uint32_t g_fwd_count = 0;

// ======================= ĐỌC LOG TỪ STM32 ===============================

static char     stm_line[STM_LINE_MAX];
static uint16_t stm_len = 0;
static uint32_t g_stm_lines = 0;     // tổng số dòng đọc được từ STM32
static uint16_t g_pose_div  = 0;

// Chỉ nhận ASCII in được: gói ESP-NOW không có checksum nên đây là lớp lọc
// rẻ tiền duy nhất ở tầng này.
static bool payload_is_clean(const uint8_t *data, int len)
{
    if (len <= 0 || len > PAYLOAD_MAX) return false;

    for (int i = 0; i < len; i++) {
        char c = (char)data[i];
        if (c < 0x20 || c > 0x7E) return false;
    }
    return true;
}

static bool starts_with(const char *s, const char *prefix)
{
    while (*prefix) {
        if (*s++ != *prefix++) return false;
    }
    return true;
}

static void handle_stm32_line(const char *s)
{
    g_stm_lines++;

#if FORWARD_STM32_LOG
    if (starts_with(s, "IMU,")) {
    #if LOG_SHOW_IMU
        Serial.printf("[STM] %s\n", s);
    #endif
        return;
    }

    if (starts_with(s, "POSE,")) {
        if (++g_pose_div < LOG_POSE_EVERY_N) return;
        g_pose_div = 0;
        Serial.printf("[STM] %s\n", s);
        return;
    }

    // PLINK, STM32:BOOT, STM32:READY, STM32:RUNNING... - in đủ
    Serial.printf("[STM] %s\n", s);
#endif
}

static void pump_stm32_log(void)
{
    while (Serial2.available()) {
        char c = (char)Serial2.read();

        if (c == '\n' || c == '\r') {
            if (stm_len > 0) {
                stm_line[stm_len] = '\0';
                handle_stm32_line(stm_line);
                stm_len = 0;
            }
        } else if (stm_len < STM_LINE_MAX - 1) {
            stm_line[stm_len++] = c;
        } else {
            stm_len = 0;   // dòng quá dài -> bỏ, chờ dòng tiếp
        }
    }
}

// ======================= CALLBACK ESP-NOW ===============================

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
#else
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
#endif

    g_rx_count++;
    g_last_rx_ms = millis();

    if (!payload_is_clean(data, len)) {
        g_rx_bad++;
        return;
    }

    uint16_t next = (uint16_t)((q_head + 1) % QUEUE_LEN);
    if (next == q_tail) {
        g_dropped++;
        return;
    }

    memcpy(q_buf[q_head], data, len);
    q_buf[q_head][len] = '\0';
    q_len[q_head] = (uint16_t)len;
    q_head = next;
}

// ======================= SETUP / LOOP ====================================

void setup()
{
    Serial.begin(115200);
    delay(300);

    // Nới buffer RX: STM32 bắn ~3700 byte/giây, buffer mặc định 256 byte dễ
    // tràn nếu loop() bị chậm một nhịp.
    Serial2.setRxBufferSize(1024);
    Serial2.begin(STM32_UART_BAUD, SERIAL_8N1, STM32_UART_RX_PIN, STM32_UART_TX_PIN);

    WiFi.mode(WIFI_STA);
    delay(100);
    esp_wifi_set_ps(WIFI_PS_NONE);     // giống gateway: tắt power save
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
    Serial.println("esp32R - ROBOT 29 (cau ESP-NOW <-> STM32)");
    Serial.println("====================================");
    Serial.print("MAC BOARD NAY: ");
    Serial.println(WiFi.macAddress());
    Serial.println("(phai khop robot29Mac trong code gateway)");
    Serial.print("CHANNEL: ");
    Serial.println(ESPNOW_CHANNEL);
    Serial.println("[R29] = log cua board nay, [STM] = log cua STM32");
    Serial.println("CHO GOI TU GATEWAY...");
}

void loop()
{
    // ---- Chiều xuống: đẩy hàng đợi ESP-NOW xuống STM32 ----
    while (q_tail != q_head) {
        uint16_t idx = q_tail;

        Serial2.write((const uint8_t *)q_buf[idx], q_len[idx]);
        Serial2.write('\n');

        q_tail = (uint16_t)((idx + 1) % QUEUE_LEN);
        g_fwd_count++;
    }

    // ---- Chiều lên: đọc telemetry STM32 và in ra Serial Monitor ----
    pump_stm32_log();

    static uint32_t last_status_ms = 0;
    uint32_t now = millis();

    if (now - last_status_ms >= 1000) {
        last_status_ms = now;

        bool timed_out = (g_rx_count == 0) || ((now - g_last_rx_ms) > RX_TIMEOUT_MS);

        // stm = so dong doc duoc tu STM32. Bang 0 => day PC6 -> GPIO16 chua
        // thong hoac STM32 chua chay.
        Serial.printf("[R29] link=%s rx=%lu fwd=%lu bad=%lu drop=%lu stm=%lu\n",
                      timed_out ? "MAT" : "OK",
                      (unsigned long)g_rx_count,
                      (unsigned long)g_fwd_count,
                      (unsigned long)g_rx_bad,
                      (unsigned long)g_dropped,
                      (unsigned long)g_stm_lines);
    }
}
