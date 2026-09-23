#include "pose_link.h"
#include "config.h"
#include "usart6.h"
#include "stm32f4xx_hal.h"

#define RAD_TO_DEG   57.29577951f

static float    pose_x_m       = 0.0f;
static float    pose_y_m       = 0.0f;
static float    pose_theta_deg = 0.0f;

static uint32_t pose_last_ms   = 0;
static uint8_t  pose_fresh     = 0;
static uint8_t  pose_got_first = 0;

static uint32_t rx_ok    = 0;
static uint32_t rx_other = 0;
static uint32_t rx_err   = 0;

// Buffer dong dat o file-scope (khong nam tren stack): 128 byte trong ham
// duoc goi tu vong lap chinh la qua nhieu so voi _Min_Stack_Size mac dinh.
static char line_buf[USART6_RX_LINE_MAXLEN];

// Doc 1 so thuc dang [-]ddd[.ddd] tai *pp, day *pp toi ky tu ngay sau so do.
// Tu viet thay vi dung sscanf/strtof de khong keo them vai KB flash - dung
// dung cach usart6_send_float() da lam o chieu nguoc lai.
static uint8_t parse_float(const char **pp, float *out)
{
    const char *p = *pp;
    float sign = 1.0f;
    float val  = 0.0f;
    uint8_t digits = 0;

    if      (*p == '-') { sign = -1.0f; p++; }
    else if (*p == '+') {               p++; }

    while (*p >= '0' && *p <= '9') {
        val = val * 10.0f + (float)(*p - '0');
        p++;
        if (++digits > 9) return 0;
    }

    if (*p == '.') {
        p++;
        float scale = 0.1f;
        while (*p >= '0' && *p <= '9') {
            val += (float)(*p - '0') * scale;
            scale *= 0.1f;
            p++;
            if (++digits > 12) return 0;
        }
    }

    if (digits == 0) return 0;

    *out = sign * val;
    *pp  = p;
    return 1;
}

static uint8_t str_starts_with(const char *s, const char *prefix)
{
    while (*prefix) {
        if (*s++ != *prefix++) return 0;
    }
    return 1;
}

// "<x_cm>;<y_cm>;<theta_rad>" - ky tu '#' da bi ngat USART cat lam het dong
// nhung van chap nhan neu con sot lai.
static uint8_t parse_pose_line(const char *s)
{
    float x_cm, y_cm, th_rad;
    const char *p = s;

    if (!parse_float(&p, &x_cm))  return 0;
    if (*p++ != ';')              return 0;
    if (!parse_float(&p, &y_cm))  return 0;
    if (*p++ != ';')              return 0;
    if (!parse_float(&p, &th_rad)) return 0;
    if (*p != '\0' && *p != '#')  return 0;

    // Khong co checksum tren duong truyen -> chan goi phi ly bang vung hop le.
    if (x_cm < -POSE_XY_ABS_MAX_CM || x_cm > POSE_XY_ABS_MAX_CM) return 0;
    if (y_cm < -POSE_XY_ABS_MAX_CM || y_cm > POSE_XY_ABS_MAX_CM) return 0;
    if (th_rad < -7.0f || th_rad > 7.0f)                          return 0;

    pose_x_m       = x_cm * 0.01f;          // cm -> m
    pose_y_m       = y_cm * 0.01f;
    pose_theta_deg = th_rad * RAD_TO_DEG;   // rad -> do
    return 1;
}

// Lenh da biet nhung chua xu ly o buoc nay - dem rieng de khong lan vao rx_err.
static uint8_t is_known_command(const char *s)
{
    return (str_starts_with(s, "START")  ||
            str_starts_with(s, "STOP")   ||
            str_starts_with(s, "WPLIST") ||
            str_starts_with(s, "WPCLR")) ? 1 : 0;
}

void pose_link_init(void)
{
    pose_x_m = pose_y_m = pose_theta_deg = 0.0f;
    pose_last_ms   = 0;
    pose_fresh     = 0;
    pose_got_first = 0;
    rx_ok = rx_other = rx_err = 0;
}

void pose_link_poll(void)
{
    // Vet het hang doi moi chu ky: neu nhieu dong ve cung luc thi gia tri
    // cuoi cung (moi nhat) la cai duoc giu.
    while (usart6_rx_line_ready()) {
        usart6_rx_get_line(line_buf, sizeof(line_buf));

        if (line_buf[0] == '\0') continue;

        if (parse_pose_line(line_buf)) {
            pose_last_ms   = HAL_GetTick();
            pose_fresh     = 1;
            pose_got_first = 1;
            rx_ok++;
        } else if (is_known_command(line_buf)) {
            rx_other++;
        } else {
            rx_err++;
        }
    }
}

uint8_t pose_link_take_fresh(void)
{
    uint8_t f = pose_fresh;
    pose_fresh = 0;
    return f;
}

uint8_t pose_link_valid(void)
{
    if (!pose_got_first) return 0;
    return ((HAL_GetTick() - pose_last_ms) <= POSE_LINK_TIMEOUT_MS) ? 1 : 0;
}

uint32_t pose_link_age_ms(void)
{
    if (!pose_got_first) return 0xFFFFFFFFu;
    return HAL_GetTick() - pose_last_ms;
}

float pose_link_get_x(void)         { return pose_x_m; }
float pose_link_get_y(void)         { return pose_y_m; }
float pose_link_get_theta_deg(void) { return pose_theta_deg; }

uint32_t pose_link_get_rx_ok(void)    { return rx_ok; }
uint32_t pose_link_get_rx_other(void) { return rx_other; }
uint32_t pose_link_get_rx_err(void)   { return rx_err; }
