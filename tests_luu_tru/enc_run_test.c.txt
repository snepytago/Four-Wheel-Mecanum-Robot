#include "enc_run_test.h"
#include "config.h"
#include "encoder.h"
#include "motor.h"
#include "mpu6050.h"
#include "robot_control.h"
#include "usart6.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <math.h>

#if ENC_RUN_IMU_HOLD && !ENC_RUN_EQUAL_DUTY
#error "ENC_RUN_IMU_HOLD chi dung voi ENC_RUN_EQUAL_DUTY = 1"
#endif

#define RAD2DEG  57.29577951f
#define DEG2RAD  0.017453293f

static const char *WNAME[4] = { "FL", "FR", "RL", "RR" };

static int32_t cnt[4];              // xung luy ke tu luc xuat phat
static int32_t win[4];              // xung trong cua so odometry hien tai
static float   pos_x, pos_y, th;    // m, m, rad - chi tu encoder
static float   imu_x, imu_y;        // m - encoder + huong IMU
static float   yaw_prev;            // rad - yaw IMU o lan odo_step truoc
static uint8_t g_imu;               // 1 = co MPU6050

// Vong giu huong
static float   hh_i, hh_u;          // phan tich phan, hieu chinh hien tai (duty)
static float   yaw_max_abs;         // |yaw| lon nhat trong luc chay (do)
static float   u_sum;               // de tinh u trung binh luc on dinh
static uint32_t u_n;

static float imu_yaw_deg(void) { return g_imu ? mpu6050_get_yaw_deg() : 0.0f; }
static float imu_gz_dps(void)  { return g_imu ? mpu6050_get_gyro_z_dps() : 0.0f; }

// Doc encoder, cong don. Tra ve 1 neu co banh nao vua nhich.
static uint8_t poll_encoders(void)
{
    int32_t d[4];
    uint8_t moved = 0;
    encoder_get_deltas(d);
    for (int i = 0; i < 4; i++) {
        cnt[i] += d[i];
        win[i] += d[i];
        if (d[i] != 0) moved = 1;
    }
    return moved;
}

// Odometry: goc quay banh trong cua so -> do dich than xe (FK), tich phan theo
// goc trung diem. 2 ban song song: huong tu encoder, va huong tu IMU.
static void odo_step(void)
{
    const float k = TWO_PI / COUNTS_PER_REV;
    float fl = win[0] * k, fr = win[1] * k, rl = win[2] * k, rr = win[3] * k;
    for (int i = 0; i < 4; i++) win[i] = 0;

    float dxb = (WHEEL_R / 4.0f) * ( fl + fr + rl + rr);
    float dyb = (WHEEL_R / 4.0f) * (-fl + fr + rl - rr);
    float dth = (WHEEL_R / (4.0f * K_GEOM)) * (-fl + fr - rl + rr);

    float tm = th + 0.5f * dth;
    pos_x += dxb * cosf(tm) - dyb * sinf(tm);
    pos_y += dxb * sinf(tm) + dyb * cosf(tm);
    th    += dth;

    float yaw = imu_yaw_deg() * DEG2RAD;
    float ym  = 0.5f * (yaw + yaw_prev);
    imu_x += dxb * cosf(ym) - dyb * sinf(ym);
    imu_y += dxb * sinf(ym) + dyb * cosf(ym);
    yaw_prev = yaw;
}

static int32_t max_cnt(void)
{
    int32_t m = cnt[0];
    for (int i = 1; i < 4; i++) if (cnt[i] > m) m = cnt[i];
    return m;
}

static float cnt_to_mm(int32_t c)
{
    return (float)c * (TWO_PI * WHEEL_R * 1000.0f) / COUNTS_PER_REV;
}

static void send_mm(float m)  { usart6_send_int((int32_t)lroundf(m * 1000.0f)); }

static void send_counts(const int32_t c[4])
{
    for (int i = 0; i < 4; i++) {
        usart6_send_char(' ');
        usart6_send_string(WNAME[i]);
        usart6_send_char('=');
        usart6_send_int(c[i]);
    }
}

// v: m/s o che do van toc, duty o che do duty chung
static void log_line(uint32_t t, float v)
{
    usart6_send_string("ENC,");
    usart6_send_int((int32_t)t);          usart6_send_char(',');
#if ENC_RUN_EQUAL_DUTY
    usart6_send_int((int32_t)v);
#else
    usart6_send_float(v, 3);
#endif
    for (int i = 0; i < 4; i++) { usart6_send_char(','); usart6_send_int(cnt[i]); }
    usart6_send_char(','); send_mm(pos_x);
    usart6_send_char(','); send_mm(pos_y);
    usart6_send_char(','); usart6_send_float(th * RAD2DEG, 2);
    usart6_send_char(','); usart6_send_float(imu_yaw_deg(), 2);
    usart6_send_char(','); usart6_send_float(imu_gz_dps(), 2);
    usart6_send_char(','); usart6_send_float(hh_u, 1);
    usart6_send_char(','); send_mm(imu_x);
    usart6_send_char(','); send_mm(imu_y);
    usart6_send_string("\r\n");
}

#if !ENC_RUN_EQUAL_DUTY
static float v_ramp(uint32_t t)
{
#if ENC_RUN_RAMP_MS > 0
    if (t < ENC_RUN_RAMP_MS)
        return ROBOT_V_MIN + (ENC_RUN_V - ROBOT_V_MIN) * (float)t / (float)ENC_RUN_RAMP_MS;
#endif
    (void)t;
    return ENC_RUN_V;
}

static float wz_hold(void)
{
#if ENC_RUN_HEADING_HOLD
    float wz = HEADING_KP * (THETA_TARGET_DEG - th * RAD2DEG);
    if (wz >  HEADING_MAX_WZ) wz =  HEADING_MAX_WZ;
    if (wz < -HEADING_MAX_WZ) wz = -HEADING_MAX_WZ;
    return wz;
#else
    return 0.0f;
#endif
}
#endif

#if ENC_RUN_EQUAL_DUTY
static int32_t duty_ramp(uint32_t t)
{
#if ENC_RUN_RAMP_MS > 0
    if (t < ENC_RUN_RAMP_MS)
        return ENC_RUN_DUTY_START + (int32_t)((ENC_RUN_DUTY - ENC_RUN_DUTY_START) * (int32_t)t / (int32_t)ENC_RUN_RAMP_MS);
#endif
    (void)t;
    return ENC_RUN_DUTY;
}

// Lenh cho 1 chu ky (goi SAU mpu6050_update). Tra ve duty nen de ghi log.
static float apply_cmd(uint32_t t, float dt)
{
    int32_t d = duty_ramp(t);
#if ENC_RUN_IMU_HOLD
    float e = 0.0f - imu_yaw_deg();

    if (t >= ENC_RUN_RAMP_MS) {                     // khong tich phan luc ramp
        hh_i += HH_KI * e * dt;
        if (hh_i >  HH_I_MAX) hh_i =  HH_I_MAX;
        if (hh_i < -HH_I_MAX) hh_i = -HH_I_MAX;
    }

    float u = HH_KP * e + hh_i - HH_KD * imu_gz_dps();
    if (u >  HH_U_MAX) u =  HH_U_MAX;
    if (u < -HH_U_MAX) u = -HH_U_MAX;
    hh_u = u;

    if (t >= 1000u) { u_sum += u; u_n++; }

    int32_t du = (int32_t)lroundf(u);
    motor_set_duty_raw(d - du, d + du, d - du, d + du);   // FL, FR, RL, RR
#else
    (void)dt;
    motor_set_duty_raw(d, d, d, d);
#endif
    return (float)d;
}
#endif

void enc_run_test_run(uint8_t imu_ready)
{
    const int32_t target = ENC_RUN_TARGET_COUNTS;

    g_imu = imu_ready;
    robot_set_velocity(0.0f, 0.0f, 0.0f);

#if ENC_RUN_IMU_HOLD
    if (!g_imu) {
        while (1) {
            usart6_send_string("ENCRUN:LOI khong co MPU6050 - che do giu huong IMU khong chay duoc\r\n");
            HAL_Delay(2000);
        }
    }
#endif

    usart6_send_string("ENCRUN:START muc_tieu=");
    usart6_send_int(target);
    usart6_send_string(" xung (");
    usart6_send_int((int32_t)lroundf(ENC_RUN_DIST_M * 1000.0f));
    usart6_send_string(" mm)");
#if ENC_RUN_EQUAL_DUTY
    usart6_send_string(" | CHE DO DUTY CHUNG: duty=");
    usart6_send_int(ENC_RUN_DUTY);
  #if ENC_RUN_IMU_HOLD
    usart6_send_string(" | GIU HUONG IMU: KP=");  usart6_send_float(HH_KP, 2);
    usart6_send_string(" KI=");                   usart6_send_float(HH_KI, 2);
    usart6_send_string(" KD=");                   usart6_send_float(HH_KD, 2);
  #else
    usart6_send_string(" | KHONG giu huong");
  #endif
#else
    usart6_send_string(" v=");
    usart6_send_float(ENC_RUN_V, 3);
    usart6_send_string(" m/s giu_huong=");
    usart6_send_int(ENC_RUN_HEADING_HOLD);
#endif
    usart6_send_string("\r\n");

    for (uint32_t s = ENC_RUN_START_DELAY_MS / 1000u; s > 0; s--) {
        usart6_send_string("ENCRUN:chay sau ");
        usart6_send_int((int32_t)s);
        usart6_send_string(" s\r\n");
        HAL_Delay(1000);
    }

    encoder_reset_all();
    int32_t dummy[4];
    encoder_get_deltas(dummy);   // dong bo bien nho ben trong sau khi reset
    for (int i = 0; i < 4; i++) { cnt[i] = 0; win[i] = 0; }
    pos_x = pos_y = th = 0.0f;
    imu_x = imu_y = yaw_prev = 0.0f;
    hh_i = hh_u = yaw_max_abs = u_sum = 0.0f;
    u_n = 0;
    if (g_imu) mpu6050_set_yaw_deg(0.0f);

#if ENC_RUN_EQUAL_DUTY
    usart6_send_string("ENC,t_ms,duty,FL,FR,RL,RR,x_mm,y_mm,th_deg,yaw_deg,gz_dps,u,xi_mm,yi_mm\r\n");
#else
    usart6_send_string("ENC,t_ms,v_cmd,FL,FR,RL,RR,x_mm,y_mm,th_deg,yaw_deg,gz_dps,u,xi_mm,yi_mm\r\n");
#endif

    // ---------------- Pha chay ----------------
    const uint32_t t0 = HAL_GetTick();
    uint32_t last_ctl = t0;
    uint32_t last_log = t0 - ENC_RUN_LOG_EVERY_MS;
    int      hit = -1;
    uint32_t t_hit = 0;
#if ENC_RUN_EQUAL_DUTY
    float    v = apply_cmd(0, 0.0f);
#else
    float    v = v_ramp(0);
    robot_set_velocity(v, 0.0f, 0.0f);
#endif

    while (1) {
        poll_encoders();
        uint32_t now = HAL_GetTick();
        uint32_t t   = now - t0;

        // Kiem tra moc ngay sau moi lan doc encoder, khong cho chu ky 20 ms
        for (int i = 0; i < 4; i++) {
            if (cnt[i] >= target) { hit = i; break; }
        }
        if (hit >= 0 || t >= ENC_RUN_TIMEOUT_MS) {
            robot_set_velocity(0.0f, 0.0f, 0.0f);
            t_hit = t;
            break;
        }

        if ((now - last_ctl) >= CONTROL_DT_MS) {
            float dt = (float)(now - last_ctl) / 1000.0f;
            last_ctl = now;
            if (g_imu) {
                mpu6050_update(dt);
                float a = fabsf(imu_yaw_deg());
                if (a > yaw_max_abs) yaw_max_abs = a;
            }
            odo_step();
#if ENC_RUN_EQUAL_DUTY
            v = apply_cmd(t, dt);
#else
            v = v_ramp(t);
            robot_set_velocity(v, 0.0f, wz_hold());
#endif
        }

        if ((now - last_log) >= ENC_RUN_LOG_EVERY_MS &&
            max_cnt() < target - ENC_RUN_LOG_GUARD) {
            last_log = now;
            log_line(t, v);
        }
    }

    // Anh chup luc cham moc
    hh_u = 0.0f;
    odo_step();
    int32_t cnt_hit[4];
    for (int i = 0; i < 4; i++) cnt_hit[i] = cnt[i];
    float x_hit = pos_x, y_hit = pos_y, th_hit = th;
    float xi_hit = imu_x, yi_hit = imu_y, yaw_hit = imu_yaw_deg();

    log_line(t_hit, 0.0f);
    if (hit >= 0) {
        usart6_send_string("ENCRUN:CHAM_MOC banh=");
        usart6_send_string(WNAME[hit]);
    } else {
        usart6_send_string("ENCRUN:HET_GIO");
    }
    usart6_send_string(" t_ms=");
    usart6_send_int((int32_t)t_hit);
    usart6_send_string("\r\n");

    // ---------------- Pha troi den khi dung han ----------------
    uint32_t t_still = HAL_GetTick();
    last_ctl = t_still;
    last_log = t_still;
    while (1) {
        uint32_t now = HAL_GetTick();
        if (poll_encoders()) t_still = now;
        if ((now - last_ctl) >= CONTROL_DT_MS) {
            float dt = (float)(now - last_ctl) / 1000.0f;
            last_ctl = now;
            if (g_imu) mpu6050_update(dt);
            odo_step();
        }
        if ((now - last_log) >= ENC_RUN_LOG_EVERY_MS) { last_log = now; log_line(now - t0, 0.0f); }
        if ((now - t_still) >= ENC_RUN_REST_MS) break;
        if ((now - t0 - t_hit) >= ENC_RUN_COAST_MAX_MS) break;
    }
    odo_step();
    uint32_t t_rest = t_still - t0;   // lan cuoi con thay xung
    float yaw_end = imu_yaw_deg();

    // ---------------- Tong ket, in lai moi 5 s ----------------
    while (1) {
        usart6_send_string("===== KET QUA =====\r\n");

        usart6_send_string("ket_qua: ");
        if (hit >= 0) { usart6_send_string("CHAM_MOC banh "); usart6_send_string(WNAME[hit]); }
        else          { usart6_send_string("HET_GIO"); }
        usart6_send_string("  muc_tieu=");
        usart6_send_int(target);
#if ENC_RUN_EQUAL_DUTY
        usart6_send_string(" xung  duty_chung=");
        usart6_send_int(ENC_RUN_DUTY);
        usart6_send_string("\r\n");
#else
        usart6_send_string(" xung  v_cmd=");
        usart6_send_float(ENC_RUN_V, 3);
        usart6_send_string(" m/s\r\n");
#endif

        usart6_send_string("thoi_gian_chay_ms=");
        usart6_send_int((int32_t)t_hit);
        usart6_send_string("  (bat dong co -> cham moc)\r\n");
        usart6_send_string("thoi_gian_toi_luc_dung_han_ms=");
        usart6_send_int((int32_t)t_rest);
        usart6_send_string("  troi_ms=");
        usart6_send_int((int32_t)(t_rest > t_hit ? t_rest - t_hit : 0));
        usart6_send_string("\r\n");

        usart6_send_string("xung_luc_cham_moc:");
        send_counts(cnt_hit);
        usart6_send_string("\r\n");
        usart6_send_string("xung_cuoi_cung:  ");
        send_counts(cnt);
        usart6_send_string("\r\n");

        usart6_send_string("quang_duong_banh_mm:");
        for (int i = 0; i < 4; i++) {
            usart6_send_char(' ');
            usart6_send_string(WNAME[i]);
            usart6_send_char('=');
            usart6_send_int((int32_t)lroundf(cnt_to_mm(cnt[i])));
        }
        usart6_send_string("\r\n");

        usart6_send_string("toa_do_luc_cham_moc: x=");  send_mm(x_hit);
        usart6_send_string(" mm  y=");                  send_mm(y_hit);
        usart6_send_string(" mm  th=");                 usart6_send_float(th_hit * RAD2DEG, 2);
        usart6_send_string(" do\r\n");

        usart6_send_string("toa_do_cuoi_cung:    x=");  send_mm(pos_x);
        usart6_send_string(" mm  y=");                  send_mm(pos_y);
        usart6_send_string(" mm  th=");                 usart6_send_float(th * RAD2DEG, 2);
        usart6_send_string(" do\r\n");

        if (g_imu) {
            usart6_send_string("toa_do_imu_luc_cham_moc: x=");  send_mm(xi_hit);
            usart6_send_string(" mm  y=");                      send_mm(yi_hit);
            usart6_send_string(" mm  yaw=");                    usart6_send_float(yaw_hit, 2);
            usart6_send_string(" do\r\n");

            usart6_send_string("toa_do_imu_cuoi_cung:    x=");  send_mm(imu_x);
            usart6_send_string(" mm  y=");                      send_mm(imu_y);
            usart6_send_string(" mm  yaw=");                    usart6_send_float(yaw_end, 2);
            usart6_send_string(" do\r\n");

            usart6_send_string("imu: yaw_max_abs=");  usart6_send_float(yaw_max_abs, 2);
            usart6_send_string(" do  u_tb_on_dinh="); usart6_send_float(u_n ? u_sum / (float)u_n : 0.0f, 1);
#if ENC_RUN_IMU_HOLD
            usart6_send_string(" duty  KP=");         usart6_send_float(HH_KP, 2);
            usart6_send_string(" KI=");               usart6_send_float(HH_KI, 2);
            usart6_send_string(" KD=");               usart6_send_float(HH_KD, 2);
#else
            usart6_send_string(" duty  (khong giu huong)");
#endif
            usart6_send_string("\r\n");
        }

        usart6_send_string("v_tb_m_s=");
        usart6_send_float(t_hit > 0 ? x_hit / ((float)t_hit / 1000.0f) : 0.0f, 3);
        usart6_send_string("  (x luc cham moc / thoi gian chay)\r\n");

        usart6_send_string("===================\r\n");
        HAL_Delay(5000);
    }
}
