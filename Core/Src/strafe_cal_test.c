#include "strafe_cal_test.h"
#include "config.h"
#include "encoder.h"
#include "kinematics.h"
#include "motor.h"
#include "mpu6050.h"
#include "pose_link.h"
#include "robot_control.h"
#include "usart6.h"
#include "stm32f4xx_hal.h"
#include <math.h>

#define DEG2RAD  0.017453293f
#define RAD2DEG  57.29577951f

static const char *WNAME[4] = { "FL", "FR", "RL", "RR" };

static int32_t  cnt[4], acc[4];
static float    ex_, ey_, eth_;           // odometry thuan encoder, he xuat phat
static float    ix_, iy_, yaw_prev;       // odometry K_SLIP + goc IMU
static float    yaw0;
static uint8_t  moving;
static float    cxp, cyp, cthp;
static uint16_t cam_nlog;
static uint32_t cam_good_ms, cam_gap_max, cam_n;

static float    xs, ys, th0, c0, s0;      // he xuat phat (camera)
static float    av_x, av_y, av_s, av_c;
static uint32_t av_n;
static uint8_t  av_on;

static float wrap_pi(float a)
{
    while (a >  PI) a -= TWO_PI;
    while (a < -PI) a += TWO_PI;
    return a;
}

static void send_mm(float m) { usart6_send_int((int32_t)lroundf(m * 1000.0f)); }

static void to_start(float X, float Y, float *a, float *c)
{
    float dx = X - xs, dy = Y - ys;
    *a =  c0 * dx + s0 * dy;
    *c = -s0 * dx + c0 * dy;
}

static float yaw_rel_deg(void) { return mpu6050_get_yaw_deg() - yaw0; }

static uint8_t poll_enc(void)
{
    int32_t d[4];
    uint8_t moved = 0;
    encoder_get_deltas(d);
    for (int i = 0; i < 4; i++) {
        cnt[i] += d[i];
        acc[i] += d[i];
        if (d[i] != 0) moved = 1;
    }
    return moved;
}

static int32_t max_abs_cnt(void)
{
    int32_t m = 0;
    for (int i = 0; i < 4; i++) {
        int32_t a = cnt[i] < 0 ? -cnt[i] : cnt[i];
        if (a > m) m = a;
    }
    return m;
}

static void odo_step(void)
{
    const float k = TWO_PI / COUNTS_PER_REV;
    float dxb, dyb, dth;
    forward_kinematics(acc[MOTOR_FL] * k, acc[MOTOR_FR] * k, acc[MOTOR_RL] * k, acc[MOTOR_RR] * k,
                       &dxb, &dyb, &dth);
    for (int i = 0; i < 4; i++) acc[i] = 0;

    float tm = eth_ + 0.5f * dth;
    ex_ += dxb * cosf(tm) - dyb * sinf(tm);
    ey_ += dxb * sinf(tm) + dyb * cosf(tm);
    eth_ += dth;

    float yaw = yaw_rel_deg() * DEG2RAD;
    float ym  = 0.5f * (yaw + yaw_prev);
    float ax = dxb * K_SLIP_X, ay = dyb * K_SLIP_Y;
    ix_ += ax * cosf(ym) - ay * sinf(ym);
    iy_ += ax * sinf(ym) + ay * cosf(ym);
    yaw_prev = yaw;
}

static void cam_step(uint32_t now)
{
#if !SC_USE_CAM
    (void)now;
    return;
#endif
    pose_link_poll();
    if (!pose_link_take_fresh()) return;
    float xc = pose_link_get_x(), yc = pose_link_get_y();
    float thc = pose_link_get_theta_deg() * DEG2RAD;
    uint8_t same = (xc == cxp && yc == cyp && thc == cthp);
    cxp = xc; cyp = yc; cthp = thc;
    if (same && moving) return;                  // khung dong bang luc dang chay
    uint32_t gap = now - cam_good_ms;
    if (moving && gap > cam_gap_max) cam_gap_max = gap;
    cam_good_ms = now;
    cam_nlog++;
    cam_n++;
    if (av_on) { av_x += xc; av_y += yc; av_s += sinf(thc); av_c += cosf(thc); av_n++; }
}

static void log_line(uint32_t t, int32_t duty, float u)
{
    float xc, yc;
    to_start(pose_link_get_x(), pose_link_get_y(), &xc, &yc);

    usart6_send_string("ENC,");
    usart6_send_int((int32_t)t);   usart6_send_char(',');
    usart6_send_int(duty);
    for (int i = 0; i < 4; i++) { usart6_send_char(','); usart6_send_int(cnt[i]); }
    usart6_send_char(','); send_mm(ex_);
    usart6_send_char(','); send_mm(ey_);
    usart6_send_char(','); usart6_send_float(eth_ * RAD2DEG, 2);
    usart6_send_char(','); usart6_send_float(yaw_rel_deg(), 2);
    usart6_send_char(','); usart6_send_float(mpu6050_get_gyro_z_dps(), 2);
    usart6_send_char(','); usart6_send_float(u, 1);
    usart6_send_char(','); send_mm(ix_);
    usart6_send_char(','); send_mm(iy_);
#if !SC_USE_CAM
    (void)xc; (void)yc;
    usart6_send_string("\r\n");
    return;
#endif
    usart6_send_char(','); send_mm(xc);
    usart6_send_char(','); send_mm(yc);
    usart6_send_char(','); usart6_send_float(wrap_pi(pose_link_get_theta_deg() * DEG2RAD - th0) * RAD2DEG, 1);
    usart6_send_char(','); usart6_send_int(cam_nlog);
    usart6_send_string(",0.0\r\n");
    cam_nlog = 0;
}

static void send_counts(const int32_t c[4])
{
    for (int i = 0; i < 4; i++) {
        usart6_send_char(' ');
        usart6_send_string(WNAME[i]);
        usart6_send_char('=');
        usart6_send_int(c[i]);
    }
}

static int32_t clamp_i(float v, float m)
{
    if (v >  m) v =  m;
    if (v < -m) v = -m;
    return (int32_t)lroundf(v);
}

void strafe_cal_test_run(uint8_t imu_ready)
{
    robot_set_velocity(0.0f, 0.0f, 0.0f);
    if (!imu_ready) {
        while (1) {
            usart6_send_string("NGANG:LOI - khong thay MPU6050, khong chay\r\n");
            HAL_Delay(2000);
        }
    }

    const int32_t target = SC_TARGET_COUNTS;
    usart6_send_string("NGANG:START muc_tieu=");
    usart6_send_int(target);
    usart6_send_string(" xung (");
    usart6_send_int((int32_t)lroundf(SC_DIST_M * 1000.0f));
    usart6_send_string(" mm banh lan) huong=");
    usart6_send_string(SC_DIR > 0 ? "TRAI" : "PHAI");
    usart6_send_string(" duty=");          usart6_send_int(SC_DUTY);
    usart6_send_string(" | giu huong IMU ");
#if SC_IMU_HOLD
    usart6_send_string("KP=");             usart6_send_float(HH_KP, 2);
    usart6_send_string(" KI=");            usart6_send_float(HH_KI, 2);
    usart6_send_string(" KD=");            usart6_send_float(HH_KD, 2);
#else
    usart6_send_string("TAT");
#endif
    usart6_send_string(" | K_SLIP_Y hien tai="); usart6_send_float(K_SLIP_Y, 3);
    usart6_send_string("\r\n");

    // ---------- 1. Cho camera ----------
    uint32_t last = HAL_GetTick(), last_msg = last;
    cxp = cyp = cthp = 1e9f;
    moving = 0;
    cam_n = 0;
    while (SC_USE_CAM && cam_n < GOTO_INIT_FRAMES) {
        uint32_t now = HAL_GetTick();
        if ((now - last) < CONTROL_DT_MS) continue;
        float dt = (float)(now - last) / 1000.0f;
        last = now;
        mpu6050_update(dt);
        poll_enc();
        cam_step(now);
        if ((now - last_msg) >= 1000u) {
            last_msg = now;
            usart6_send_string("NGANG:CHO_CAMERA so_khung=");
            usart6_send_int((int32_t)cam_n);
            usart6_send_string("\r\n");
        }
    }

    // ---------- 2. Dem nguoc, xe dung yen, trung binh camera lam diem xuat phat ----------
    uint32_t t_cd = HAL_GetTick();
    last = t_cd; last_msg = t_cd - 1000u;
    av_x = av_y = av_s = av_c = 0.0f; av_n = 0; av_on = 1;
    while ((HAL_GetTick() - t_cd) < ENC_RUN_START_DELAY_MS) {
        uint32_t now = HAL_GetTick();
        if ((now - last) < CONTROL_DT_MS) continue;
        float dt = (float)(now - last) / 1000.0f;
        last = now;
        mpu6050_update(dt);
        poll_enc();
        cam_step(now);
        if ((now - last_msg) >= 1000u) {
            last_msg = now;
            usart6_send_string("NGANG:chay sau ");
            usart6_send_int((int32_t)((ENC_RUN_START_DELAY_MS - (now - t_cd) + 999u) / 1000u));
            usart6_send_string(" s\r\n");
        }
    }
    av_on = 0;
#if SC_USE_CAM
    if (av_n == 0) {
        while (1) {
            usart6_send_string("NGANG:LOI - mat camera luc dem nguoc, khong chay\r\n");
            HAL_Delay(2000);
        }
    }
    xs = av_x / (float)av_n; ys = av_y / (float)av_n; th0 = atan2f(av_s, av_c);
#else
    xs = ys = th0 = 0.0f;
#endif
    c0 = cosf(th0); s0 = sinf(th0);
    yaw0 = mpu6050_get_yaw_deg();
    yaw_prev = 0.0f;
    for (int i = 0; i < 4; i++) { cnt[i] = 0; acc[i] = 0; }
    ex_ = ey_ = eth_ = 0.0f;
    ix_ = iy_ = 0.0f;
    cam_nlog = 0;
    cam_gap_max = 0;

#if SC_USE_CAM
    usart6_send_string("NGANG:XUAT_PHAT x=");  send_mm(xs);
    usart6_send_string(" y=");                 send_mm(ys);
    usart6_send_string(" th=");                usart6_send_float(th0 * RAD2DEG, 2);
    usart6_send_string(" (he camera, trung binh ");
    usart6_send_int((int32_t)av_n);
    usart6_send_string(" khung)\r\n");
    usart6_send_string("ENC,t_ms,duty,FL,FR,RL,RR,x_mm,y_mm,th_deg,yaw_deg,gz_dps,u,xi_mm,yi_mm,"
                       "xc_mm,yc_mm,thc_deg,cam,s\r\n");
#else
    usart6_send_string("NGANG:XUAT_PHAT (khong camera)\r\n");
    usart6_send_string("ENC,t_ms,duty,FL,FR,RL,RR,x_mm,y_mm,th_deg,yaw_deg,gz_dps,u,xi_mm,yi_mm\r\n");
#endif

    // ---------- 3. Chay ngang ----------
    const uint32_t t0 = HAL_GetTick();
    uint32_t last_ctl = t0, last_log = t0 - ENC_RUN_LOG_EVERY_MS;
    uint32_t t_hit = 0;
    int      hit = -1;
    uint8_t  timeout = 0;
    float    hi = 0.0f, u = 0.0f, yaw_max = 0.0f, u_sum = 0.0f;
    (void)hi;
    uint32_t u_n = 0;
    int32_t  duty = SC_DUTY_START;
    cam_good_ms = t0;
    moving = 1;

    while (1) {
        poll_enc();
        uint32_t now = HAL_GetTick();
        uint32_t t   = now - t0;

        // Kiem tra moc ngay sau moi lan doc encoder, khong cho chu ky 20 ms
        for (int i = 0; i < 4; i++) {
            int32_t a = cnt[i] < 0 ? -cnt[i] : cnt[i];
            if (a >= target) { hit = i; break; }
        }
        if (hit >= 0 || t >= ENC_RUN_TIMEOUT_MS) {
            motor_set_duty_raw(0, 0, 0, 0);
            t_hit = t;
            timeout = (hit < 0);
            break;
        }

        if ((now - last_ctl) >= CONTROL_DT_MS) {
            float dt = (float)(now - last_ctl) / 1000.0f;
            last_ctl = now;
            mpu6050_update(dt);
            odo_step();
            cam_step(now);

            duty = SC_DUTY;
            if (t < SC_RAMP_MS)
                duty = SC_DUTY_START + (int32_t)((SC_DUTY - SC_DUTY_START) * (int32_t)t / (int32_t)SC_RAMP_MS);

            float yaw = yaw_rel_deg();
            if (fabsf(yaw) > yaw_max) yaw_max = fabsf(yaw);
#if SC_IMU_HOLD
            float e = -yaw;
            if (t >= SC_RAMP_MS) {
                hi += HH_KI * e * dt;
                if (hi >  HH_I_MAX) hi =  HH_I_MAX;
                if (hi < -HH_I_MAX) hi = -HH_I_MAX;
            }
            u = HH_KP * e + hi - HH_KD * mpu6050_get_gyro_z_dps();
            if (u >  HH_U_MAX) u =  HH_U_MAX;
            if (u < -HH_U_MAX) u = -HH_U_MAX;
#endif
            if (t >= 1000u) { u_sum += u; u_n++; }

            // FL = cx - cy - u, FR = cx + cy + u, RL = cx + cy - u, RR = cx - cy + u ; cx = 0
            int32_t cy = SC_DIR * duty, du = clamp_i(u, HH_U_MAX);
            motor_set_duty_raw(-cy - du, cy + du, cy - du, -cy + du);
        }

        if ((now - last_log) >= ENC_RUN_LOG_EVERY_MS && max_abs_cnt() < target - SC_LOG_GUARD) {
            last_log = now;
            log_line(t, duty, u);
        }
    }

    // ---------- 4. Cat dong co, troi den khi dung han ----------
    int32_t cnt_hit[4];
    for (int i = 0; i < 4; i++) cnt_hit[i] = cnt[i];
    log_line(t_hit, 0, 0.0f);
    usart6_send_string(timeout ? "NGANG:HET_GIO" : "NGANG:CHAM_MOC banh=");
    if (!timeout) usart6_send_string(WNAME[hit]);
    usart6_send_string(" t_ms=");
    usart6_send_int((int32_t)t_hit);
    usart6_send_string("\r\n");

    uint32_t t_still = HAL_GetTick();
    last_ctl = t_still; last_log = t_still;
    while (1) {
        if (poll_enc()) t_still = HAL_GetTick();
        uint32_t now = HAL_GetTick();
        if ((now - last_ctl) >= CONTROL_DT_MS) {
            float dt = (float)(now - last_ctl) / 1000.0f;
            last_ctl = now;
            mpu6050_update(dt);
            odo_step();
            cam_step(now);
        }
        if ((now - last_log) >= ENC_RUN_LOG_EVERY_MS) { last_log = now; log_line(now - t0, 0, 0.0f); }
        if ((now - t_still) >= ENC_RUN_REST_MS) break;
        if ((now - t0 - t_hit) >= ENC_RUN_COAST_MAX_MS) break;
    }
    const uint32_t t_rest = t_still - t0;
    moving = 0;

    // ---------- 5. Dung yen: camera het tre, lay trung binh lam ket qua ----------
    uint32_t t_w = HAL_GetTick();
    last_ctl = t_w;
    av_x = av_y = av_s = av_c = 0.0f; av_n = 0; av_on = 0;
    while ((HAL_GetTick() - t_w) < GOTO_SETTLE_MS) {
        uint32_t now = HAL_GetTick();
        if ((now - last_ctl) < CONTROL_DT_MS) continue;
        float dt = (float)(now - last_ctl) / 1000.0f;
        last_ctl = now;
        if (!av_on && (now - t_w) >= GOTO_SETTLE_MS - GOTO_SETTLE_AVG_MS) av_on = 1;
        mpu6050_update(dt);
        poll_enc();
        odo_step();
        cam_step(now);
    }
    av_on = 0;

    float xc_e = 0.0f, yc_e = 0.0f, thc_e = 0.0f;
    uint8_t cam_ok = (av_n > 0);
    if (cam_ok) {
        to_start(av_x / (float)av_n, av_y / (float)av_n, &xc_e, &yc_e);
        thc_e = wrap_pi(atan2f(av_s, av_c) - th0) * RAD2DEG;
    }

    // Quang duong banh lan (FK tren tong xung, khong bu truot)
    const float k = TWO_PI / COUNTS_PER_REV;
    float rx, ry, rth;
    forward_kinematics(cnt[MOTOR_FL] * k, cnt[MOTOR_FR] * k, cnt[MOTOR_RL] * k, cnt[MOTOR_RR] * k,
                       &rx, &ry, &rth);
    float k_y = (cam_ok && fabsf(ry) > 1e-3f) ? yc_e / ry : 0.0f;
    float t_s  = (float)t_hit / 1000.0f;
    float v_tb = (t_s > 0.0f) ? (cam_ok ? fabsf(yc_e) : fabsf(ry)) / t_s : 0.0f;   // camera neu co

    while (1) {
        usart6_send_string("===== KET QUA =====\r\n");
        usart6_send_string("ket_qua: ");
        if (timeout) usart6_send_string("HET_GIO");
        else { usart6_send_string("CHAM_MOC banh "); usart6_send_string(WNAME[hit]); }
        usart6_send_string("  muc_tieu=");     usart6_send_int(target);
        usart6_send_string(" xung  duty_ngang="); usart6_send_int(SC_DUTY);
        usart6_send_string("  huong=");        usart6_send_string(SC_DIR > 0 ? "TRAI" : "PHAI");
        usart6_send_string("\r\nthoi_gian_chay_ms="); usart6_send_int((int32_t)t_hit);
        usart6_send_string("  thoi_gian_toi_luc_dung_han_ms="); usart6_send_int((int32_t)t_rest);
        usart6_send_string("\r\nxung_luc_cham_moc:"); send_counts(cnt_hit);
        usart6_send_string("\r\nxung_cuoi_cung:  ");  send_counts(cnt);
        usart6_send_string("\r\nbanh_lan: ngang_mm="); send_mm(ry);
        usart6_send_string("  doc_mm=");       send_mm(rx);
        usart6_send_string("  xoay_do=");      usart6_send_float(rth * RAD2DEG, 2);
        usart6_send_string("\r\ntoa_do_cuoi_cung: x="); send_mm(ex_);
        usart6_send_string(" mm  y=");         send_mm(ey_);
        usart6_send_string(" mm  th=");        usart6_send_float(eth_ * RAD2DEG, 2);
        usart6_send_string(" do  (odometry thuan encoder)\r\ntoa_do_imu_cuoi_cung: x="); send_mm(ix_);
        usart6_send_string(" mm  y=");         send_mm(iy_);
        usart6_send_string(" mm  yaw=");       usart6_send_float(yaw_rel_deg(), 2);
        usart6_send_string(" do  (K_SLIP hien tai + goc IMU)\r\n");
        if (!SC_USE_CAM) {
            // khong in dong camera
        } else if (cam_ok) {
            usart6_send_string("toa_do_camera_cuoi_cung: x="); send_mm(xc_e);
            usart6_send_string(" mm  y=");     send_mm(yc_e);
            usart6_send_string(" mm  th=");    usart6_send_float(thc_e, 2);
            usart6_send_string(" do  (trung binh khi dung yen)\r\n");
        } else {
            usart6_send_string("toa_do_camera_cuoi_cung: KHONG CO KHUNG MOI\r\n");
        }
        usart6_send_string("imu: yaw_max_abs=");  usart6_send_float(yaw_max, 2);
        usart6_send_string(" do  u_tb_on_dinh="); usart6_send_float(u_n ? u_sum / (float)u_n : 0.0f, 1);
        usart6_send_string(" duty  KP=");  usart6_send_float(SC_IMU_HOLD ? HH_KP : 0.0f, 2);
        usart6_send_string(" KI=");        usart6_send_float(SC_IMU_HOLD ? HH_KI : 0.0f, 2);
        usart6_send_string(" KD=");        usart6_send_float(SC_IMU_HOLD ? HH_KD : 0.0f, 2);
        usart6_send_string("\r\nhieu_chuan: K_SLIP_Y_do=");
        if (cam_ok) usart6_send_float(k_y, 3);
        else usart6_send_string("(do tay quang duong ngang) / banh_lan ngang_mm");
        usart6_send_string("  v_tb_mm_s=");   usart6_send_int((int32_t)lroundf(v_tb * 1000.0f));
        usart6_send_string("  mat_cam_lau_nhat_ms="); usart6_send_int((int32_t)cam_gap_max);
        usart6_send_string("\r\n");
        HAL_Delay(5000);
    }
}
