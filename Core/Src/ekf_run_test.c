#include "ekf_run_test.h"
#include "config.h"
#include "ekf.h"
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

static uint8_t  moving;
static int32_t  cnt[4];
static float    gz_prev;
static float    cxp, cyp, cthp;           // khung camera truoc (nhan ra khung dong bang)
static uint16_t cam_nlog;
static uint32_t cam_good_ms, cam_gap_max;
static float    ex_, ey_, eth_;           // odometry thuan encoder, he xuat phat

// He xuat phat
static float    xs, ys, th0, c0, s0;

// Trung binh camera luc dung yen
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

// Toa do he G -> he xuat phat (doc, ngang)
static void to_start(float X, float Y, float *a, float *c)
{
    float dx = X - xs, dy = Y - ys;
    *a =  c0 * dx + s0 * dy;
    *c = -s0 * dx + c0 * dy;
}

static void send_counts(void)
{
    for (int i = 0; i < 4; i++) {
        usart6_send_char(' ');
        usart6_send_string(WNAME[i]);
        usart6_send_char('=');
        usart6_send_int(cnt[i]);
    }
}

static void sense(float dt, uint32_t now)
{
    pose_link_poll();

    int32_t d[4];
    encoder_get_deltas(d);
    for (int i = 0; i < 4; i++) cnt[i] += d[i];

    const float k = TWO_PI / COUNTS_PER_REV;
    float dxb, dyb, dth;
    forward_kinematics(d[MOTOR_FL] * k, d[MOTOR_FR] * k, d[MOTOR_RL] * k, d[MOTOR_RR] * k,
                       &dxb, &dyb, &dth);

    // Odometry thuan encoder (giong enc_run_test: khong bu truot, goc tu encoder)
    float tm = eth_ + 0.5f * dth;
    ex_ += dxb * cosf(tm) - dyb * sinf(tm);
    ey_ += dxb * sinf(tm) + dyb * cosf(tm);
    eth_ += dth;

    mpu6050_update(dt);
    float gz = mpu6050_get_gyro_z_dps();
    if (fabsf(gz) > GOTO_GZ_MAX_DPS) gz = gz_prev;
    gz_prev = gz;

    ekf_predict(dxb * K_SLIP_X, dyb * K_SLIP_Y, gz * DEG2RAD, dt, now);

    if (pose_link_take_fresh()) {
        float xc = pose_link_get_x(), yc = pose_link_get_y();
        float thc = pose_link_get_theta_deg() * DEG2RAD;
        uint8_t same = (xc == cxp && yc == cyp && thc == cthp);
        cxp = xc; cyp = yc; cthp = thc;
        if (!(same && moving)) {
            uint32_t gap = now - cam_good_ms;
            if (moving && gap > cam_gap_max) cam_gap_max = gap;
            cam_good_ms = now;
            if (ekf_update_cam(xc, yc, thc, now - EKF_CAM_DELAY_MS) == 2) {
                usart6_send_string("EKFRUN:NAP_LAI nhay_mm=");
                usart6_send_int((int32_t)lroundf(ekf_reset_jump() * 1000.0f));
                usart6_send_string("\r\n");
            }
            cam_nlog++;
            if (av_on) { av_x += xc; av_y += yc; av_s += sinf(thc); av_c += cosf(thc); av_n++; }
        }
    }
}

static void log_line(uint32_t t, int32_t duty, float u, float s)
{
    float xi, yi, xc, yc;
    to_start(ekf_x(), ekf_y(), &xi, &yi);
    to_start(pose_link_get_x(), pose_link_get_y(), &xc, &yc);

    usart6_send_string("ENC,");
    usart6_send_int((int32_t)t);   usart6_send_char(',');
    usart6_send_int(duty);
    for (int i = 0; i < 4; i++) { usart6_send_char(','); usart6_send_int(cnt[i]); }
    usart6_send_char(','); send_mm(ex_);
    usart6_send_char(','); send_mm(ey_);
    usart6_send_char(','); usart6_send_float(eth_ * RAD2DEG, 2);
    usart6_send_char(','); usart6_send_float(wrap_pi(ekf_th() - th0) * RAD2DEG, 2);
    usart6_send_char(','); usart6_send_float(gz_prev, 2);
    usart6_send_char(','); usart6_send_float(u, 1);
    usart6_send_char(','); send_mm(xi);
    usart6_send_char(','); send_mm(yi);
    usart6_send_char(','); send_mm(xc);
    usart6_send_char(','); send_mm(yc);
    usart6_send_char(','); usart6_send_float(wrap_pi(pose_link_get_theta_deg() * DEG2RAD - th0) * RAD2DEG, 1);
    usart6_send_char(','); usart6_send_int(cam_nlog);
    usart6_send_char(','); usart6_send_float(s, 1);
    usart6_send_string("\r\n");
    cam_nlog = 0;
}

static int32_t clamp_i(float v, float m)
{
    if (v >  m) v =  m;
    if (v < -m) v = -m;
    return (int32_t)lroundf(v);
}

void ekf_run_test_run(uint8_t imu_ready)
{
    robot_set_velocity(0.0f, 0.0f, 0.0f);
    if (!imu_ready) {
        while (1) {
            usart6_send_string("EKFRUN:LOI - khong thay MPU6050, khong chay\r\n");
            HAL_Delay(2000);
        }
    }

    usart6_send_string("EKFRUN:START quang_duong=");
    usart6_send_int((int32_t)lroundf(ER_DIST_M * 1000.0f));
    usart6_send_string(" mm duty=");      usart6_send_int(ENC_RUN_DUTY);
    usart6_send_string(" | huong KP=");   usart6_send_float(HH_KP, 2);
    usart6_send_string(" KI=");           usart6_send_float(HH_KI, 2);
    usart6_send_string(" KD=");           usart6_send_float(HH_KD, 2);
    usart6_send_string(" | ngang ");
#if ER_LAT_HOLD
    usart6_send_string("KY=");            usart6_send_float(ER_KY, 2);
    usart6_send_string(" KIY=");          usart6_send_float(ER_KIY, 2);
#else
    usart6_send_string("TAT");
#endif
    usart6_send_string("\r\n");

    // ---------- 1. Cho camera ----------
    uint32_t last = HAL_GetTick(), last_msg = last;
    float sx = 0.0f, sy = 0.0f, ss = 0.0f, sc = 0.0f;
    uint32_t n = 0;
    while (n < GOTO_INIT_FRAMES) {
        uint32_t now = HAL_GetTick();
        if ((now - last) < CONTROL_DT_MS) continue;
        float dt = (float)(now - last) / 1000.0f;
        last = now;
        pose_link_poll();
        int32_t d[4];
        encoder_get_deltas(d);
        mpu6050_update(dt);
        if (pose_link_take_fresh()) {
            float th = pose_link_get_theta_deg() * DEG2RAD;
            sx += pose_link_get_x(); sy += pose_link_get_y();
            ss += sinf(th);          sc += cosf(th);
            n++;
        }
        if ((now - last_msg) >= 1000u) {
            last_msg = now;
            usart6_send_string("EKFRUN:CHO_CAMERA so_khung=");
            usart6_send_int((int32_t)n);
            usart6_send_string("\r\n");
        }
    }
    ekf_init(sx / (float)n, sy / (float)n, atan2f(ss, sc), HAL_GetTick());
    cxp = cyp = cthp = 1e9f;
    gz_prev = 0.0f;
    moving = 0;

    // ---------- 2. Dem nguoc, xe dung yen: EKF hoc bias ----------
    uint32_t t_cd = HAL_GetTick();
    last = t_cd; last_msg = t_cd - 1000u;
    while ((HAL_GetTick() - t_cd) < ENC_RUN_START_DELAY_MS) {
        uint32_t now = HAL_GetTick();
        if ((now - last) < CONTROL_DT_MS) continue;
        float dt = (float)(now - last) / 1000.0f;
        last = now;
        sense(dt, now);
        if ((now - last_msg) >= 1000u) {
            last_msg = now;
            usart6_send_string("EKFRUN:chay sau ");
            usart6_send_int((int32_t)((ENC_RUN_START_DELAY_MS - (now - t_cd) + 999u) / 1000u));
            usart6_send_string(" s\r\n");
        }
    }

    // He xuat phat = pose EKF luc nay
    xs = ekf_x(); ys = ekf_y(); th0 = ekf_th();
    c0 = cosf(th0); s0 = sinf(th0);
    for (int i = 0; i < 4; i++) cnt[i] = 0;
    ex_ = ey_ = eth_ = 0.0f;
    cam_nlog = 0;
    cam_gap_max = 0;

    usart6_send_string("EKFRUN:XUAT_PHAT x=");  send_mm(xs);
    usart6_send_string(" y=");                  send_mm(ys);
    usart6_send_string(" th=");                 usart6_send_float(th0 * RAD2DEG, 2);
    usart6_send_string(" (he camera)\r\n");
    usart6_send_string("ENC,t_ms,duty,FL,FR,RL,RR,x_mm,y_mm,th_deg,yaw_deg,gz_dps,u,xi_mm,yi_mm,"
                       "xc_mm,yc_mm,thc_deg,cam,s\r\n");

    // ---------- 3. Chay thang ----------
    const float L = ER_DIST_M;
    const uint32_t t0 = HAL_GetTick();
    uint32_t last_log = t0 - ENC_RUN_LOG_EVERY_MS;
    uint32_t t_hit = 0;
    uint8_t  timeout = 0;
    float    hi = 0.0f, yi_i = 0.0f, u = 0.0f, s = 0.0f;
    float    yaw_max = 0.0f, cr_max = 0.0f, cr_s2 = 0.0f, cr_len = 0.0f, al_prev = 0.0f;
    float    u_sum = 0.0f, s_sum = 0.0f;
    uint32_t u_n = 0;
    int32_t  duty = ENC_RUN_DUTY_START;
    last = t0;
    cam_good_ms = t0;
    moving = 1;

    while (1) {
        uint32_t now = HAL_GetTick();
        if ((now - last) < CONTROL_DT_MS) continue;
        float dt = (float)(now - last) / 1000.0f;
        last = now;
        uint32_t t = now - t0;

        sense(dt, now);

        float al, cr;
        to_start(ekf_x(), ekf_y(), &al, &cr);
        float yaw = wrap_pi(ekf_th() - th0) * RAD2DEG;

        if (fabsf(yaw) > yaw_max) yaw_max = fabsf(yaw);
        if (fabsf(cr) > cr_max) cr_max = fabsf(cr);
        float dal = fabsf(al - al_prev);
        cr_s2 += cr * cr * dal; cr_len += dal; al_prev = al;

        if (al >= L)                    { t_hit = t; break; }
        if (t >= ENC_RUN_TIMEOUT_MS)    { t_hit = t; timeout = 1; break; }

        duty = ENC_RUN_DUTY;
#if ENC_RUN_RAMP_MS > 0
        if (t < ENC_RUN_RAMP_MS)
            duty = ENC_RUN_DUTY_START + (int32_t)((ENC_RUN_DUTY - ENC_RUN_DUTY_START) * (int32_t)t / (int32_t)ENC_RUN_RAMP_MS);
#endif

        // Giu huong: e = 0 - goc EKF (do)
        float e = -yaw;
        if (t >= ENC_RUN_RAMP_MS) {
            hi += HH_KI * e * dt;
            if (hi >  HH_I_MAX) hi =  HH_I_MAX;
            if (hi < -HH_I_MAX) hi = -HH_I_MAX;
        }
        u = HH_KP * e + hi - HH_KD * gz_prev;
        if (u >  HH_U_MAX) u =  HH_U_MAX;
        if (u < -HH_U_MAX) u = -HH_U_MAX;

        // Keo ve duong thang: lech ngang cr (mm), s > 0 -> chay sang trai
#if ER_LAT_HOLD
        float cr_mm = cr * 1000.0f;
        if (t >= ENC_RUN_RAMP_MS) {
            yi_i += ER_KIY * cr_mm * dt;
            if (yi_i >  ER_SI_MAX) yi_i =  ER_SI_MAX;
            if (yi_i < -ER_SI_MAX) yi_i = -ER_SI_MAX;
        }
        s = -(ER_KY * cr_mm + yi_i);
        if (s >  ER_S_MAX) s =  ER_S_MAX;
        if (s < -ER_S_MAX) s = -ER_S_MAX;
#else
        s = 0.0f;
#endif
        if (t >= 1000u) { u_sum += u; s_sum += s; u_n++; }

        int32_t du = clamp_i(u, HH_U_MAX), ds = clamp_i(s, ER_S_MAX);
        motor_set_duty_raw(duty - du - ds, duty + du + ds, duty - du + ds, duty + du - ds);

        if ((now - last_log) >= ENC_RUN_LOG_EVERY_MS) {
            last_log = now;
            log_line(t, duty, u, s);
        }
    }

    // ---------- 4. Cat dong co, troi den khi dung han ----------
    motor_set_duty_raw(0, 0, 0, 0);
    log_line(t_hit, 0, 0.0f, 0.0f);
    usart6_send_string(timeout ? "EKFRUN:HET_GIO" : "EKFRUN:DU_QUANG_DUONG");
    usart6_send_string(" t_ms=");
    usart6_send_int((int32_t)t_hit);
    usart6_send_string("\r\n");

    int32_t cnt_hit[4];
    for (int i = 0; i < 4; i++) cnt_hit[i] = cnt[i];

    uint32_t t_still = HAL_GetTick();
    last = t_still; last_log = t_still;
    while (1) {
        uint32_t now = HAL_GetTick();
        if ((now - last) < CONTROL_DT_MS) continue;
        float dt = (float)(now - last) / 1000.0f;
        last = now;
        int32_t c_before = cnt[0] + cnt[1] + cnt[2] + cnt[3];
        sense(dt, now);
        if (cnt[0] + cnt[1] + cnt[2] + cnt[3] != c_before) t_still = now;
        if ((now - last_log) >= ENC_RUN_LOG_EVERY_MS) { last_log = now; log_line(now - t0, 0, 0.0f, 0.0f); }
        if ((now - t_still) >= ENC_RUN_REST_MS) break;
        if ((now - t0 - t_hit) >= ENC_RUN_COAST_MAX_MS) break;
    }
    const uint32_t t_rest = t_still - t0;
    moving = 0;

    // ---------- 5. Dung yen: camera het tre, lay trung binh lam ket qua ----------
    uint32_t t_w = HAL_GetTick();
    last = t_w;
    av_x = av_y = av_s = av_c = 0.0f; av_n = 0; av_on = 0;
    while ((HAL_GetTick() - t_w) < GOTO_SETTLE_MS) {
        uint32_t now = HAL_GetTick();
        if ((now - last) < CONTROL_DT_MS) continue;
        float dt = (float)(now - last) / 1000.0f;
        last = now;
        if (!av_on && (now - t_w) >= GOTO_SETTLE_MS - GOTO_SETTLE_AVG_MS) av_on = 1;
        sense(dt, now);
    }
    av_on = 0;

    float xi_e, yi_e, xc_e = 0.0f, yc_e = 0.0f, thc_e = 0.0f;
    to_start(ekf_x(), ekf_y(), &xi_e, &yi_e);
    float yaw_e = wrap_pi(ekf_th() - th0) * RAD2DEG;
    uint8_t cam_ok = (av_n > 0);
    if (cam_ok) {
        to_start(av_x / (float)av_n, av_y / (float)av_n, &xc_e, &yc_e);
        thc_e = wrap_pi(atan2f(av_s, av_c) - th0) * RAD2DEG;
    }
    float cr_rms = (cr_len > 1e-3f) ? sqrtf(cr_s2 / cr_len) : 0.0f;

    while (1) {
        usart6_send_string("===== KET QUA =====\r\n");
        usart6_send_string("ket_qua: ");
        usart6_send_string(timeout ? "HET_GIO" : "DU_QUANG_DUONG banh EKF");
        usart6_send_string("  muc_tieu=");  usart6_send_int((int32_t)lroundf(L * 1000.0f));
        usart6_send_string(" mm  duty_chung="); usart6_send_int(ENC_RUN_DUTY);
        usart6_send_string("\r\nthoi_gian_chay_ms="); usart6_send_int((int32_t)t_hit);
        usart6_send_string("  (bat dong co -> du quang duong)\r\nthoi_gian_toi_luc_dung_han_ms=");
        usart6_send_int((int32_t)t_rest);
        usart6_send_string("  troi_ms=");   usart6_send_int((int32_t)(t_rest > t_hit ? t_rest - t_hit : 0));
        usart6_send_string("\r\nxung_luc_cham_moc:");
        for (int i = 0; i < 4; i++) {
            usart6_send_char(' '); usart6_send_string(WNAME[i]);
            usart6_send_char('='); usart6_send_int(cnt_hit[i]);
        }
        usart6_send_string("\r\nxung_cuoi_cung:  ");  send_counts();
        usart6_send_string("\r\ntoa_do_cuoi_cung:    x="); send_mm(ex_);
        usart6_send_string(" mm  y=");      send_mm(ey_);
        usart6_send_string(" mm  th=");     usart6_send_float(eth_ * RAD2DEG, 2);
        usart6_send_string(" do  (odometry thuan encoder)\r\ntoa_do_ekf_cuoi_cung:    x="); send_mm(xi_e);
        usart6_send_string(" mm  y=");      send_mm(yi_e);
        usart6_send_string(" mm  yaw=");    usart6_send_float(yaw_e, 2);
        usart6_send_string(" do\r\n");
        if (cam_ok) {
            usart6_send_string("toa_do_camera_cuoi_cung: x="); send_mm(xc_e);
            usart6_send_string(" mm  y=");  send_mm(yc_e);
            usart6_send_string(" mm  th="); usart6_send_float(thc_e, 2);
            usart6_send_string(" do  (trung binh khi dung yen)\r\n");
        } else {
            usart6_send_string("toa_do_camera_cuoi_cung: KHONG CO KHUNG MOI\r\n");
        }
        usart6_send_string("imu: yaw_max_abs="); usart6_send_float(yaw_max, 2);
        usart6_send_string(" do  u_tb_on_dinh="); usart6_send_float(u_n ? u_sum / (float)u_n : 0.0f, 1);
        usart6_send_string(" duty  KP=");  usart6_send_float(HH_KP, 2);
        usart6_send_string(" KI=");        usart6_send_float(HH_KI, 2);
        usart6_send_string(" KD=");        usart6_send_float(HH_KD, 2);
        usart6_send_string("\r\nngang: lech_max_ekf="); send_mm(cr_max);
        usart6_send_string(" mm  lech_rms_ekf=");       send_mm(cr_rms);
        usart6_send_string(" mm  s_tb=");               usart6_send_float(u_n ? s_sum / (float)u_n : 0.0f, 1);
        usart6_send_string(" duty  KY=");               usart6_send_float(ER_LAT_HOLD ? ER_KY : 0.0f, 2);
        usart6_send_string(" KIY=");                    usart6_send_float(ER_LAT_HOLD ? ER_KIY : 0.0f, 2);
        usart6_send_string("\r\nekf: bias=");           usart6_send_float(ekf_bias() * RAD2DEG, 3);
        usart6_send_string(" do/s  khung_nhan=");       usart6_send_int((int32_t)ekf_n_ok());
        usart6_send_string("  khung_loai=");            usart6_send_int((int32_t)ekf_n_reject());
        usart6_send_string("  nap_lai=");               usart6_send_int((int32_t)ekf_n_reset());
        usart6_send_string("  mat_cam_lau_nhat_ms=");   usart6_send_int((int32_t)cam_gap_max);
        usart6_send_string("\r\n");
        HAL_Delay(5000);
    }
}
