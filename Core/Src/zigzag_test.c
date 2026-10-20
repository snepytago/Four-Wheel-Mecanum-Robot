#include "zigzag_test.h"
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
#define N_STEPS  19

static const char *WNAME[4] = { "FL", "FR", "RL", "RR" };

typedef struct {
    uint8_t turn;          // 0 = di thang, 1 = quay
    float   val;           // m (thang) hoac do co dau (quay, + = trai)
    float   hdg;           // huong muc tieu sau buoc (do, he xuat phat)
    float   wx, wy;        // diem ly thuyet cuoi buoc (m)
} zz_step_t;

typedef struct {
    int8_t   hit;          // 0..3 banh cham moc, 4 = IMU/EKF dat, -1 = het gio
    uint32_t t_start, t_cut, t_rest;
    float    ex, ey, eth;  // odometry encoder (m, m, do)
    float    px, py, ph;   // mode 0/1: encoder + IMU ; mode 2: EKF
    uint8_t  cam_ok;
    float    cx, cy, ch;   // camera trung binh khi dung yen
} zz_res_t;

static zz_step_t ST[N_STEPS];
static zz_res_t  RS[N_STEPS];

static int32_t  cnt[4], acc[4];
static float    ex_, ey_, eth_;           // odometry thuan encoder
static float    ix_, iy_, yaw_prev;       // quang duong encoder + goc IMU (khong K_SLIP)
static float    yaw0, gz_prev;
static uint8_t  moving, cam_on;
static float    cxp, cyp, cthp;
static uint16_t cam_nlog;
static uint32_t cam_good_ms, cam_gap_max, cam_n;
static float    xs, ys, th0, c0, s0;      // he xuat phat trong he camera
static float    av_x, av_y, av_s, av_c;
static uint32_t av_n;
static uint8_t  av_on;

static float wrap_pi(float a)
{
    while (a >  PI) a -= TWO_PI;
    while (a < -PI) a += TWO_PI;
    return a;
}

static float wrap180(float a)
{
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
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

// Pose dung de dieu khien/log: mode 0/1 = encoder + IMU, mode 2 = EKF (he xuat phat)
static void est_pose(float *x, float *y, float *h_deg)
{
#if ZZ_MODE == 2
    to_start(ekf_x(), ekf_y(), x, y);
    *h_deg = wrap_pi(ekf_th() - th0) * RAD2DEG;
#else
    *x = ix_; *y = iy_; *h_deg = yaw_rel_deg();
#endif
}

// Di W0->W5, quay dau 180 do, ve W5->W0 dung duong cu. Thang: v = 0 doan dai, 1 doan ngan ; quay: v = goc (+ trai)
static const struct { uint8_t turn; int16_t v; } MV[N_STEPS] = {
    {0, 0}, {1,  90}, {0, 1}, {1,  90}, {0, 0}, {1, -90}, {0, 1}, {1, -90}, {0, 0},
    {1, 180},
    {0, 0}, {1,  90}, {0, 1}, {1,  90}, {0, 0}, {1, -90}, {0, 1}, {1, -90}, {0, 0},
};

static void build_steps(void)
{
    float x = 0.0f, y = 0.0f, h = 0.0f;
    for (int k = 0; k < N_STEPS; k++) {
        zz_step_t *s = &ST[k];
        if (!MV[k].turn) {
            float L = MV[k].v ? ZZ_SHORT_M : ZZ_LONG_M;
            x += L * cosf(h * DEG2RAD);
            y += L * sinf(h * DEG2RAD);
            if (fabsf(x) < 1e-3f) x = 0.0f;
            if (fabsf(y) < 1e-3f) y = 0.0f;
            s->turn = 0; s->val = L;
        } else {
            h += (float)MV[k].v;
            s->turn = 1; s->val = (float)MV[k].v;
        }
        s->hdg = h; s->wx = x; s->wy = y;
    }
}

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

static void cam_step(uint32_t now)
{
    if (!cam_on) return;
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
#if ZZ_MODE == 2
    if (ekf_update_cam(xc, yc, thc, now - EKF_CAM_DELAY_MS) == 2) {
        usart6_send_string("ZZ:EKF_NAP_LAI nhay_mm=");
        usart6_send_int((int32_t)lroundf(ekf_reset_jump() * 1000.0f));
        usart6_send_string("\r\n");
    }
#endif
}

// 1 chu ky 20 ms: IMU, odometry, EKF, camera
static void ctl_tick(float dt, uint32_t now)
{
    mpu6050_update(dt);
    float gz = mpu6050_get_gyro_z_dps();
    if (fabsf(gz) > GOTO_GZ_MAX_DPS) gz = gz_prev;
    gz_prev = gz;

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
    ix_ += dxb * cosf(ym) - dyb * sinf(ym);
    iy_ += dxb * sinf(ym) + dyb * cosf(ym);
    yaw_prev = yaw;

#if ZZ_MODE == 2
    ekf_predict(dxb, dyb, gz * DEG2RAD, dt, now);
#endif
    cam_step(now);
}

static void log_line(uint32_t t, int32_t duty, float u, float s)
{
    float px, py, ph;
    est_pose(&px, &py, &ph);

    usart6_send_string("ENC,");
    usart6_send_int((int32_t)t);   usart6_send_char(',');
    usart6_send_int(duty);
    for (int i = 0; i < 4; i++) { usart6_send_char(','); usart6_send_int(cnt[i]); }
    usart6_send_char(','); send_mm(ex_);
    usart6_send_char(','); send_mm(ey_);
    usart6_send_char(','); usart6_send_float(eth_ * RAD2DEG, 2);
    usart6_send_char(','); usart6_send_float(ph, 2);
    usart6_send_char(','); usart6_send_float(gz_prev, 2);
    usart6_send_char(','); usart6_send_float(u, 1);
    usart6_send_char(','); send_mm(px);
    usart6_send_char(','); send_mm(py);
    if (cam_on) {
        float xc, yc;
        to_start(pose_link_get_x(), pose_link_get_y(), &xc, &yc);
        usart6_send_char(','); send_mm(xc);
        usart6_send_char(','); send_mm(yc);
        usart6_send_char(','); usart6_send_float(wrap_pi(pose_link_get_theta_deg() * DEG2RAD - th0) * RAD2DEG, 1);
        usart6_send_char(','); usart6_send_int(cam_nlog);
        usart6_send_char(','); usart6_send_float(s, 1);
        cam_nlog = 0;
    }
    usart6_send_string("\r\n");
}

static int32_t max_abs_d(const int32_t c[4])
{
    int32_t m = 0;
    for (int i = 0; i < 4; i++) {
        int32_t a = cnt[i] - c[i];
        if (a < 0) a = -a;
        if (a > m) m = a;
    }
    return m;
}

static int32_t clamp_i(float v, float m)
{
    if (v >  m) v =  m;
    if (v < -m) v = -m;
    return (int32_t)lroundf(v);
}

// Chay 1 buoc: lai -> cat -> troi den khi dung han -> nghi. Tra ve 0 neu het gio.
static uint8_t run_step(int k, uint32_t t0)
{
    const zz_step_t *s = &ST[k];
    zz_res_t *r = &RS[k];
    int32_t c_s[4];
    for (int i = 0; i < 4; i++) c_s[i] = cnt[i];

    const float cpm = COUNTS_PER_REV / (TWO_PI * WHEEL_R);         // xung / m banh lan
    const float cpd = K_GEOM * DEG2RAD * cpm;                      // xung / do khi quay
    const int8_t dir = (s->turn && s->val < 0.0f) ? -1 : 1;
    const int32_t tgt_cnt = (int32_t)(s->turn ? fabsf(s->val) * cpd : s->val * cpm);
    const uint32_t tmo = s->turn ? ZZ_TURN_TIMEOUT_MS : ENC_RUN_TIMEOUT_MS;
    const uint8_t by_cnt = (ZZ_MODE == 0) || (ZZ_MODE == 1 && (!s->turn || !ZZ_V2_TURN_IMU));
    const float lx = (k > 0) ? ST[k - 1].wx : 0.0f;               // diem dau doan (ly thuyet)
    const float ly = (k > 0) ? ST[k - 1].wy : 0.0f;
    const float cps = cosf(s->hdg * DEG2RAD), sps = sinf(s->hdg * DEG2RAD);

    r->t_start = HAL_GetTick() - t0;
    r->hit = -1;
    usart6_send_string("ZZ:BUOC k=");   usart6_send_int(k + 1);
    usart6_send_string(s->turn ? " loai=QUAY goc=" : " loai=THANG mm=");
    usart6_send_int(s->turn ? (int32_t)s->val : (int32_t)lroundf(s->val * 1000.0f));
    usart6_send_string(" huong_dich="); usart6_send_int((int32_t)s->hdg);
    usart6_send_string(" xung_dich=");  usart6_send_int(by_cnt ? tgt_cnt : 0);
    usart6_send_string(" t_ms=");       usart6_send_int((int32_t)r->t_start);
    usart6_send_string("\r\n");

    // ---------- Lai ----------
    const uint32_t ts = HAL_GetTick();
    uint32_t last_ctl = ts, last_log = ts;
    float hi = 0.0f, yi_i = 0.0f, u = 0.0f, sl = 0.0f;
    int32_t duty = s->turn ? ZZ_TURN_DUTY_START : ENC_RUN_DUTY_START;
    uint8_t near = 0, stop = 0;
    float e0 = 0.0f, turned = 0.0f, ph_p = 0.0f;      // goc can quay luc dau buoc, goc da quay (khong cat +-180)
    {
        float px0, py0;
        est_pose(&px0, &py0, &ph_p);
        e0 = s->val + wrap180(s->hdg - ph_p - s->val);
    }
    // Huong can giu khi di thang: V2 mac dinh giu huong luc bat dau doan (IMU chi chong troi),
    // V3 (va V2 khi ZZ_V2_HOLD_ABS = 1) giu huong dich tuyet doi
    const float h_ref = (ZZ_MODE == 1 && !ZZ_V2_HOLD_ABS) ? ph_p : s->hdg;
    (void)h_ref;
    moving = 1;

    while (1) {
        poll_enc();
        uint32_t now = HAL_GetTick();
        uint32_t t = now - ts;

        if (by_cnt) {
            for (int i = 0; i < 4; i++) {
                int32_t a = cnt[i] - c_s[i];
                if (a < 0) a = -a;
                if (a >= tgt_cnt) { r->hit = (int8_t)i; stop = 1; break; }
            }
        }
        if (!stop && t >= tmo) stop = 1;

        if (!stop && (now - last_ctl) >= CONTROL_DT_MS) {
            float dt = (float)(now - last_ctl) / 1000.0f;
            last_ctl = now;
            ctl_tick(dt, now);

            float px, py, ph;
            est_pose(&px, &py, &ph);

            if (!s->turn) {
                duty = ENC_RUN_DUTY;
                if (t < ENC_RUN_RAMP_MS)
                    duty = ENC_RUN_DUTY_START + (int32_t)((ENC_RUN_DUTY - ENC_RUN_DUTY_START) * (int32_t)t / (int32_t)ENC_RUN_RAMP_MS);
                u = 0.0f; sl = 0.0f;
                near = by_cnt && (max_abs_d(c_s) > tgt_cnt - ZZ_LOG_GUARD_CNT);
#if ZZ_MODE >= 1
                float e = wrap180(h_ref - ph);
                if (t >= ENC_RUN_RAMP_MS) {
                    hi += HH_KI * e * dt;
                    if (hi >  HH_I_MAX) hi =  HH_I_MAX;
                    if (hi < -HH_I_MAX) hi = -HH_I_MAX;
                }
                u = HH_KP * e + hi - HH_KD * gz_prev;
                if (u >  HH_U_MAX) u =  HH_U_MAX;
                if (u < -HH_U_MAX) u = -HH_U_MAX;
#endif
#if ZZ_MODE == 2
                float dx = px - lx, dy = py - ly;
                float al =  cps * dx + sps * dy;                   // doc doan
                float cr = -sps * dx + cps * dy;                   // lech ngang, > 0 = ben trai
                if (al >= s->val) { r->hit = 4; stop = 1; }
                near = (al > s->val - ZZ_LOG_GUARD_M);
                float cr_mm = cr * 1000.0f;
                if (t >= ENC_RUN_RAMP_MS) {
                    yi_i += ER_KIY * cr_mm * dt;
                    if (yi_i >  ER_SI_MAX) yi_i =  ER_SI_MAX;
                    if (yi_i < -ER_SI_MAX) yi_i = -ER_SI_MAX;
                }
                sl = -(ER_KY * cr_mm + yi_i);
                if (sl >  ER_S_MAX) sl =  ER_S_MAX;
                if (sl < -ER_S_MAX) sl = -ER_S_MAX;
#else
                (void)lx; (void)ly; (void)cps; (void)sps; (void)yi_i; (void)hi;
#endif
                if (!stop) {
                    int32_t du = clamp_i(u, HH_U_MAX), ds = clamp_i(sl, ER_S_MAX);
                    motor_set_duty_raw(duty - du - ds, duty + du + ds, duty - du + ds, duty + du - ds);
                }
            } else {
                float rem;                                         // goc con lai (do)
                if (by_cnt) {
                    rem = (float)(tgt_cnt - max_abs_d(c_s)) / cpd;
                } else {
                    turned += wrap180(ph - ph_p);
                    ph_p = ph;
                    rem = dir * (e0 - turned);
                    if (rem <= 0.0f) { r->hit = 4; stop = 1; }
                }
                near = (rem < ZZ_LOG_GUARD_DEG);
                duty = ZZ_TURN_DUTY;
                if (t < ZZ_TURN_RAMP_MS)
                    duty = ZZ_TURN_DUTY_START + (int32_t)((ZZ_TURN_DUTY - ZZ_TURN_DUTY_START) * (int32_t)t / (int32_t)ZZ_TURN_RAMP_MS);
                if (ZZ_TURN_SLOW_DEG > 0.0f && rem < ZZ_TURN_SLOW_DEG) {
                    int32_t ds = ZZ_TURN_DUTY_MIN + (int32_t)((ZZ_TURN_DUTY - ZZ_TURN_DUTY_MIN) * (rem > 0.0f ? rem : 0.0f) / ZZ_TURN_SLOW_DEG);
                    if (ds < duty) duty = ds;
                }
                if (!stop) {
                    int32_t d = dir * duty;
                    motor_set_duty_raw(-d, d, -d, d);
                }
            }
        }

        if (stop) {
            motor_set_duty_raw(0, 0, 0, 0);
            r->t_cut = t;
            break;
        }

        if ((now - last_log) >= ENC_RUN_LOG_EVERY_MS && !near) {
            last_log = now;
            log_line(now - t0, s->turn ? dir * duty : duty, u, sl);
        }
    }
    log_line(HAL_GetTick() - t0, 0, 0.0f, 0.0f);

    // ---------- Troi den khi dung han ----------
    uint32_t t_still = HAL_GetTick();
    while (1) {
        if (poll_enc()) t_still = HAL_GetTick();
        uint32_t now = HAL_GetTick();
        if ((now - last_ctl) >= CONTROL_DT_MS) {
            float dt = (float)(now - last_ctl) / 1000.0f;
            last_ctl = now;
            ctl_tick(dt, now);
        }
        if ((now - last_log) >= ENC_RUN_LOG_EVERY_MS) { last_log = now; log_line(now - t0, 0, 0.0f, 0.0f); }
        if ((now - t_still) >= ENC_RUN_REST_MS) break;
        if ((now - ts - r->t_cut) >= ENC_RUN_COAST_MAX_MS) break;
    }
    r->t_rest = t_still - ts;
    moving = 0;

    // ---------- Nghi: camera het tre, lay trung binh ----------
    uint32_t t_w = HAL_GetTick();
    av_x = av_y = av_s = av_c = 0.0f; av_n = 0; av_on = 0;
    while ((HAL_GetTick() - t_w) < ZZ_PAUSE_MS) {
        poll_enc();
        uint32_t now = HAL_GetTick();
        if ((now - last_ctl) >= CONTROL_DT_MS) {
            float dt = (float)(now - last_ctl) / 1000.0f;
            last_ctl = now;
            if (!av_on && (now - t_w) >= ZZ_PAUSE_MS - GOTO_SETTLE_AVG_MS) av_on = 1;
            ctl_tick(dt, now);
        }
        if ((now - last_log) >= ENC_RUN_LOG_EVERY_MS) { last_log = now; log_line(now - t0, 0, 0.0f, 0.0f); }
    }
    av_on = 0;

    r->ex = ex_; r->ey = ey_; r->eth = eth_ * RAD2DEG;
    est_pose(&r->px, &r->py, &r->ph);
    r->cam_ok = (av_n > 0);
    if (r->cam_ok) {
        to_start(av_x / (float)av_n, av_y / (float)av_n, &r->cx, &r->cy);
        r->ch = wrap_pi(atan2f(av_s, av_c) - th0) * RAD2DEG;
    }
    return (r->hit != -1);
}

static void print_res(int k)
{
    const zz_step_t *s = &ST[k];
    const zz_res_t  *r = &RS[k];
    usart6_send_string("ZZ:KQ k=");      usart6_send_int(k + 1);
    usart6_send_string(s->turn ? " loai=QUAY" : " loai=THANG");
    usart6_send_string(" cham=");
    if (r->hit < 0)       usart6_send_string("HET_GIO");
    else if (r->hit < 4)  usart6_send_string(WNAME[r->hit]);
    else                  usart6_send_string(ZZ_MODE == 2 ? "EKF" : "IMU");
    usart6_send_string(" t_bd_ms=");    usart6_send_int((int32_t)r->t_start);
    usart6_send_string(" t_cat_ms=");   usart6_send_int((int32_t)r->t_cut);
    usart6_send_string(" t_dung_ms=");  usart6_send_int((int32_t)r->t_rest);
    usart6_send_string(" dich=");       send_mm(s->wx); usart6_send_char(','); send_mm(s->wy);
    usart6_send_char(',');              usart6_send_int((int32_t)s->hdg);
    usart6_send_string(" enc=");        send_mm(r->ex); usart6_send_char(','); send_mm(r->ey);
    usart6_send_char(',');              usart6_send_float(r->eth, 1);
    usart6_send_string(ZZ_MODE == 2 ? " ekf=" : " imu="); send_mm(r->px); usart6_send_char(','); send_mm(r->py);
    usart6_send_char(',');              usart6_send_float(r->ph, 1);
    if (r->cam_ok) {
        usart6_send_string(" cam=");    send_mm(r->cx); usart6_send_char(','); send_mm(r->cy);
        usart6_send_char(',');          usart6_send_float(r->ch, 1);
        usart6_send_string(" sai_cam="); send_mm(r->cx - s->wx); usart6_send_char(','); send_mm(r->cy - s->wy);
        usart6_send_char(',');          usart6_send_float(wrap180(r->ch - s->hdg), 1);
    } else {
        usart6_send_string(" cam=KHONG");
    }
    usart6_send_string("\r\n");
}

void zigzag_test_run(uint8_t imu_ready)
{
    robot_set_velocity(0.0f, 0.0f, 0.0f);
    if (!imu_ready) {
        while (1) {
            usart6_send_string("ZZ:LOI - khong thay MPU6050, khong chay\r\n");
            HAL_Delay(2000);
        }
    }
    build_steps();

    usart6_send_string("ZZ:START mode=");  usart6_send_int(ZZ_MODE);
    usart6_send_string(ZZ_MODE == 0 ? " (thuan encoder)" : ZZ_MODE == 1 ? " (encoder+IMU)" : " (EKF+camera)");
    usart6_send_string(" dai=");           usart6_send_int((int32_t)lroundf(ZZ_LONG_M * 1000.0f));
    usart6_send_string(" ngan=");          usart6_send_int((int32_t)lroundf(ZZ_SHORT_M * 1000.0f));
    usart6_send_string(" duty_thang=");    usart6_send_int(ENC_RUN_DUTY);
    usart6_send_string(" duty_quay=");     usart6_send_int(ZZ_TURN_DUTY);
    usart6_send_string(" cham_dan_do=");   usart6_send_float(ZZ_TURN_SLOW_DEG, 1);
#if ZZ_MODE == 1
    usart6_send_string(ZZ_V2_TURN_IMU ? " quay=IMU" : " quay=XUNG");
    usart6_send_string(ZZ_V2_HOLD_ABS ? " giu=HUONG_DICH" : " giu=HUONG_DAU_DOAN");
#endif
    usart6_send_string(" | KP=");          usart6_send_float(HH_KP, 2);
    usart6_send_string(" KI=");            usart6_send_float(HH_KI, 2);
    usart6_send_string(" KD=");            usart6_send_float(HH_KD, 2);
    usart6_send_string(" KY=");            usart6_send_float(ER_KY, 2);
    usart6_send_string(" KIY=");           usart6_send_float(ER_KIY, 2);
    usart6_send_string("\r\n");

    // ---------- 1. Cho camera ----------
    uint32_t t_c = HAL_GetTick(), last = t_c, last_msg = t_c;
    float sx = 0.0f, sy = 0.0f, ss = 0.0f, sc = 0.0f;
    uint32_t n = 0;
    while (ZZ_MODE == 2 || ZZ_CAM_LOG) {
        uint32_t now = HAL_GetTick();
        if (ZZ_MODE == 2 && n >= GOTO_INIT_FRAMES) break;
        if (ZZ_MODE != 2 && (n >= GOTO_INIT_FRAMES || (now - t_c) >= ZZ_CAM_WAIT_MS)) break;
        if ((now - last) < CONTROL_DT_MS) continue;
        float dt = (float)(now - last) / 1000.0f;
        last = now;
        int32_t d[4];
        encoder_get_deltas(d);
        mpu6050_update(dt);
        pose_link_poll();
        if (pose_link_take_fresh()) {
            float th = pose_link_get_theta_deg() * DEG2RAD;
            sx += pose_link_get_x(); sy += pose_link_get_y();
            ss += sinf(th);          sc += cosf(th);
            n++;
        }
        if ((now - last_msg) >= 1000u) {
            last_msg = now;
            usart6_send_string("ZZ:CHO_CAMERA so_khung=");
            usart6_send_int((int32_t)n);
            usart6_send_string("\r\n");
        }
    }
    cam_on = (n > 0);
#if ZZ_MODE == 2
    ekf_init(sx / (float)n, sy / (float)n, atan2f(ss, sc), HAL_GetTick());
#endif
    usart6_send_string(cam_on ? "ZZ:CAMERA CO\r\n" : "ZZ:CAMERA KHONG (chi log encoder + IMU)\r\n");

    // ---------- 2. Dem nguoc, xe dung yen ----------
    cxp = cyp = cthp = 1e9f;
    gz_prev = 0.0f;
    moving = 0;
    cam_n = 0;
    yaw0 = mpu6050_get_yaw_deg();
    uint32_t t_cd = HAL_GetTick();
    last = t_cd; last_msg = t_cd - 1000u;
    av_x = av_y = av_s = av_c = 0.0f; av_n = 0; av_on = 1;
    while ((HAL_GetTick() - t_cd) < ENC_RUN_START_DELAY_MS) {
        poll_enc();
        uint32_t now = HAL_GetTick();
        if ((now - last) < CONTROL_DT_MS) continue;
        float dt = (float)(now - last) / 1000.0f;
        last = now;
        ctl_tick(dt, now);
        if ((now - last_msg) >= 1000u) {
            last_msg = now;
            usart6_send_string("ZZ:chay sau ");
            usart6_send_int((int32_t)((ENC_RUN_START_DELAY_MS - (now - t_cd) + 999u) / 1000u));
            usart6_send_string(" s\r\n");
        }
    }
    av_on = 0;

#if ZZ_MODE == 2
    xs = ekf_x(); ys = ekf_y(); th0 = ekf_th();
#else
    if (cam_on && av_n > 0) { xs = av_x / (float)av_n; ys = av_y / (float)av_n; th0 = atan2f(av_s, av_c); }
    else                    { xs = ys = th0 = 0.0f; }
#endif
    c0 = cosf(th0); s0 = sinf(th0);
    yaw0 = mpu6050_get_yaw_deg();
    yaw_prev = 0.0f;
    for (int i = 0; i < 4; i++) { cnt[i] = 0; acc[i] = 0; }
    ex_ = ey_ = eth_ = 0.0f;
    ix_ = iy_ = 0.0f;
    cam_nlog = 0;
    cam_gap_max = 0;

    if (cam_on) {
        usart6_send_string("ZZ:XUAT_PHAT x="); send_mm(xs);
        usart6_send_string(" y=");             send_mm(ys);
        usart6_send_string(" th=");            usart6_send_float(th0 * RAD2DEG, 2);
        usart6_send_string(" (he camera)\r\n");
        usart6_send_string("ENC,t_ms,duty,FL,FR,RL,RR,x_mm,y_mm,th_deg,yaw_deg,gz_dps,u,xi_mm,yi_mm,"
                           "xc_mm,yc_mm,thc_deg,cam,s\r\n");
    } else {
        usart6_send_string("ENC,t_ms,duty,FL,FR,RL,RR,x_mm,y_mm,th_deg,yaw_deg,gz_dps,u,xi_mm,yi_mm\r\n");
    }

    // ---------- 3. 9 buoc ----------
    const uint32_t t0 = HAL_GetTick();
    cam_good_ms = t0;
    int n_done = 0;
    for (int k = 0; k < N_STEPS && k < ZZ_STEP_LAST; k++) {
        uint8_t ok = run_step(k, t0);
        print_res(k);
        n_done = k + 1;
        if (!ok) {
            usart6_send_string("ZZ:HET_GIO o buoc ");
            usart6_send_int(k + 1);
            usart6_send_string(" - dung bai\r\n");
            break;
        }
    }
    const uint32_t t_total = HAL_GetTick() - t0;

    // ---------- 4. Ket qua ----------
    while (1) {
        usart6_send_string("===== KET QUA ZIGZAG =====\r\n");
        for (int k = 0; k < n_done; k++) print_res(k);
        usart6_send_string("tong: mode=");      usart6_send_int(ZZ_MODE);
        usart6_send_string(" so_buoc=");        usart6_send_int(n_done);
        usart6_send_string(" t_ms=");           usart6_send_int((int32_t)t_total);
        usart6_send_string(" mat_cam_lau_nhat_ms="); usart6_send_int((int32_t)cam_gap_max);
#if ZZ_MODE == 2
        usart6_send_string("\r\nekf: bias=");   usart6_send_float(ekf_bias() * RAD2DEG, 3);
        usart6_send_string(" do/s  khung_nhan="); usart6_send_int((int32_t)ekf_n_ok());
        usart6_send_string("  khung_loai=");    usart6_send_int((int32_t)ekf_n_reject());
        usart6_send_string("  nap_lai=");       usart6_send_int((int32_t)ekf_n_reset());
#endif
        usart6_send_string("\r\n");
        HAL_Delay(5000);
    }
}
