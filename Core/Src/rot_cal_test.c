#include "rot_cal_test.h"
#include "config.h"
#include "encoder.h"
#include "kinematics.h"
#include "motor.h"
#include "mpu6050.h"
#include "robot_control.h"
#include "usart6.h"
#include "stm32f4xx_hal.h"
#include <math.h>

#define DEG2RAD  0.017453293f
#define RAD2DEG  57.29577951f

static const char *WNAME[4] = { "FL", "FR", "RL", "RR" };

typedef struct {
    int8_t   dir;
    int8_t   hit;          // banh cham moc, -1 = het gio
    uint32_t t_cut, t_rest;
    float    enc_cut, enc_rest;   // goc encoder (do), tu dau lan quay
    float    imu_cut, imu_rest;   // goc IMU (do), tu dau lan quay
    float    gz_max;
    float    tx, ty;              // tinh tien banh lan (mm), he than xe luc bat dau quay
} rc_res_t;

static rc_res_t R[RC_N_TURNS];

static int32_t cnt[4], acc[4];
static float   ex_, ey_, eth_;
static float   ix_, iy_, yaw_prev;
static float   yaw0, gz_max;

static void send_mm(float m) { usart6_send_int((int32_t)lroundf(m * 1000.0f)); }

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

static void ctl_tick(float dt)
{
    mpu6050_update(dt);
    odo_step();
    float g = fabsf(mpu6050_get_gyro_z_dps());
    if (g > gz_max) gz_max = g;
}

// Goc va tinh tien banh lan tu xung dem ke tu c0 (FK, khong bu truot)
static void turn_fk(const int32_t c0[4], float *th_deg, float *tx, float *ty)
{
    const float k = TWO_PI / COUNTS_PER_REV;
    float rx, ry, rth;
    forward_kinematics((cnt[MOTOR_FL] - c0[MOTOR_FL]) * k, (cnt[MOTOR_FR] - c0[MOTOR_FR]) * k,
                       (cnt[MOTOR_RL] - c0[MOTOR_RL]) * k, (cnt[MOTOR_RR] - c0[MOTOR_RR]) * k,
                       &rx, &ry, &rth);
    *th_deg = rth * RAD2DEG;
    if (tx) *tx = rx;
    if (ty) *ty = ry;
}

static int32_t max_abs_d(const int32_t c0[4])
{
    int32_t m = 0;
    for (int i = 0; i < 4; i++) {
        int32_t a = cnt[i] - c0[i];
        if (a < 0) a = -a;
        if (a > m) m = a;
    }
    return m;
}

static void log_line(uint32_t t, int32_t duty, int k)
{
    usart6_send_string("ENC,");
    usart6_send_int((int32_t)t);   usart6_send_char(',');
    usart6_send_int(duty);
    for (int i = 0; i < 4; i++) { usart6_send_char(','); usart6_send_int(cnt[i]); }
    usart6_send_char(','); send_mm(ex_);
    usart6_send_char(','); send_mm(ey_);
    usart6_send_char(','); usart6_send_float(eth_ * RAD2DEG, 2);
    usart6_send_char(','); usart6_send_float(yaw_rel_deg(), 2);
    usart6_send_char(','); usart6_send_float(mpu6050_get_gyro_z_dps(), 2);
    usart6_send_char(','); usart6_send_int(k);
    usart6_send_char(','); send_mm(ix_);
    usart6_send_char(','); send_mm(iy_);
    usart6_send_string("\r\n");
}

static int8_t turn_dir(int k)
{
#if RC_PATTERN
    return (int8_t)((((k / 2) % 2) == 0 ? 1 : -1) * RC_DIR);
#else
    (void)k;
    return (int8_t)RC_DIR;
#endif
}

static void print_res(int k, const rc_res_t *r)
{
    float vot  = (r->imu_rest - r->imu_cut) * r->dir;
    float krot = (fabsf(r->enc_rest) > 1.0f) ? r->imu_rest / r->enc_rest : 0.0f;
    float wz   = (r->t_cut > 0) ? fabsf(r->imu_cut) / ((float)r->t_cut / 1000.0f) : 0.0f;

    usart6_send_string("QUAY:KQ k=");     usart6_send_int(k + 1);
    usart6_send_string(" huong=");        usart6_send_string(r->dir > 0 ? "TRAI" : "PHAI");
    usart6_send_string(" cham=");         usart6_send_string(r->hit >= 0 ? WNAME[r->hit] : "HET_GIO");
    usart6_send_string(" t_cat_ms=");     usart6_send_int((int32_t)r->t_cut);
    usart6_send_string(" t_dung_ms=");    usart6_send_int((int32_t)r->t_rest);
    usart6_send_string(" enc_cat=");      usart6_send_float(r->enc_cut, 2);
    usart6_send_string(" enc_dung=");     usart6_send_float(r->enc_rest, 2);
    usart6_send_string(" imu_cat=");      usart6_send_float(r->imu_cut, 2);
    usart6_send_string(" imu_dung=");     usart6_send_float(r->imu_rest, 2);
    usart6_send_string(" vot=");          usart6_send_float(vot, 2);
    usart6_send_string(" k_rot=");        usart6_send_float(krot, 3);
    usart6_send_string(" wz_tb=");        usart6_send_float(wz, 1);
    usart6_send_string(" gz_max=");       usart6_send_float(r->gz_max, 1);
    usart6_send_string(" tx_mm=");        send_mm(r->tx);
    usart6_send_string(" ty_mm=");        send_mm(r->ty);
    usart6_send_string("\r\n");
}

void rot_cal_test_run(uint8_t imu_ready)
{
    robot_set_velocity(0.0f, 0.0f, 0.0f);
    if (!imu_ready) {
        while (1) {
            usart6_send_string("QUAY:LOI - khong thay MPU6050, khong chay\r\n");
            HAL_Delay(2000);
        }
    }

    const int32_t target = RC_TARGET_COUNTS;
    usart6_send_string("QUAY:START goc=");  usart6_send_int((int32_t)RC_ANGLE_DEG);
    usart6_send_string(" do muc_tieu=");    usart6_send_int(target);
    usart6_send_string(" xung so_lan=");    usart6_send_int(RC_N_TURNS);
    usart6_send_string(" kieu=");           usart6_send_string(RC_PATTERN ? "TRAI,TRAI,PHAI,PHAI" : "CUNG_HUONG");
    usart6_send_string(" duty=");           usart6_send_int(RC_DUTY);
    usart6_send_string("\r\n");

    // ---------- 1. Dem nguoc, xe dung yen ----------
    uint32_t t_cd = HAL_GetTick(), last = t_cd, last_msg = t_cd - 1000u;
    while ((HAL_GetTick() - t_cd) < ENC_RUN_START_DELAY_MS) {
        uint32_t now = HAL_GetTick();
        if ((now - last) < CONTROL_DT_MS) continue;
        float dt = (float)(now - last) / 1000.0f;
        last = now;
        mpu6050_update(dt);
        poll_enc();
        if ((now - last_msg) >= 1000u) {
            last_msg = now;
            usart6_send_string("QUAY:chay sau ");
            usart6_send_int((int32_t)((ENC_RUN_START_DELAY_MS - (now - t_cd) + 999u) / 1000u));
            usart6_send_string(" s\r\n");
        }
    }
    yaw0 = mpu6050_get_yaw_deg();
    yaw_prev = 0.0f;
    for (int i = 0; i < 4; i++) { cnt[i] = 0; acc[i] = 0; }
    ex_ = ey_ = eth_ = 0.0f;
    ix_ = iy_ = 0.0f;
    usart6_send_string("ENC,t_ms,duty,FL,FR,RL,RR,x_mm,y_mm,th_deg,yaw_deg,gz_dps,u,xi_mm,yi_mm\r\n");

    const uint32_t t0 = HAL_GetTick();
    uint32_t last_ctl = t0, last_log = t0;

    for (int k = 0; k < RC_N_TURNS; k++) {
        rc_res_t *r = &R[k];
        int32_t c0[4];
        for (int i = 0; i < 4; i++) c0[i] = cnt[i];
        const float yaw_s = yaw_rel_deg();
        r->dir = turn_dir(k);
        r->hit = -1;
        gz_max = 0.0f;

        usart6_send_string("QUAY:BAT_DAU k="); usart6_send_int(k + 1);
        usart6_send_string(" huong=");         usart6_send_string(r->dir > 0 ? "TRAI" : "PHAI");
        usart6_send_string(" t_ms=");          usart6_send_int((int32_t)(HAL_GetTick() - t0));
        usart6_send_string("\r\n");

        // ---------- 2. Quay, kiem tra moc sau moi lan doc encoder ----------
        const uint32_t ts = HAL_GetTick();
        int32_t duty = RC_DUTY_START;
        while (1) {
            poll_enc();
            uint32_t now = HAL_GetTick();
            uint32_t t   = now - ts;

            for (int i = 0; i < 4; i++) {
                int32_t a = cnt[i] - c0[i];
                if (a < 0) a = -a;
                if (a >= target) { r->hit = (int8_t)i; break; }
            }
            if (r->hit >= 0 || t >= RC_TIMEOUT_MS) {
                motor_set_duty_raw(0, 0, 0, 0);
                r->t_cut = t;
                break;
            }

            if ((now - last_ctl) >= CONTROL_DT_MS) {
                float dt = (float)(now - last_ctl) / 1000.0f;
                last_ctl = now;
                ctl_tick(dt);

                duty = RC_DUTY;
                if (t < RC_RAMP_MS)
                    duty = RC_DUTY_START + (int32_t)((RC_DUTY - RC_DUTY_START) * (int32_t)t / (int32_t)RC_RAMP_MS);
                int32_t d = r->dir * duty;
                motor_set_duty_raw(-d, d, -d, d);
            }

            if ((now - last_log) >= ENC_RUN_LOG_EVERY_MS && max_abs_d(c0) < target - RC_LOG_GUARD) {
                last_log = now;
                log_line(now - t0, r->dir * duty, k + 1);
            }
        }
        r->imu_cut = yaw_rel_deg() - yaw_s;
        turn_fk(c0, &r->enc_cut, 0, 0);
        log_line(HAL_GetTick() - t0, 0, k + 1);

        // ---------- 3. Troi den khi dung han ----------
        uint32_t t_still = HAL_GetTick();
        while (1) {
            if (poll_enc()) t_still = HAL_GetTick();
            uint32_t now = HAL_GetTick();
            if ((now - last_ctl) >= CONTROL_DT_MS) {
                float dt = (float)(now - last_ctl) / 1000.0f;
                last_ctl = now;
                ctl_tick(dt);
            }
            if ((now - last_log) >= ENC_RUN_LOG_EVERY_MS) { last_log = now; log_line(now - t0, 0, k + 1); }
            if ((now - t_still) >= ENC_RUN_REST_MS) break;
            if ((now - ts - r->t_cut) >= ENC_RUN_COAST_MAX_MS) break;
        }
        r->t_rest = t_still - ts;

        // ---------- 4. Nghi, than xe het lac ----------
        uint32_t t_w = HAL_GetTick();
        while ((HAL_GetTick() - t_w) < RC_PAUSE_MS) {
            poll_enc();
            uint32_t now = HAL_GetTick();
            if ((now - last_ctl) >= CONTROL_DT_MS) {
                float dt = (float)(now - last_ctl) / 1000.0f;
                last_ctl = now;
                ctl_tick(dt);
            }
            if ((now - last_log) >= ENC_RUN_LOG_EVERY_MS) { last_log = now; log_line(now - t0, 0, k + 1); }
        }
        r->imu_rest = yaw_rel_deg() - yaw_s;
        turn_fk(c0, &r->enc_rest, &r->tx, &r->ty);
        r->gz_max = gz_max;
        print_res(k, r);
    }

    // ---------- 5. Tong hop ----------
    float krot_s[2] = { 0, 0 }, vot_s[2] = { 0, 0 }, cut_s[2] = { 0, 0 }, wz_s[2] = { 0, 0 }, gzm = 0.0f;
    int   n_s[2] = { 0, 0 };
    float cmd_sum = 0.0f;
    for (int k = 0; k < RC_N_TURNS; k++) {
        const rc_res_t *r = &R[k];
        cmd_sum += r->dir * RC_ANGLE_DEG;
        if (r->gz_max > gzm) gzm = r->gz_max;
        if (r->hit < 0 || fabsf(r->enc_rest) < 1.0f) continue;
        int j = r->dir > 0 ? 0 : 1;
        krot_s[j] += r->imu_rest / r->enc_rest;
        vot_s[j]  += (r->imu_rest - r->imu_cut) * r->dir;
        cut_s[j]  += fabsf(r->imu_cut);
        wz_s[j]   += (r->t_cut > 0) ? fabsf(r->imu_cut) / ((float)r->t_cut / 1000.0f) : 0.0f;
        n_s[j]++;
    }
    const float yaw_end = yaw_rel_deg();

    while (1) {
        usart6_send_string("===== KET QUA QUAY =====\r\n");
        for (int k = 0; k < RC_N_TURNS; k++) print_res(k, &R[k]);
        for (int j = 0; j < 2; j++) {
            usart6_send_string(j == 0 ? "tb_TRAI: n=" : "tb_PHAI: n=");
            usart6_send_int(n_s[j]);
            if (n_s[j] > 0) {
                float n = (float)n_s[j];
                float cut = cut_s[j] / n, vot = vot_s[j] / n;
                usart6_send_string(" k_rot=");    usart6_send_float(krot_s[j] / n, 3);
                usart6_send_string(" imu_cat=");  usart6_send_float(cut, 2);
                usart6_send_string(" vot=");      usart6_send_float(vot, 2);
                usart6_send_string(" wz_tb_dps="); usart6_send_float(wz_s[j] / n, 1);
                // So xung cat de goc dung han = RC_ANGLE_DEG (tuyen tinh theo lan do nay)
                usart6_send_string(" xung_de_nghi=");
                usart6_send_int(cut > 1.0f ? (int32_t)lroundf(target * (RC_ANGLE_DEG - vot) / cut) : 0);
            }
            usart6_send_string("\r\n");
        }
        usart6_send_string("tong: goc_lenh=");  usart6_send_float(cmd_sum, 1);
        usart6_send_string(" yaw_cuoi=");       usart6_send_float(yaw_end, 2);
        usart6_send_string(" lech_tich_luy=");  usart6_send_float(yaw_end - cmd_sum, 2);
        usart6_send_string(" toa_do_enc x=");   send_mm(ex_);
        usart6_send_string(" y=");              send_mm(ey_);
        usart6_send_string(" mm gz_max=");      usart6_send_float(gzm, 1);
        usart6_send_string(gzm > RC_GZ_SAT_DPS ? " CANH_BAO:gyro_gan_bao_hoa\r\n" : "\r\n");
        HAL_Delay(5000);
    }
}
