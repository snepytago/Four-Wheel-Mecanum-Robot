#include "ekf.h"
#include "config.h"
#include <math.h>

#define DEG2RAD  0.017453293f
#define HIST_N   64u            // 64 x 20 ms = 1.28 s lich su de bu do tre camera

static float s[4];              // X, Y, TH, B
static float P[4][4];

static float    hx[HIST_N], hy[HIST_N], hth[HIST_N];
static uint32_t ht[HIST_N];
static uint16_t h_head, h_cnt;

static uint32_t n_ok, n_rej, n_rej_run, n_reset;
static float    reset_jump;     // do nhay vi tri lan nap lai gan nhat (m)

static float wrap_pi(float a)
{
    while (a >  PI) a -= TWO_PI;
    while (a < -PI) a += TWO_PI;
    return a;
}

static void hist_push(uint32_t t)
{
    hx[h_head] = s[0]; hy[h_head] = s[1]; hth[h_head] = s[2]; ht[h_head] = t;
    h_head = (uint16_t)((h_head + 1u) % HIST_N);
    if (h_cnt < HIST_N) h_cnt++;
}

// Trang thai moi nhat co thoi diem <= t (hoac cu nhat con giu)
static uint16_t hist_find(uint32_t t)
{
    uint16_t idx = (uint16_t)((h_head + HIST_N - 1u) % HIST_N);
    for (uint16_t k = 0; k < h_cnt; k++) {
        if ((int32_t)(t - ht[idx]) >= 0) return idx;
        if (k + 1u < h_cnt) idx = (uint16_t)((idx + HIST_N - 1u) % HIST_N);
    }
    return idx;
}

static void symmetrize(void)
{
    for (int i = 0; i < 4; i++)
        for (int j = i + 1; j < 4; j++) {
            float m = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = m; P[j][i] = m;
        }
}

void ekf_init(float x, float y, float th, uint32_t t_ms)
{
    s[0] = x; s[1] = y; s[2] = wrap_pi(th); s[3] = 0.0f;

    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) P[i][j] = 0.0f;
    float rxy = EKF_R_XY_M;
    float rth = EKF_R_TH_DEG * DEG2RAD;
    float b0  = EKF_B0_DPS * DEG2RAD;
    P[0][0] = rxy * rxy;
    P[1][1] = rxy * rxy;
    P[2][2] = rth * rth;
    P[3][3] = b0 * b0;

    h_head = 0; h_cnt = 0;
    hist_push(t_ms);
    n_ok = n_rej = n_rej_run = n_reset = 0;
    reset_jump = 0.0f;
}

void ekf_set_pose(float x, float y, float th, uint32_t t_ms)
{
    float b = s[3], pb = P[3][3];
    uint32_t ok = n_ok, rej = n_rej, rs = n_reset;
    ekf_init(x, y, th, t_ms);
    s[3] = b; P[3][3] = pb;
    n_ok = ok; n_rej = rej; n_reset = rs;
}

void ekf_predict(float dx, float dy, float gz, float dt, uint32_t t_ms)
{
    float w  = gz - s[3];
    float tm = s[2] + 0.5f * w * dt;
    float c  = cosf(tm), sn = sinf(tm);

    s[0] += c * dx - sn * dy;
    s[1] += sn * dx + c * dy;
    s[2]  = wrap_pi(s[2] + w * dt);

    // Jacobian F = I + cac thanh phan duoi
    float a = -sn * dx - c * dy;    // dX/dTH
    float b =  c * dx - sn * dy;    // dY/dTH
    float F[4][4] = {
        { 1.0f, 0.0f, a,    -0.5f * dt * a },
        { 0.0f, 1.0f, b,    -0.5f * dt * b },
        { 0.0f, 0.0f, 1.0f, -dt            },
        { 0.0f, 0.0f, 0.0f,  1.0f          },
    };

    float FP[4][4];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            float acc = 0.0f;
            for (int k = 0; k < 4; k++) acc += F[i][k] * P[k][j];
            FP[i][j] = acc;
        }
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            float acc = 0.0f;
            for (int k = 0; k < 4; k++) acc += FP[i][k] * F[j][k];
            P[i][j] = acc;
        }

    // Q: sai so do doi encoder (ti le quang duong, doc / ngang than xe) xoay sang G
    float ds = sqrtf(dx * dx + dy * dy);
    float sx = EKF_Q_ENC_X * ds + EKF_Q_ENC_MIN_M;
    float sy = EKF_Q_ENC_Y * ds + EKF_Q_ENC_MIN_M;
    float qx = sx * sx, qy = sy * sy;
    P[0][0] += c * c * qx + sn * sn * qy;
    P[1][1] += sn * sn * qx + c * c * qy;
    P[0][1] += c * sn * (qx - qy);
    P[1][0] += c * sn * (qx - qy);

    float sth = EKF_Q_GYRO_DPS * DEG2RAD * dt + EKF_Q_GYRO_SCALE * fabsf(w * dt);
    P[2][2] += sth * sth;
    float qb = EKF_Q_BIAS_DPS * DEG2RAD;
    P[3][3] += qb * qb * dt;

    symmetrize();
    hist_push(t_ms);
}

uint8_t ekf_update_cam(float xc, float yc, float thc, uint32_t t_meas_ms)
{
    uint16_t k = hist_find(t_meas_ms);
    float r[3] = { xc - hx[k], yc - hy[k], wrap_pi(thc - hth[k]) };

    float rxy = EKF_R_XY_M, rth = EKF_R_TH_DEG * DEG2RAD;
    float S[3][3];
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) S[i][j] = P[i][j];
    S[0][0] += rxy * rxy;
    S[1][1] += rxy * rxy;
    S[2][2] += rth * rth;

    float c00 = S[1][1] * S[2][2] - S[1][2] * S[2][1];
    float c01 = S[1][2] * S[2][0] - S[1][0] * S[2][2];
    float c02 = S[1][0] * S[2][1] - S[1][1] * S[2][0];
    float det = S[0][0] * c00 + S[0][1] * c01 + S[0][2] * c02;
    if (fabsf(det) < 1e-30f) return 0;
    float id = 1.0f / det;
    float Si[3][3] = {
        { c00 * id, (S[0][2] * S[2][1] - S[0][1] * S[2][2]) * id, (S[0][1] * S[1][2] - S[0][2] * S[1][1]) * id },
        { c01 * id, (S[0][0] * S[2][2] - S[0][2] * S[2][0]) * id, (S[0][2] * S[1][0] - S[0][0] * S[1][2]) * id },
        { c02 * id, (S[0][1] * S[2][0] - S[0][0] * S[2][1]) * id, (S[0][0] * S[1][1] - S[0][1] * S[1][0]) * id },
    };

    // Cong Mahalanobis: loai khung nhay bat thuong (nhan nham marker, nhieu)
    float d2 = 0.0f;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) d2 += r[i] * Si[i][j] * r[j];

    if (d2 > EKF_GATE) {
        n_rej++;
        if (++n_rej_run >= EKF_REJECT_RESET) {
            // Sua 19/10/2026: nap lai CO BU TRE. Khung camera mo ta luc t_meas, nen
            // cong do lech (z - trang thai luc t_meas) vao hien tai va ca lich su,
            // khong gan thang z (cu ~200 ms) cho hien tai nhu ban 18/10.
            reset_jump = sqrtf(r[0] * r[0] + r[1] * r[1]);
            s[0] += r[0];
            s[1] += r[1];
            s[2]  = wrap_pi(s[2] + r[2]);
            for (uint16_t i = 0; i < HIST_N; i++) {
                hx[i] += r[0];
                hy[i] += r[1];
                hth[i] = wrap_pi(hth[i] + r[2]);
            }
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 4; j++) { P[i][j] = 0.0f; P[j][i] = 0.0f; }
            P[0][0] = rxy * rxy;
            P[1][1] = rxy * rxy;
            P[2][2] = rth * rth;
            n_rej_run = 0;
            n_reset++;
            return 2;
        }
        return 0;
    }
    n_rej_run = 0;

    // K = P H^T S^-1  (H = [I3 0])
    float K[4][3];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 3; j++) {
            float acc = 0.0f;
            for (int m = 0; m < 3; m++) acc += P[i][m] * Si[m][j];
            K[i][j] = acc;
        }

    float d[4];
    for (int i = 0; i < 4; i++) d[i] = K[i][0] * r[0] + K[i][1] * r[1] + K[i][2] * r[2];

    s[0] += d[0];
    s[1] += d[1];
    s[2]  = wrap_pi(s[2] + d[2]);
    s[3] += d[3];

    // P = (I - K H) P
    float Pn[4][4];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            Pn[i][j] = P[i][j] - (K[i][0] * P[0][j] + K[i][1] * P[1][j] + K[i][2] * P[2][j]);
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) P[i][j] = Pn[i][j];
    symmetrize();

    // Doi ca lich su theo cung luong hieu chinh, neu khong khung sau se
    // so voi trang thai cu va cong hieu chinh nay them lan nua.
    for (uint16_t i = 0; i < HIST_N; i++) {
        hx[i] += d[0];
        hy[i] += d[1];
        hth[i] = wrap_pi(hth[i] + d[2]);
    }

    n_ok++;
    return 1;
}

float ekf_x(void)    { return s[0]; }
float ekf_y(void)    { return s[1]; }
float ekf_th(void)   { return s[2]; }
float ekf_bias(void) { return s[3]; }

float ekf_sigma_xy(void)
{
    float m = (P[0][0] > P[1][1]) ? P[0][0] : P[1][1];
    return sqrtf(m > 0.0f ? m : 0.0f);
}

uint32_t ekf_n_ok(void)     { return n_ok; }
uint32_t ekf_n_reject(void) { return n_rej; }
uint32_t ekf_n_reset(void)  { return n_reset; }
float    ekf_reset_jump(void) { return reset_jump; }
