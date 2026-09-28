#include "goto_test.h"
#include "config.h"
#include "encoder.h"
#include "kinematics.h"
#include "mpu6050.h"
#include "odometry.h"
#include "pose_link.h"
#include "robot_control.h"
#include "usart6.h"
#include "stm32f4xx_hal.h"
#include <math.h>

#define DEG2RAD  0.017453293f

enum { ST_RUN = 0, ST_TOI_DICH, ST_HET_GIO, ST_ENC_FAIL, ST_CAM_VUOT, ST_CAM_LECH };

static const char *WNAME[4] = { "FL", "FR", "RL", "RR" };

static float wrap180(float a)
{
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static void send_mm(float m) { usart6_send_int((int32_t)(m * 1000.0f)); }

// Toc do robot nho nhat de banh CHAM NHAT van vuot nguong khoi dong, voi
// huong di (ux_b, uy_b) trong he than xe. Theo IK: |w| banh = |vx -+ vy| / r.
static float v_floor_for(float ux_b, float uy_b)
{
    float a = fabsf(ux_b - uy_b);   // FL, RR
    float b = fabsf(ux_b + uy_b);   // FR, RL
    float k = ((a < b) ? a : b) / WHEEL_R;
    if (k < 1e-3f) return GTP_V_MAX;
    float v = GTP_WHEEL_MARGIN * MOTOR_OMEGA_MIN / k;
    return (v > GTP_V_MAX) ? GTP_V_MAX : v;
}

static int32_t iabs(int32_t v) { return (v < 0) ? -v : v; }

void goto_test_run(uint8_t imu_ready)
{
    int32_t dcnt[4];

    robot_set_velocity(0.0f, 0.0f, 0.0f);

    usart6_send_string("GTP:START dich=(");
    send_mm(GTP_GOAL_X_M); usart6_send_char(';'); send_mm(GTP_GOAL_Y_M);
    usart6_send_string(")mm che_do=");
    usart6_send_string(GTP_USE_CAMERA ? "ODO+CAMERA\r\n" : "CHI_ODO (camera = trong tai)\r\n");
    usart6_send_string("GTP:cho pose tu camera...\r\n");

    uint32_t t_wait = HAL_GetTick();
    while (!pose_link_valid() && (HAL_GetTick() - t_wait) < GTP_CAM_WAIT_MS) {
        pose_link_poll();
    }

    // --- Diem xuat phat: trung binh pose camera khi dung yen ---
    float xs = 0.0f, ys = 0.0f, ths = 0.0f;
    uint8_t cam_start = pose_link_valid();

    if (cam_start) {
        float th_ref = pose_link_get_theta_deg();
        float sx = 0.0f, sy = 0.0f, sth = 0.0f;
        uint32_t n = 0;
        uint32_t t0 = HAL_GetTick();
        while ((HAL_GetTick() - t0) < GTP_START_AVG_MS) {
            pose_link_poll();
            if (pose_link_take_fresh()) {
                sx  += pose_link_get_x();
                sy  += pose_link_get_y();
                sth += wrap180(pose_link_get_theta_deg() - th_ref);
                n++;
            }
        }
        if (n > 0) {
            xs  = sx / (float)n;
            ys  = sy / (float)n;
            ths = th_ref + sth / (float)n;
        } else {
            xs  = pose_link_get_x();
            ys  = pose_link_get_y();
            ths = th_ref;
        }
        usart6_send_string("GTP:xuat_phat_cam=(");
        send_mm(xs); usart6_send_char(';'); send_mm(ys);
        usart6_send_string(")mm th=");
        usart6_send_float(ths, 2);
        usart6_send_string(" so_khung=");
        usart6_send_int((int32_t)n);
        usart6_send_string("\r\n");
    } else {
        usart6_send_string("GTP:KHONG CO CAMERA - coi xuat phat la (0;0;0), khong co trong tai, TAT phanh camera\r\n");
    }

    if (imu_ready) mpu6050_set_yaw_deg(ths);
    encoder_get_deltas(dcnt);
    odometry_set_pose(xs, ys);
    (void)pose_link_take_fresh();

    // Duong thang ly tuong S -> G
    const float lx = GTP_GOAL_X_M - xs;
    const float ly = GTP_GOAL_Y_M - ys;
    const float L  = sqrtf(lx * lx + ly * ly);
    const float ulx = (L > 1e-6f) ? lx / L : 1.0f;
    const float uly = (L > 1e-6f) ? ly / L : 0.0f;

    float cross_odo = 0.0f, cross_cam = 0.0f, along_cam = 0.0f;
    float cross_odo_max = 0.0f, cross_cam_max = 0.0f;
    float th_err_max = 0.0f;
    float v_cmd = 0.0f;
    uint8_t  state = ST_RUN;
    uint32_t t_stop = 0;

    // Watchdog encoder: lenh banh cua chu ky TRUOC (dang tac dung trong khoang
    // vua do), so chu ky lien tiep banh bao it xung bat thuong, banh bi loi.
    float    w_cmd_prev[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    uint16_t enc_bad[4]    = { 0, 0, 0, 0 };
    int      enc_fail_w    = -1;
    const uint16_t ENC_BAD_CYCLES = (uint16_t)(GTP_ENC_WD_MS / CONTROL_DT_MS);

    int32_t cnt_log[4] = { 0, 0, 0, 0 };   // tong xung tung banh tu lan log truoc
    int32_t cnt_tot[4] = { 0, 0, 0, 0 };   // tong xung tung banh ca hanh trinh

    usart6_send_string("GTP,t_ms,v_cmd,x_odo,y_odo,yaw,cam_ok,x_cam,y_cam,th_cam,cross_odo,cross_cam,dist_odo,dFL,dFR,dRL,dRR\r\n");

    const uint32_t t_start = HAL_GetTick();
    uint32_t last_step = t_start, last_dt = t_start, last_log = t_start - GTP_LOG_EVERY_MS;

    while (1) {
        uint32_t now = HAL_GetTick();
        uint32_t t   = now - t_start;

        pose_link_poll();

        if ((now - last_step) < CONTROL_DT_MS) continue;
        last_step = now;

        float dt = (float)(now - last_dt) / 1000.0f;
        last_dt = now;

        encoder_get_deltas(dcnt);
        for (int i = 0; i < 4; i++) { cnt_log[i] += dcnt[i]; cnt_tot[i] += dcnt[i]; }

        if (imu_ready) mpu6050_update(dt);
        odometry_update(dcnt, dt);

        uint8_t fresh = pose_link_take_fresh();

#if GTP_USE_CAMERA
        if (fresh) {
            odometry_set_pose(pose_link_get_x(), pose_link_get_y());
            float yaw_now = mpu6050_get_yaw_deg();
            float d = wrap180(pose_link_get_theta_deg() - yaw_now);
            mpu6050_set_yaw_deg(yaw_now + POSE_YAW_ALPHA * d);
        }
#endif

        float x  = odometry_get_x();
        float y  = odometry_get_y();
        float th = odometry_get_theta_deg();

        cross_odo = ulx * (y - ys) - uly * (x - xs);
        if (fabsf(cross_odo) > cross_odo_max) cross_odo_max = fabsf(cross_odo);

        if (fresh) {
            float xc = pose_link_get_x(), yc = pose_link_get_y();
            cross_cam = ulx * (yc - ys) - uly * (xc - xs);
            along_cam = ulx * (xc - xs) + uly * (yc - ys);
            if (fabsf(cross_cam) > cross_cam_max) cross_cam_max = fabsf(cross_cam);
        }

        float ex   = GTP_GOAL_X_M - x;
        float ey   = GTP_GOAL_Y_M - y;
        float dist = sqrtf(ex * ex + ey * ey);

        float th_err = wrap180(GTP_HEADING_DEG - th);

        if (state == ST_RUN) {
            // --- Watchdog encoder ---
            for (int i = 0; i < 4 && state == ST_RUN; i++) {
                float expect = fabsf(w_cmd_prev[i]) * dt / TWO_PI * COUNTS_PER_REV;
                if (fabsf(w_cmd_prev[i]) >= GTP_ENC_WD_MIN_W &&
                    (float)iabs(dcnt[i]) < GTP_ENC_WD_RATIO * expect) {
                    if (++enc_bad[i] >= ENC_BAD_CYCLES) {
                        state = ST_ENC_FAIL; t_stop = t; enc_fail_w = i;
                    }
                } else {
                    enc_bad[i] = 0;
                }
            }

            // --- Phanh an toan bang camera (chi khi co camera luc xuat phat) ---
            if (state == ST_RUN && cam_start && pose_link_valid()) {
                if (along_cam > L + GTP_CAM_OVERSHOOT_M)       { state = ST_CAM_VUOT; t_stop = t; }
                else if (fabsf(cross_cam) > GTP_CAM_CROSS_MAX_M) { state = ST_CAM_LECH; t_stop = t; }
            }

            if (state == ST_RUN) {
                if (dist < GTP_TOL_M)        { state = ST_TOI_DICH; t_stop = t; }
                else if (t > GTP_TIMEOUT_MS) { state = ST_HET_GIO;  t_stop = t; }
            }
        }

        if (state != ST_RUN) {
            robot_set_velocity(0.0f, 0.0f, 0.0f);
            for (int i = 0; i < 4; i++) w_cmd_prev[i] = 0.0f;
            v_cmd = 0.0f;
            if ((t - t_stop) >= GTP_SETTLE_MS) break;
        } else {
            if (fabsf(th_err) > th_err_max) th_err_max = fabsf(th_err);

            float thr = th * DEG2RAD;
            float c = cosf(thr), s = sinf(thr);
            float uxw = ex / dist, uyw = ey / dist;
            float uxb =  uxw * c + uyw * s;
            float uyb = -uxw * s + uyw * c;

            float v = GTP_KP_POS * dist;
            if (v > GTP_V_MAX) v = GTP_V_MAX;
            if (t < GTP_RAMP_MS) {
                float vr = GTP_V_MAX * (float)t / (float)GTP_RAMP_MS;
                if (v > vr) v = vr;
            }
            float vf = v_floor_for(uxb, uyb);
            if (v < vf) v = vf;
            v_cmd = v;

            float wz = HEADING_KP * th_err;
            if (wz >  HEADING_MAX_WZ) wz =  HEADING_MAX_WZ;
            if (wz < -HEADING_MAX_WZ) wz = -HEADING_MAX_WZ;

            // Bu truot ngay o lenh: truc y chi dat K_SLIP_Y nen ra lenh du hon
            // de huong di THAT trung duong thang, khong bi cong roi moi sua.
            float vxb = v * uxb / K_SLIP_X;
            float vyb = v * uyb / K_SLIP_Y;
            inverse_kinematics(vxb, vyb, wz, &w_cmd_prev[0], &w_cmd_prev[1],
                               &w_cmd_prev[2], &w_cmd_prev[3]);
            robot_set_velocity(vxb, vyb, wz);
        }

        if ((now - last_log) >= GTP_LOG_EVERY_MS) {
            last_log = now;
            usart6_send_string("GTP,");
            usart6_send_int((int32_t)t);                    usart6_send_char(',');
            usart6_send_float(v_cmd, 3);                    usart6_send_char(',');
            send_mm(x);                                     usart6_send_char(',');
            send_mm(y);                                     usart6_send_char(',');
            usart6_send_float(th, 1);                       usart6_send_char(',');
            usart6_send_int(pose_link_valid());             usart6_send_char(',');
            send_mm(pose_link_get_x());                     usart6_send_char(',');
            send_mm(pose_link_get_y());                     usart6_send_char(',');
            usart6_send_float(pose_link_get_theta_deg(), 1); usart6_send_char(',');
            send_mm(cross_odo);                             usart6_send_char(',');
            send_mm(cross_cam);                             usart6_send_char(',');
            send_mm(dist);
            for (int i = 0; i < 4; i++) {
                usart6_send_char(',');
                usart6_send_int(cnt_log[i]);
                cnt_log[i] = 0;
            }
            usart6_send_string("\r\n");
        }

        if (state == ST_ENC_FAIL && t == t_stop) {
            usart6_send_string("GTP:ENC_FAIL banh ");
            usart6_send_string(WNAME[enc_fail_w]);
            usart6_send_string(" - lenh quay nhung encoder gan nhu khong dem, DUNG KHAN\r\n");
        } else if (state == ST_CAM_VUOT && t == t_stop) {
            usart6_send_string("GTP:CAM_VUOT_DICH - camera thay robot da qua dich, DUNG KHAN\r\n");
        } else if (state == ST_CAM_LECH && t == t_stop) {
            usart6_send_string("GTP:CAM_LECH_NGANG - camera thay robot lech khoi duong, DUNG KHAN\r\n");
        }
    }

    robot_set_velocity(0.0f, 0.0f, 0.0f);

    float xo = odometry_get_x(), yo = odometry_get_y();
    uint8_t cam_end = pose_link_valid();
    float xc = pose_link_get_x(), yc = pose_link_get_y(), thc = pose_link_get_theta_deg();

    float gdx = xc - GTP_GOAL_X_M, gdy = yc - GTP_GOAL_Y_M;
    float odx = xo - xc,           ody = yo - yc;

    usart6_send_string("GTP:DONE\r\n");

    while (1) {
        usart6_send_string("GTPSUM,ket_qua=");
        switch (state) {
            case ST_TOI_DICH: usart6_send_string("TOI_DICH"); break;
            case ST_HET_GIO:  usart6_send_string("HET_GIO");  break;
            case ST_ENC_FAIL: usart6_send_string("ENC_FAIL_");
                              usart6_send_string(WNAME[enc_fail_w]); break;
            case ST_CAM_VUOT: usart6_send_string("CAM_VUOT_DICH"); break;
            case ST_CAM_LECH: usart6_send_string("CAM_LECH_NGANG"); break;
            default:          usart6_send_string("?"); break;
        }
        usart6_send_string(",t_ms=");       usart6_send_int((int32_t)t_stop);
        usart6_send_string(",quang_duong_ly_tuong_mm="); send_mm(L);
        usart6_send_string("\r\n");

        usart6_send_string("GTPSUM,odo=(");  send_mm(xo); usart6_send_char(';'); send_mm(yo);
        usart6_send_string(") cam=(");       send_mm(xc); usart6_send_char(';'); send_mm(yc);
        usart6_send_string(") th_cam=");     usart6_send_float(thc, 1);
        usart6_send_string(" cam_ok=");      usart6_send_int(cam_end);
        usart6_send_string("\r\n");

        usart6_send_string("GTPSUM,SAI_SO_THAT(cam-dich): dx=");  send_mm(gdx);
        usart6_send_string(" dy=");  send_mm(gdy);
        usart6_send_string(" d=");   send_mm(sqrtf(gdx * gdx + gdy * gdy));
        usart6_send_string(" mm\r\n");

        usart6_send_string("GTPSUM,TROI_ODO(odo-cam): dx=");  send_mm(odx);
        usart6_send_string(" dy=");  send_mm(ody);
        usart6_send_string(" d=");   send_mm(sqrtf(odx * odx + ody * ody));
        usart6_send_string(" mm\r\n");

        usart6_send_string("GTPSUM,cross_max_cam=");  send_mm(cross_cam_max);
        usart6_send_string("mm cross_max_odo=");      send_mm(cross_odo_max);
        usart6_send_string("mm th_err_max=");         usart6_send_float(th_err_max, 1);
        usart6_send_string("do\r\n");

        // Tong xung ca hanh trinh: di cheo thi FR~RL va FL~RR, cap nhanh ~2.5x
        // cap cham. Banh nao lech han cap cua no la banh co van de.
        usart6_send_string("GTPSUM,xung_tong FL=");  usart6_send_int(cnt_tot[0]);
        usart6_send_string(" FR=");                  usart6_send_int(cnt_tot[1]);
        usart6_send_string(" RL=");                  usart6_send_int(cnt_tot[2]);
        usart6_send_string(" RR=");                  usart6_send_int(cnt_tot[3]);
        usart6_send_string("\r\n");

        HAL_Delay(3000);
    }
}
