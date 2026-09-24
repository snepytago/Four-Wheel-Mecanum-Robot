#include "pwm_test.h"
#include "config.h"
#include "motor.h"
#include "encoder.h"
#include "usart6.h"
#include "pose_link.h"
#include "mpu6050.h"
#include "odometry.h"
#include "robot_control.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <math.h>

static const char *WHEEL_NAME[4] = { "FL", "FR", "RL", "RR" };

// Chieu tien cua tung banh - dung dung bang da xac nhan tren phan cung that.
// Ghi thang BSRR thay vi goi motor_set_XX() vi cac ham do tinh duty tu toc do
// qua K_FF, con o day can dat duty TRUC TIEP tung nac mot.
static void set_dir_forward(uint8_t w)
{
    switch (w) {
        case MOTOR_FL: GPIOB->BSRR = (1u << 0);
                       GPIOA->BSRR = (1u << (4 + 16));               break;
        case MOTOR_FR: GPIOC->BSRR = (1u << 0) | (1u << (1 + 16));   break;
        case MOTOR_RL: GPIOC->BSRR = (1u << (4 + 16)) | (1u << 5);   break;
        case MOTOR_RR: GPIOC->BSRR = (1u << 2) | (1u << (3 + 16));   break;
        default: break;
    }
}

static void set_duty(uint8_t w, uint32_t duty)
{
    switch (w) {
        case MOTOR_FL: TIM1->CCR1 = duty; break;
        case MOTOR_FR: TIM1->CCR2 = duty; break;
        case MOTOR_RL: TIM1->CCR3 = duty; break;
        case MOTOR_RR: TIM1->CCR4 = duty; break;
        default: break;
    }
}

static void all_motors_off(void)
{
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM1->CCR4 = 0;
}

static int32_t abs_i32(int32_t v) { return (v < 0) ? -v : v; }

void pwm_test_run(void)
{
    uint32_t threshold[4] = { 0, 0, 0, 0 };
    int32_t  dcnt[4];

    const float dt_s = (float)PWM_TEST_DWELL_MS / 1000.0f;

    all_motors_off();

    usart6_send_string("PWMTEST:START pha1 - KE BANH LEN KHOI SAN\r\n");
    usart6_send_string("PWMT,<banh>,<duty>,<so_xung>,<rad/s>\r\n");

    HAL_Delay(1000);   // de nguoi dung kip mo Serial Monitor va ke banh

    for (uint8_t w = 0; w < 4; w++) {
        set_dir_forward(w);
        encoder_get_deltas(dcnt);   // xoa moc, bo qua gia tri cu

        uint8_t  run_streak = 0;    // so nac LIEN TIEP banh quay duoc
        uint32_t extra_left = 0;    // so nac chay them sau khi da chot nguong

        usart6_send_string("PWMTEST:wheel=");
        usart6_send_string(WHEEL_NAME[w]);
        usart6_send_string("\r\n");

        for (uint32_t duty = PWM_TEST_STEP; duty <= PWM_TEST_MAX_DUTY;
             duty += PWM_TEST_STEP) {

            set_duty(w, duty);
            HAL_Delay(PWM_TEST_DWELL_MS);

            encoder_get_deltas(dcnt);
            int32_t moved = abs_i32(dcnt[w]);

            // Quy ra toc do goc de nhin duoc duong dac tinh, khong chi diem nguong:
            // do doc cua doan nay chinh la K_FF thuc cua rieng banh do.
            float omega = ((float)moved / COUNTS_PER_REV) * TWO_PI / dt_s;

            usart6_send_string("PWMT,");
            usart6_send_string(WHEEL_NAME[w]);   usart6_send_char(',');
            usart6_send_int((int32_t)duty);      usart6_send_char(',');
            usart6_send_int(moved);              usart6_send_char(',');
            usart6_send_float(omega, 2);
            usart6_send_string("\r\n");

            if (moved >= PWM_TEST_MIN_COUNTS) {
                if (run_streak < 255) run_streak++;
            } else {
                run_streak = 0;      // mot cu giat le -> xoa chuoi, khong tinh
            }

            // Chi chot nguong khi banh quay LIEN TIEP du nhieu nac. Mot nac
            // don le co the chi la dong co giat mot cai roi ket lai.
            if (threshold[w] == 0 && run_streak >= PWM_TEST_CONFIRM_STEPS) {
                threshold[w] = duty - (uint32_t)(PWM_TEST_CONFIRM_STEPS - 1) * PWM_TEST_STEP;

                usart6_send_string("PWMTH,");
                usart6_send_string(WHEEL_NAME[w]);       usart6_send_char(',');
                usart6_send_int((int32_t)threshold[w]);
                usart6_send_string("\r\n");

                extra_left = PWM_TEST_EXTRA_STEPS;
                continue;
            }

            // Da chot nguong -> chay them vai nac lay duong dac tinh roi dung,
            // khong quet het dai de banh khong quay long tren khong.
            if (threshold[w] != 0) {
                if (extra_left == 0) break;
                extra_left--;
            }
        }

        set_duty(w, 0);

        if (threshold[w] == 0) {
            usart6_send_string("PWMTH,");
            usart6_send_string(WHEEL_NAME[w]);
            usart6_send_string(",0 (khong quay lien tuc duoc)\r\n");
        }

        HAL_Delay(800);   // banh dung han truoc khi sang banh tiep
    }

    all_motors_off();
    usart6_send_string("PWMTEST:DONE\r\n");

    // Treo lai, in tong ket lap lai - dong co da tat het nen an toan.
    // Doi PWM_TEST_ENABLE ve 0 va nap lai de robot chay binh thuong.
    while (1) {
        usart6_send_string("PWMSUM,");
        for (uint8_t w = 0; w < 4; w++) {
            usart6_send_int((int32_t)threshold[w]);
            if (w < 3) usart6_send_char(',');
        }
        usart6_send_string("   (FL,FR,RL,RR - 0 = khong quay lien tuc duoc)\r\n");
        HAL_Delay(3000);
    }
}

// ===========================================================================
// PHA 2 - do tren san that, ca 4 banh cung duty, camera lam trong tai
// ===========================================================================

static void set_all_dir(uint8_t forward)
{
    if (forward) {
        GPIOB->BSRR = (1u << 0);        GPIOA->BSRR = (1u << (4 + 16));  // FL
        GPIOC->BSRR = (1u << 0) | (1u << (1 + 16));                      // FR
        GPIOC->BSRR = (1u << (4 + 16)) | (1u << 5);                      // RL
        GPIOC->BSRR = (1u << 2) | (1u << (3 + 16));                      // RR
    } else {
        GPIOB->BSRR = (1u << (0 + 16)); GPIOA->BSRR = (1u << 4);
        GPIOC->BSRR = (1u << (0 + 16)) | (1u << 1);
        GPIOC->BSRR = (1u << 4) | (1u << (5 + 16));
        GPIOC->BSRR = (1u << (2 + 16)) | (1u << 3);
    }
}

static void set_all_duty(uint32_t d)
{
    TIM1->CCR1 = d; TIM1->CCR2 = d; TIM1->CCR3 = d; TIM1->CCR4 = d;
}

// Bu vung chet: moi banh duoc cong nguong khoi dong rieng cua no, roi cung
// tang them mot luong delta nhu nhau. Muc tieu la ca 4 banh bat dau quay tai
// cung mot diem thay vi banh nay quay truoc banh kia vai chuc duty.
static void set_duty_offset(uint32_t delta)
{
    const uint32_t th[4] = { PWM_TH_FL, PWM_TH_FR, PWM_TH_RL, PWM_TH_RR };
    uint32_t d[4];

    for (uint8_t i = 0; i < 4; i++) {
        d[i] = th[i] + delta;
        if (d[i] > PWM_MAX - 1) d[i] = PWM_MAX - 1;
    }

    TIM1->CCR1 = d[MOTOR_FL];
    TIM1->CCR2 = d[MOTOR_FR];
    TIM1->CCR3 = d[MOTOR_RL];
    TIM1->CCR4 = d[MOTOR_RR];
}

// Cho ms mili giay, VAN doc pose lien tuc. Khong dung HAL_Delay o day: neu
// khong goi pose_link_poll() thi hang doi RX day va ta mat khung hinh moi.
static void wait_polling(uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < ms) {
        pose_link_poll();
    }
}

void pwm_test2_run(void)
{
    int32_t  dcnt[4];
    uint32_t threshold  = 0;
    uint8_t  thr_found  = 0;   // co rieng: o che do bu nguong, delta hop le
                               // bat dau tu 0 nen khong dung threshold==0 lam co
    uint8_t  streak     = 0;
    uint32_t extra_left = 0;
    uint8_t  forward    = 1;
    float    v_at_thr   = 0.0f;

    // Luu v_cam vai nac gan nhat: khi chot nguong can lay toc do tai nac DAU
    // cua chuoi (chinh la nguong), khong phai tai nac xac nhan cuoi chuoi.
    float    v_hist[4]  = { 0.0f, 0.0f, 0.0f, 0.0f };

    const float dt_s = (float)PWM_TEST2_DWELL_MS / 1000.0f;

    all_motors_off();

    usart6_send_string("PWMTEST2:START pha2 - ROBOT TREN SAN, CAN KHOANG TRONG 2 DAU\r\n");
    usart6_send_string("PWMTEST2:cho pose tu camera...\r\n");

    while (!pose_link_valid()) {
        pose_link_poll();
    }

    usart6_send_string("PWMTEST2:POSE_OK\r\n");
#if PWM_TEST2_USE_OFFSET
    usart6_send_string("PWMTEST2:che do BU NGUONG - cot 1 la DELTA, "
                       "duty that = nguong rieng + delta\r\n");
    usart6_send_string("PWMTEST2:nguong dung (FL,FR,RL,RR)=");
    usart6_send_int(PWM_TH_FL);  usart6_send_char(',');
    usart6_send_int(PWM_TH_FR);  usart6_send_char(',');
    usart6_send_int(PWM_TH_RL);  usart6_send_char(',');
    usart6_send_int(PWM_TH_RR);
    usart6_send_string("\r\n");
    usart6_send_string("PWMT2,delta,chieu,d_cam_mm,v_cam,v_enc,truot%,"
                       "dtheta,x_mm,y_mm,dFL,dFR,dRL,dRR\r\n");
#else
    usart6_send_string("PWMT2,duty,chieu,d_cam_mm,v_cam,v_enc,truot%,"
                       "dtheta,x_mm,y_mm,dFL,dFR,dRL,dRR\r\n");
#endif
    wait_polling(1500);

    for (uint32_t duty = PWM_TEST2_START; duty <= PWM_TEST2_MAX_DUTY;
         duty += PWM_TEST2_STEP) {

        pose_link_poll();
        float x0  = pose_link_get_x();
        float y0  = pose_link_get_y();
        float th0 = pose_link_get_theta_deg();
        encoder_get_deltas(dcnt);       // xoa moc encoder

        set_all_dir(forward);
#if PWM_TEST2_USE_OFFSET
        set_duty_offset(duty);      // 'duty' o day la DELTA tren nguong tung banh
#else
        set_all_duty(duty);
#endif
        wait_polling(PWM_TEST2_DWELL_MS);
        set_all_duty(0);

        // Cho robot dung han va cho pose camera bat kip (camera 30Hz + tre
        // duong truyen). Quang duong do duoc vi the gom ca doan troi theo
        // quan tinh - chap nhan duoc cho muc dich tim nguong.
        wait_polling(PWM_TEST2_SETTLE_MS);

        encoder_get_deltas(dcnt);
        float x1  = pose_link_get_x();
        float y1  = pose_link_get_y();
        float th1 = pose_link_get_theta_deg();

        float dx = x1 - x0, dy = y1 - y0;
        float d_cam = sqrtf(dx * dx + dy * dy);

        // Goc xoay trong nac nay. Neu robot vua tien vua xoay thi quy dao la
        // cung cong, trong khi d_cam do KHOANG CACH THANG giua diem dau va
        // diem cuoi - cung cang cong thi d_cam cang ngan hon quang duong that,
        // va ty le v_cam/v_enc tut xuong ma khong he co truot banh.
        float dth = th1 - th0;
        while (dth >  180.0f) dth -= 360.0f;
        while (dth < -180.0f) dth += 360.0f;

        float avg = (float)(dcnt[0] + dcnt[1] + dcnt[2] + dcnt[3]) / 4.0f;
        if (avg < 0.0f) avg = -avg;
        float d_enc = WHEEL_R * TWO_PI * (avg / COUNTS_PER_REV);

        float v_cam = d_cam / dt_s;
        float v_enc = d_enc / dt_s;
        float slip  = (d_enc > 0.002f) ? ((d_enc - d_cam) / d_enc * 100.0f) : 0.0f;

        usart6_send_string("PWMT2,");
        usart6_send_int((int32_t)duty);                 usart6_send_char(',');
        usart6_send_char(forward ? 'F' : 'B');          usart6_send_char(',');
        usart6_send_int((int32_t)(d_cam * 1000.0f));    usart6_send_char(',');
        usart6_send_float(v_cam, 3);                    usart6_send_char(',');
        usart6_send_float(v_enc, 3);                    usart6_send_char(',');
        usart6_send_int((int32_t)slip);                 usart6_send_char(',');
        usart6_send_float(dth, 1);                      usart6_send_char(',');
        usart6_send_int((int32_t)(x1 * 1000.0f));       usart6_send_char(',');
        usart6_send_int((int32_t)(y1 * 1000.0f));       usart6_send_char(',');
        usart6_send_int(dcnt[MOTOR_FL]);                usart6_send_char(',');
        usart6_send_int(dcnt[MOTOR_FR]);                usart6_send_char(',');
        usart6_send_int(dcnt[MOTOR_RL]);                usart6_send_char(',');
        usart6_send_int(dcnt[MOTOR_RR]);
        usart6_send_string("\r\n");

        v_hist[3] = v_hist[2];
        v_hist[2] = v_hist[1];
        v_hist[1] = v_hist[0];
        v_hist[0] = v_cam;

        if (!pose_link_valid()) {
            all_motors_off();
            usart6_send_string("PWMTEST2:MAT_POSE - dung bai do\r\n");
            break;
        }

        forward = forward ? 0 : 1;   // doi chieu de robot khong troi xa dan

        if (d_cam >= PWM_TEST2_MIN_MOVE_M) {
            if (streak < 255) streak++;
        } else {
            streak = 0;
        }

        if (!thr_found && streak >= PWM_TEST2_CONFIRM_STEPS) {
            uint32_t back = (uint32_t)(PWM_TEST2_CONFIRM_STEPS - 1) * PWM_TEST2_STEP;
            threshold = (duty >= back) ? (duty - back) : 0u;   // tranh tran uint32
            thr_found = 1;

            // Toc do tai dung nac nguong, khong phai tai nac xac nhan.
            uint8_t hist_i = PWM_TEST2_CONFIRM_STEPS - 1;
            if (hist_i > 3) hist_i = 3;
            v_at_thr = v_hist[hist_i];

            usart6_send_string("PWMTH2,");
            usart6_send_int((int32_t)threshold);        usart6_send_char(',');
            usart6_send_float(v_at_thr, 3);
            usart6_send_string("\r\n");

            extra_left = PWM_TEST2_EXTRA_STEPS;
            continue;
        }

        if (thr_found) {
            if (extra_left == 0) break;
            extra_left--;
        }
    }

    all_motors_off();
    usart6_send_string("PWMTEST2:DONE\r\n");

    while (1) {
        if (thr_found) {
            usart6_send_string("PWMSUM2,");
            usart6_send_int((int32_t)threshold);    usart6_send_char(',');
            usart6_send_float(v_at_thr, 3);
#if PWM_TEST2_USE_OFFSET
            usart6_send_string("   (DELTA nguong tren san, V_MIN m/s)\r\n");
#else
            usart6_send_string("   (duty nguong tren san, V_MIN m/s)\r\n");
#endif
        } else {
            usart6_send_string("PWMSUM2,KHONG_DAT_NGUONG\r\n");
        }
        HAL_Delay(3000);
    }
}

// ===========================================================================
// Kiem tra QUY UOC DAU cua theta tu camera - khong chay dong co
// ===========================================================================
// Van de: camera nhin tu tren xuong, truc y cua anh huong XUONG, nen goc tinh
// bang atan2 tren toa do anh se NGUOC dau so voi chieu duong cua gyro Z. Neu
// sai dau, vong giu huong tro thanh phan hoi DUONG: moi lan robot xoay de sua
// loi thi so bao ve lai cho thay loi to hon, nen no xoay manh them.
//
// Cach kiem: doc song song theta camera va yaw gyro, KHONG ghi de lan nhau,
// roi xoay robot bang tay. Gyro da duoc xac lap dung quy uoc tu lau (vong giu
// huong dat +-1 do), nen no dong vai tro chuan doi chieu.
//   hai so cung dau  -> quy uoc khop, dung nguyen
//   hai so nguoc dau -> phai DAO DAU theta o phia Python, khong sua firmware
//     (odometry.c va vong giu huong deu dang bam theo quy uoc gyro)
void pose_theta_check_run(uint8_t imu_ready)
{
    all_motors_off();

    usart6_send_string("THCHK:START - XOAY ROBOT BANG TAY ~90 DO\r\n");
    if (!imu_ready) {
        usart6_send_string("THCHK:CANH BAO - khong co IMU, chi co theta camera\r\n");
    }
    usart6_send_string("THCHK:cho pose tu camera...\r\n");

    while (!pose_link_valid()) {
        pose_link_poll();
    }

    float th_cam_0 = pose_link_get_theta_deg();
    float yaw_0    = imu_ready ? mpu6050_get_yaw_deg() : 0.0f;

    usart6_send_string("THCHK,valid,th_cam,yaw_gyro,d_cam,d_gyro\r\n");

    uint32_t last_tick  = HAL_GetTick();
    uint32_t last_print = last_tick;
    uint8_t  verdict    = 0;

    while (1) {
        pose_link_poll();

        uint32_t now = HAL_GetTick();

        // Gyro phai duoc tich phan deu dan moi ra goc dung
        if ((now - last_tick) >= 20u) {
            float dt = (float)(now - last_tick) / 1000.0f;
            last_tick = now;
            if (imu_ready) mpu6050_update(dt);
        }

        if ((now - last_print) < 200u) continue;
        last_print = now;

        float th_cam = pose_link_get_theta_deg();
        float yaw    = imu_ready ? mpu6050_get_yaw_deg() : 0.0f;

        float d_cam = th_cam - th_cam_0;
        while (d_cam >  180.0f) d_cam -= 360.0f;
        while (d_cam < -180.0f) d_cam += 360.0f;

        float d_gyro = yaw - yaw_0;

        usart6_send_string("THCHK,");
        usart6_send_int(pose_link_valid());      usart6_send_char(',');
        usart6_send_float(th_cam, 1);            usart6_send_char(',');
        usart6_send_float(yaw, 1);               usart6_send_char(',');
        usart6_send_float(d_cam, 1);             usart6_send_char(',');
        usart6_send_float(d_gyro, 1);
        usart6_send_string("\r\n");

        // Ket luan tu dong khi da xoay du xa de vuot qua nhieu do
        if (imu_ready && !verdict && fabsf(d_gyro) > 30.0f && fabsf(d_cam) > 30.0f) {
            verdict = 1;
            if (d_cam * d_gyro > 0.0f) {
                usart6_send_string("THCHK:KET LUAN = CUNG CHIEU - quy uoc DUNG, giu nguyen\r\n");
            } else {
                usart6_send_string("THCHK:KET LUAN = NGUOC CHIEU - phai DAO DAU theta o Python\r\n");
            }
        }

        // Xoay tra ve gan cho cu thi cho phep thu lai lan nua
        if (verdict && fabsf(d_gyro) < 10.0f) verdict = 0;
    }
}

// ===========================================================================
// Bai chay thang - kiem chung K_FF moi, vong giu huong, va do tre camera
// ===========================================================================
// Khac cac bai truoc: KHONG ghi thang PWM ma di qua dung duong van hanh that
// (odometry -> ghi de pose tu camera -> vong giu huong -> robot_set_velocity).
// Hong o dau thi biet ngay o do vi chi co feedforward va vong giu huong hoat
// dong, chua co vong vi tri nao can thiep.
//
// Ba cau hoi bai nay tra loi:
//   1. K_FF = 23.3 co dung khong? -> so quang duong do duoc voi quang duong
//      le ra phai di (toc do lenh x thoi gian).
//   2. Vong giu huong co lam viec khi robot dang chay khong? -> nhin th_err
//      (luc dung yen no bat luc vi wz mot minh khong vuot noi MOTOR_OMEGA_MIN,
//      nhung khi dang chay thi vx da day cac banh vuot nguong roi).
//   3. Camera tre bao nhieu? -> cat lenh dot ngot roi do: banh ngung quay
//      (d_enc dung tang) truoc, camera thay robot dung (d_cam dung tang) sau.
//      Chenh lech thoi gian giua hai moc do chinh la do tre duong truyen pose.
//
// Huong di lay theo DAU XE luc bat dau, khong phai truc X the gioi, de robot
// tien thang chu khong truot ngang - nhu vay moi so sanh duoc voi encoder.
// Goc can giu cung lay luc bat dau, nen robot khong bi giat ve 0 do.
void straight_test_run(uint8_t imu_ready)
{
    int32_t dcnt[4];

    all_motors_off();
    usart6_send_string("ST:START - can khoang trong >1.5m phia truoc dau xe\r\n");
    usart6_send_string("ST:cho pose tu camera...\r\n");

    while (!pose_link_valid()) {
        pose_link_poll();
    }

    const float x0  = pose_link_get_x();
    const float y0  = pose_link_get_y();
    const float th0 = pose_link_get_theta_deg();

    // Huong tien = huong dau xe hien tai, chieu ra he the gioi
    const float dir_c = cosf(th0 * 0.017453293f);
    const float dir_s = sinf(th0 * 0.017453293f);

    if (imu_ready) mpu6050_set_yaw_deg(th0);   // dong bo gyro voi camera truoc khi chay

    encoder_get_deltas(dcnt);                  // xoa moc encoder
    odometry_set_pose(x0, y0);

    float d_enc = 0.0f;
    float th_err_max = 0.0f;

    float d_cam_hold = 0.0f, d_enc_hold = 0.0f;   // moc luc het ramp
    float d_cam_cut  = 0.0f, d_enc_cut  = 0.0f;   // moc luc cat lenh
    uint32_t t_cut = 0;
    uint32_t t_enc_stop = 0, t_cam_stop = 0;
    float d_cam_prev = 0.0f, d_enc_prev = 0.0f;
    uint8_t hold_done = 0, cut_done = 0;

    usart6_send_string("ST,t_ms,v_cmd,x_mm,y_mm,th_cam,yaw,th_err,d_enc_mm,d_cam_mm\r\n");

    const uint32_t t_start = HAL_GetTick();
    uint32_t last_step = t_start, last_dt = t_start, last_log = t_start;

    while (1) {
        uint32_t now = HAL_GetTick();
        uint32_t t   = now - t_start;

        if (t >= (ST_RAMP_MS + ST_HOLD_MS + ST_COAST_MS)) break;

        pose_link_poll();

        if ((now - last_step) < CONTROL_DT_MS) continue;
        last_step = now;

        float dt = (float)(now - last_dt) / 1000.0f;
        last_dt = now;

        encoder_get_deltas(dcnt);
        float avg = (float)(dcnt[0] + dcnt[1] + dcnt[2] + dcnt[3]) / 4.0f;
        d_enc += WHEEL_R * TWO_PI * (avg / COUNTS_PER_REV);

        if (imu_ready) mpu6050_update(dt);
        odometry_update(dcnt, dt);

        if (pose_link_take_fresh()) {
            odometry_set_pose(pose_link_get_x(), pose_link_get_y());
            float yaw_now = mpu6050_get_yaw_deg();
            float dth = pose_link_get_theta_deg() - yaw_now;
            while (dth >  180.0f) dth -= 360.0f;
            while (dth < -180.0f) dth += 360.0f;
            mpu6050_set_yaw_deg(yaw_now + POSE_YAW_ALPHA * dth);
        }

        if (!pose_link_valid()) {
            all_motors_off();
            usart6_send_string("ST:MAT_POSE - dung khan\r\n");
            break;
        }

        float dx = odometry_get_x() - x0;
        float dy = odometry_get_y() - y0;
        float d_cam = sqrtf(dx * dx + dy * dy);

        // --- lenh toc do: ramp -> giu -> cat ---
        float v;
        if (t < ST_RAMP_MS) {
            v = ST_TARGET_V * (float)t / (float)ST_RAMP_MS;
        } else if (t < (ST_RAMP_MS + ST_HOLD_MS)) {
            v = ST_TARGET_V;
            if (!hold_done) { hold_done = 1; d_cam_hold = d_cam; d_enc_hold = d_enc; }
        } else {
            v = 0.0f;
            if (!cut_done) {
                cut_done = 1; t_cut = t;
                d_cam_cut = d_cam; d_enc_cut = d_enc;
                d_cam_prev = d_cam; d_enc_prev = d_enc;
            }
        }

        // Sau khi cat lenh: bat thoi diem banh ngung quay va thoi diem camera
        // thay robot dung. Chenh lech = do tre duong truyen pose.
        if (cut_done) {
            if (t_enc_stop == 0 && fabsf(d_enc - d_enc_prev) < 0.001f) t_enc_stop = t;
            if (t_cam_stop == 0 && fabsf(d_cam - d_cam_prev) < 0.002f) t_cam_stop = t;
            d_enc_prev = d_enc;
            d_cam_prev = d_cam;
        }

        float vxw = v * dir_c;
        float vyw = v * dir_s;

        float th  = odometry_get_theta_deg();
        float thr = th * 0.017453293f;
        float vxb =  vxw * cosf(thr) + vyw * sinf(thr);
        float vyb = -vxw * sinf(thr) + vyw * cosf(thr);

        float err = th0 - th;
        while (err >  180.0f) err -= 360.0f;
        while (err < -180.0f) err += 360.0f;
        if (fabsf(err) > th_err_max) th_err_max = fabsf(err);

        float wz = HEADING_KP * err;
        if (wz >  HEADING_MAX_WZ) wz =  HEADING_MAX_WZ;
        if (wz < -HEADING_MAX_WZ) wz = -HEADING_MAX_WZ;

        robot_set_velocity(vxb, vyb, wz);

        if ((now - last_log) >= ST_LOG_EVERY_MS) {
            last_log = now;
            usart6_send_string("ST,");
            usart6_send_int((int32_t)t);                             usart6_send_char(',');
            usart6_send_float(v, 3);                                 usart6_send_char(',');
            usart6_send_int((int32_t)(odometry_get_x() * 1000.0f));  usart6_send_char(',');
            usart6_send_int((int32_t)(odometry_get_y() * 1000.0f));  usart6_send_char(',');
            usart6_send_float(pose_link_get_theta_deg(), 1);         usart6_send_char(',');
            usart6_send_float(mpu6050_get_yaw_deg(), 1);             usart6_send_char(',');
            usart6_send_float(err, 1);                               usart6_send_char(',');
            usart6_send_int((int32_t)(d_enc * 1000.0f));             usart6_send_char(',');
            usart6_send_int((int32_t)(d_cam * 1000.0f));
            usart6_send_string("\r\n");
        }
    }

    all_motors_off();

    float dxf = odometry_get_x() - x0;
    float dyf = odometry_get_y() - y0;
    float d_cam_end = sqrtf(dxf * dxf + dyf * dyf);

    float v_real     = (d_cam_cut - d_cam_hold) / ((float)ST_HOLD_MS / 1000.0f);
    float v_enc_real = (d_enc_cut - d_enc_hold) / ((float)ST_HOLD_MS / 1000.0f);

    usart6_send_string("ST:DONE\r\n");

    while (1) {
        // Quang duong le ra phai di trong doan giu toc do, so voi thuc te:
        // ty le nay chinh la he so can nhan vao K_FF neu muon chinh cho dung.
        usart6_send_string("STSUM,v_dat=");   usart6_send_float(ST_TARGET_V, 3);
        usart6_send_string(",v_cam=");        usart6_send_float(v_real, 3);
        usart6_send_string(",v_enc=");        usart6_send_float(v_enc_real, 3);
        usart6_send_string(",ty_le=");        usart6_send_float(v_real / ST_TARGET_V, 3);
        usart6_send_string("\r\n");

        usart6_send_string("STSUM,d_cam=");   usart6_send_int((int32_t)(d_cam_end * 1000.0f));
        usart6_send_string("mm,d_enc=");      usart6_send_int((int32_t)(d_enc * 1000.0f));
        usart6_send_string("mm,th_err_max="); usart6_send_float(th_err_max, 1);
        usart6_send_string("do\r\n");

        usart6_send_string("STSUM,sau_khi_cat: troi_cam=");
        usart6_send_int((int32_t)((d_cam_end - d_cam_cut) * 1000.0f));
        usart6_send_string("mm,troi_enc=");
        usart6_send_int((int32_t)((d_enc - d_enc_cut) * 1000.0f));
        usart6_send_string("mm\r\n");

        usart6_send_string("STSUM,t_banh_dung=");
        usart6_send_int((int32_t)(t_enc_stop ? (t_enc_stop - t_cut) : -1));
        usart6_send_string("ms,t_cam_thay_dung=");
        usart6_send_int((int32_t)(t_cam_stop ? (t_cam_stop - t_cut) : -1));
        usart6_send_string("ms  -> hieu = do tre camera\r\n");

        HAL_Delay(3000);
    }
}
