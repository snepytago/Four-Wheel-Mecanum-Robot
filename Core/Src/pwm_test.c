#include "pwm_test.h"
#include "config.h"
#include "motor.h"
#include "encoder.h"
#include "usart6.h"
#include "pose_link.h"
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
    uint8_t  streak     = 0;
    uint32_t extra_left = 0;
    uint8_t  forward    = 1;
    float    v_at_thr   = 0.0f;

    const float dt_s = (float)PWM_TEST2_DWELL_MS / 1000.0f;

    all_motors_off();

    usart6_send_string("PWMTEST2:START pha2 - ROBOT TREN SAN, CAN KHOANG TRONG 2 DAU\r\n");
    usart6_send_string("PWMTEST2:cho pose tu camera...\r\n");

    while (!pose_link_valid()) {
        pose_link_poll();
    }

    usart6_send_string("PWMTEST2:POSE_OK\r\n");
    usart6_send_string("PWMT2,<duty>,<chieu>,<d_cam_mm>,<v_cam>,<v_enc>,<truot_%>\r\n");
    wait_polling(1500);

    for (uint32_t duty = PWM_TEST2_STEP; duty <= PWM_TEST2_MAX_DUTY;
         duty += PWM_TEST2_STEP) {

        pose_link_poll();
        float x0 = pose_link_get_x();
        float y0 = pose_link_get_y();
        encoder_get_deltas(dcnt);       // xoa moc encoder

        set_all_dir(forward);
        set_all_duty(duty);
        wait_polling(PWM_TEST2_DWELL_MS);
        set_all_duty(0);

        // Cho robot dung han va cho pose camera bat kip (camera 30Hz + tre
        // duong truyen). Quang duong do duoc vi the gom ca doan troi theo
        // quan tinh - chap nhan duoc cho muc dich tim nguong.
        wait_polling(PWM_TEST2_SETTLE_MS);

        encoder_get_deltas(dcnt);
        float x1 = pose_link_get_x();
        float y1 = pose_link_get_y();

        float dx = x1 - x0, dy = y1 - y0;
        float d_cam = sqrtf(dx * dx + dy * dy);

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
        usart6_send_int((int32_t)slip);
        usart6_send_string("\r\n");

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

        if (threshold == 0 && streak >= PWM_TEST2_CONFIRM_STEPS) {
            threshold = duty - (uint32_t)(PWM_TEST2_CONFIRM_STEPS - 1) * PWM_TEST2_STEP;
            v_at_thr  = v_cam;

            usart6_send_string("PWMTH2,");
            usart6_send_int((int32_t)threshold);        usart6_send_char(',');
            usart6_send_float(v_at_thr, 3);
            usart6_send_string("\r\n");

            extra_left = PWM_TEST2_EXTRA_STEPS;
            continue;
        }

        if (threshold != 0) {
            if (extra_left == 0) break;
            extra_left--;
        }
    }

    all_motors_off();
    usart6_send_string("PWMTEST2:DONE\r\n");

    while (1) {
        usart6_send_string("PWMSUM2,");
        usart6_send_int((int32_t)threshold);
        usart6_send_string("   (duty nguong tren san; 0 = khong dat duoc)\r\n");
        HAL_Delay(3000);
    }
}
