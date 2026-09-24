#ifndef PWM_TEST_H
#define PWM_TEST_H

#include <stdint.h>

// Bai do NGUONG PWM khoi dong cua 4 banh.
//
// Muc dich: tim duty nho nhat lam banh bat dau quay. Duoi nguong nay dong co
// chi u u ma khong nhuc nhich (ma sat tinh), nen vong go-to-point sau nay se
// bi "chet cung" cach dich vai cm neu khong biet con so nay ma dat V_MIN.
//
// PHA 1 (ham nay): KE BANH LEN KHOI MAT SAN, quay tung banh mot.
//   Do nguong khong tai -> dung de so sanh 4 dong co voi nhau. Mot banh can
//   duty cao hon han 3 banh kia = hop so ket / day dau loi / driver yeu.
//   KHONG dung con so nay cho V_MIN: thieu ma sat san va quan tinh robot.
//
// PHA 2 (lam sau, tren san that): ca 4 banh cung duty de robot tien thang,
//   tang dan den khi camera thay (x,y) nhuc nhich. So do moi la V_MIN.
//
// Cach chay: dat PWM_TEST_ENABLE = 1 trong config.h, nap, mo Serial Monitor
// cua esp32R. Ham chay 1 lan luc boot roi TREO lai in ket qua lap di lap lai
// (dong co da tat het) - khong roi vao vong dieu khien binh thuong, de vong
// giu huong bang gyro khong can thiep vao PWM giua luc dang do.
//
// Cach chot nguong: banh phai quay duoc PWM_TEST_CONFIRM_STEPS nac LIEN TIEP
// moi cong nhan, va nguong lay la nac DAU cua chuoi do. Mot nac don le bi loai
// vi dong co hay giat mot cai vuot ma sat tinh roi ket lai ngay - neu nhan cu
// giat do lam nguong thi con so ghi duoc se thap hon thuc te.
// Sau khi chot, chay them PWM_TEST_EXTRA_STEPS nac nua de lay doan dau duong
// dac tinh duty -> toc do; do doc doan nay la K_FF thuc cua rieng banh do.
//
// Dinh dang log:
//   PWMT,<banh>,<duty>,<so_xung>,<rad/s>  - moi nac duty (ca khi chua quay)
//   PWMTH,<banh>,<duty_nguong>            - nguong da xac nhan
//   PWMSUM,<FL>,<FR>,<RL>,<RR>            - tong ket, in lap lai moi 3 giay

void pwm_test_run(void);

// ---------------------------------------------------------------------------
// PHA 2: do tren SAN THAT, ca 4 banh cung duty de robot tien thang.
// ---------------------------------------------------------------------------
// Khac pha 1 o cho: trong tai quyet dinh "da di chuyen chua" la CAMERA, khong
// phai encoder. Ly do: 4 banh co nguong lech nhau, nen o duty thap co banh da
// quay trong khi banh khac con ket - may banh ket do khoa cung va ghi robot
// lai, banh quay duoc chi truot tai cho. Encoder bao co chuyen dong rat ro
// trong khi robot dung nguyen. Camera do thang dai luong can do.
//
// Encoder van duoc ghi song song: hieu so giua quang duong banh lan duoc va
// quang duong robot that chinh la DO TRUOT cua banh mecanum tren mat san do -
// so lieu khong phep do nao khac cho duoc, va la can cu dat nhieu qua trinh
// neu sau nay lam Kalman.
//
// CAN CHUAN BI: gateway + camera dang chay (co pose hop le), robot dat tren
// san, phia truoc va phia sau co khoang trong. Robot chay tien/lui XEN KE moi
// nac de khong troi xa khoi vung camera quet.
//
// LUU Y ve do chinh xac: quang duong do gom ca doan robot troi theo quan tinh
// sau khi cat PWM, nen v_cam hoi cao hon thuc te mot chut. Du chinh xac de tim
// NGUONG, nhung neu muon do K_FF that chuan thi can bai do rieng chay on dinh
// o toc do co dinh trong vai giay.
//
// Dinh dang log:
//   PWMT2,<duty>,<F|B>,<d_cam_mm>,<v_cam>,<v_enc>,<truot_%>,
//         <dtheta_deg>,<x_mm>,<y_mm>,<dFL>,<dFR>,<dRL>,<dRR>
//   PWMTH2,<duty_nguong>,<v_cam_tai_nguong>   - nguong da xac nhan = V_MIN
//   PWMSUM2,<duty_nguong>                     - in lap lai moi 3 giay
//
// Ba nhom cot chan doan them (de tra loi vi sao ty le v_cam/v_enc co nac tut):
//   dtheta  - robot xoay bao nhieu do trong nac do. Xoay nhieu -> quy dao la
//             cung cong, ma d_cam chi do khoang cach THANG dau-cuoi, nen ty le
//             tut xuong du banh khong he truot. Bai do nay ghi thang PWM nen
//             vong giu huong bang gyro KHONG hoat dong, robot xoay tu do.
//   x,y     - robot dang o dau. Neu cac nac xau tap trung o mot vung toa do
//             thi nguyen nhan la mat san cho do hoac mep vung camera quet.
//   dFL..RR - delta encoder tung banh. Banh nao gan 0 trong khi 3 banh kia
//             quay deu thi chinh banh do dang ket va lam robot xoay.

void pwm_test2_run(void);

// ---------------------------------------------------------------------------
// Kiem tra quy uoc DAU cua theta camera - khong chay dong co, chi doc.
// ---------------------------------------------------------------------------
// In song song theta camera va yaw gyro, KHONG cho ben nao ghi de ben nao,
// roi nguoi dung xoay robot bang tay. Gyro lam chuan doi chieu vi quy uoc cua
// no da duoc xac lap tu lau. Ham tu in ket luan khi da xoay qua 30 do.
//
// Dinh dang log:
//   THCHK,<valid>,<th_cam>,<yaw_gyro>,<d_cam>,<d_gyro>
//   THCHK:KET LUAN = CUNG CHIEU / NGUOC CHIEU
//
// Goi SAU khi da hieu chuan IMU trong main.c. Ham khong bao gio return.
void pose_theta_check_run(uint8_t imu_ready);

// ---------------------------------------------------------------------------
// Bai chay thang - kiem chung K_FF, vong giu huong, va do tre camera
// ---------------------------------------------------------------------------
// Di qua dung duong van hanh that (odometry -> ghi de pose -> vong giu huong
// -> robot_set_velocity), khong ghi thang PWM. Ramp len toc do dat, giu vai
// giay, roi CAT LENH dot ngot va theo doi tiep de do do tre camera.
//
// Dinh dang log:
//   ST,<t_ms>,<v_cmd>,<x_mm>,<y_mm>,<th_cam>,<yaw>,<th_err>,<d_enc_mm>,<d_cam_mm>
//   STSUM,... - tong ket, in lap lai moi 3 giay
//
// Doc ket qua:
//   ty_le          = v_thuc / v_dat. Lech nhieu -> nhan K_FF voi chinh ty le do.
//   th_err_max     = vong giu huong co lam viec khi dang chay khong.
//   t_banh_dung vs t_cam_thay_dung: hieu hai moc nay la do tre duong truyen pose.
//
// Goi SAU khi hieu chuan IMU. Ham khong bao gio return.
void straight_test_run(uint8_t imu_ready);

#endif
