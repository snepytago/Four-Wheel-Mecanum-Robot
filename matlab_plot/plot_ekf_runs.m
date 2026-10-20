%% plot_ekf_runs.m
% Ve cac lan chay thang 3 m bam duong bang EKF (ekf_run_test) va so sanh voi
% 2 bo test truoc: thuan encoder va vong IMU giu huong (KP=3).
%
% Can cung thu muc: fwmr_parse_log.m va cac file log.
% Moi toa do trong HE XUAT PHAT: goc = diem xuat phat, truc x = huong mui xe luc
% xuat phat (duong co so la doan thang tu (0;0) toi (3000;0)).
%
% Nguon toa do:
%   EKF   : vi tri CAMERA (ArUco) - vi tri that cua robot
%   enc/IMU: odometry cua chinh robot (luc do chua co camera) -> chi de tham khao
%
% Lan chay "loi camera" (tu nhan ra): khong co khung camera luc dung, hoac mat
% camera > 1 s, hoac encoder cho thay xe chay xa hon 1.3 x 3 m.

%clear; close all; clc;

%% ===================== CAU HINH =====================
LOG_EKF   = 'log_ekf_18102026.txt';
LOG_ENC   = 'log_enc_run_06102026.txt';
LOG_IMU   = 'log_imu_kp3_07102026.txt';
TARGET_MM = 3000;
TOL_MM    = 50;          % hanh lang quanh duong co so
BAD_RUNS  = [];          % [] = tu nhan ra ; vd [6 9] = chi dinh tay
SAVE_PNG  = true;
PT_SIZE   = 4;

C_ENC = [0.922 0.408 0.204];   % cam
C_IMU = [0.165 0.471 0.839];   % xanh duong
C_EKF = [0.000 0.600 0.330];   % xanh la
C_BAD = [0.85 0.15 0.15];
C_TOL = [0.106 0.686 0.478];
INK   = [0 0 0];
INK2  = [0.32 0.32 0.31];
GRIDC = [0.88 0.88 0.87];
LEG_INK = [1 1 1];  LEG_BG = [0.2 0.2 0.2];
PAL = [0.000 0.600 0.330; 0.165 0.471 0.839; 0.922 0.408 0.204; 0.929 0.631 0.000;
       0.910 0.482 0.643; 0.290 0.227 0.655; 0.000 0.514 0.514; 0.55 0.35 0.15; 0.45 0.45 0.45];

style_ax = @(a) set(a, 'Color', 'w', 'XColor', INK2, 'YColor', INK2, 'GridColor', GRIDC, ...
                    'GridAlpha', 1, 'FontSize', 9, 'Box', 'off');
setleg = @(lg) set(lg, 'TextColor', LEG_INK, 'Color', LEG_BG, 'EdgeColor', LEG_BG);

%% ===================== DOC LOG =====================
E  = fwmr_parse_log(LOG_EKF);
RE = fwmr_parse_log(LOG_ENC);
RI = fwmr_parse_log(LOG_IMU);
nE = numel(E);

bad = false(1, nE);
why = cell(1, nE);
for k = 1:nE
    r = E(k);
    dist_enc = r.cnt(end, :) * (2 * pi * 34) / 1496;       % quang duong tung banh (mm)
    w = {};
    if isnan(r.xc_end),                       w{end+1} = 'khong co khung camera luc dung'; end
    if ~isnan(r.cam_gap_ms) && r.cam_gap_ms > 1000
        w{end+1} = sprintf('mat camera %.1f s', r.cam_gap_ms / 1000);
    end
    if mean(dist_enc) > 1.3 * TARGET_MM
        w{end+1} = sprintf('encoder: xe da chay %.2f m', mean(dist_enc) / 1000);
    end
    bad(k) = ~isempty(w);
    why{k} = strjoin(w, ', ');
end
if ~isempty(BAD_RUNS), bad(:) = false; bad(BAD_RUNS) = true; end
good = find(~bad);  badk = find(bad);

%% ===================== CHI SO =====================
% Moi phuong phap: |y| cuoi, RMS lech ngang (tich phan theo x), |y| max, |goc| max, |goc| cuoi
% EKF: camera (vi tri that). enc/IMU: odometry encoder + huong IMU (xi, yi, yaw).
M_NAME = {'|y| cuoi (mm)', 'RMS lech ngang (mm)', '|y| lon nhat (mm)', '|goc| lon nhat (do)', '|goc| cuoi (do)'};

MET_E = NaN(nE, 5);
for k = 1:nE
    r = E(k);
    mv = (r.cmd > 0) & (r.camn > 0);         % khung camera moi, dang chay
    if isnan(r.xc_end), continue; end
    MET_E(k, :) = fwmr_metrics_xy(r.xc(mv), r.yc(mv), r.xc_end, r.yc_end, r.yaw(r.cmd > 0), r.thc_end);
end
MET_O = {NaN(numel(RE), 5), NaN(numel(RI), 5)};
SRC = {RE, RI};
for m = 1:2
    for k = 1:numel(SRC{m})
        r = SRC{m}(k);
        mv = r.t_ms <= r.t_hit;
        MET_O{m}(k, :) = fwmr_metrics_xy(r.xi(mv), r.yi(mv), r.xi_end, r.yi_end, r.yaw, r.yaw_end);
    end
end

fprintf('\n================ EKF: %d lan chay (%d binh thuong, %d loi camera) ================\n', ...
        nE, numel(good), numel(badk));
fprintf('%-4s %-8s %8s %8s %8s %8s %8s %8s %6s %6s %8s\n', 'Lan', 'Loai', 'xc_cuoi', 'yc_cuoi', ...
        'thc_cuoi', 'RMS_y', '|y|max', 'x_enc', 'loai', 'nap', 'matcam');
for k = 1:nE
    r = E(k);
    loai = 'OK'; if bad(k), loai = 'LOI_CAM'; end
    fprintf('%-4d %-8s %8.0f %8.0f %8.2f %8.1f %8.1f %8.0f %6d %6d %8d\n', k, ...
            loai, r.xc_end, r.yc_end, r.thc_end, MET_E(k, 2), ...
            MET_E(k, 3), r.x_end, r.n_cam_rej, r.n_reset, r.cam_gap_ms);
end
for k = badk
    fprintf('  Lan %d LOI: %s\n', k, why{k});
end

fprintf('\n================ SO SANH 3 PHUONG PHAP (trung binh +- do lech chuan) ================\n');
fprintf('%-22s %20s %20s %20s\n', 'Chi so', 'Thuan encoder*', 'Vong IMU KP=3*', 'EKF (camera)');
for i = 1:5
    a = MET_O{1}(:, i); b = MET_O{2}(:, i); c = MET_E(good, i);
    fprintf('%-22s %11.1f +- %6.1f %11.1f +- %6.1f %11.1f +- %6.1f\n', M_NAME{i}, ...
            mean(a), std(a), mean(b), std(b), mean(c), std(c));
end
fprintf('* enc/IMU: toa do tu odometry cua robot (chua co camera), EKF: toa do camera.\n\n');

%% ===================== HINH 1: QUY DAO EKF (CAMERA) =====================
fig1 = figure('Name', 'EKF - quy dao camera', 'Color', 'w', 'Position', [30 60 1400 560]);
ax = axes('Parent', fig1); hold(ax, 'on'); style_ax(ax); grid(ax, 'on');
patch('Parent', ax, 'XData', [0 TARGET_MM TARGET_MM 0], 'YData', [-TOL_MM -TOL_MM TOL_MM TOL_MM], ...
      'FaceColor', C_TOL, 'FaceAlpha', 0.15, 'EdgeColor', 'none');
hb = plot(ax, [0 TARGET_MM], [0 0], 'k-', 'LineWidth', 3);
plot(ax, [TARGET_MM TARGET_MM], [-120 120], ':', 'Color', [0.5 0.5 0.5]);
h = zeros(1, numel(good)); lab = cell(1, numel(good));
for j = 1:numel(good)
    k = good(j); r = E(k); c = PAL(j, :);
    mv = (r.camn > 0) & (r.cmd > 0);
    h(j) = plot(ax, [0; r.xc(mv); r.xc_end], [0; r.yc(mv); r.yc_end], '-o', 'Color', c, ...
                'LineWidth', 1.3, 'MarkerSize', PT_SIZE, 'MarkerFaceColor', c, 'MarkerEdgeColor', c);
    plot(ax, r.xc_end, r.yc_end, 'o', 'MarkerSize', 9, 'MarkerFaceColor', c, 'MarkerEdgeColor', 'w', 'LineWidth', 1.5);
    lab{j} = sprintf('Lan %d: cuoi (%.0f; %.0f) mm, %.1f^o', k, r.xc_end, r.yc_end, r.thc_end);
end
xlim(ax, [-100 TARGET_MM + 300]); ylim(ax, [-120 120]);
xlabel(ax, 'x doc duong co so (mm)', 'Color', INK); ylabel(ax, 'y lech ngang (mm) - truc y phong dai', 'Color', INK);
title(ax, sprintf('EKF: %d lan chay binh thuong, vi tri do bang camera - duong co so 3 m (hanh lang \\pm%d mm)', ...
      numel(good), TOL_MM), 'Color', INK, 'FontWeight', 'bold');
lg = legend(ax, [hb h], [{'Duong co so 3 m'}, lab], 'Location', 'eastoutside'); setleg(lg);
if SAVE_PNG, print(fig1, 'ekf_quy_dao.png', '-dpng', '-r130'); end

%% ===================== HINH 2: SO SANH 3 PHUONG PHAP =====================
fig2 = figure('Name', 'So sanh 3 phuong phap', 'Color', 'w', 'Position', [50 80 1400 700]);
titles = {'Toan canh', 'Phong to \pm100 mm'};
YLIM = {[-700 150], [-100 100]};
for s = 1:2
    ax = subplot(2, 1, s, 'Parent', fig2); hold(ax, 'on'); style_ax(ax); grid(ax, 'on');
    patch('Parent', ax, 'XData', [0 TARGET_MM TARGET_MM 0], 'YData', [-TOL_MM -TOL_MM TOL_MM TOL_MM], ...
          'FaceColor', C_TOL, 'FaceAlpha', 0.15, 'EdgeColor', 'none');
    hb = plot(ax, [0 TARGET_MM], [0 0], 'k-', 'LineWidth', 3);
    hh = zeros(1, 3);
    for k = 1:numel(RE)
        r = RE(k); mv = r.t_ms <= r.t_hit;
        hh(1) = plot(ax, [r.xi(mv); r.xi_end], [r.yi(mv); r.yi_end], '-', 'Color', C_ENC, 'LineWidth', 1.1);
    end
    for k = 1:numel(RI)
        r = RI(k); mv = r.t_ms <= r.t_hit;
        hh(2) = plot(ax, [r.xi(mv); r.xi_end], [r.yi(mv); r.yi_end], '-', 'Color', C_IMU, 'LineWidth', 1.1);
    end
    for k = good
        r = E(k); mv = (r.camn > 0) & (r.cmd > 0);
        hh(3) = plot(ax, [0; r.xc(mv); r.xc_end], [0; r.yc(mv); r.yc_end], '-', 'Color', C_EKF, 'LineWidth', 1.6);
    end
    xlim(ax, [-50 TARGET_MM + 250]); ylim(ax, YLIM{s});
    ylabel(ax, 'lech ngang y (mm)', 'Color', INK);
    title(ax, titles{s}, 'Color', INK, 'FontWeight', 'normal');
    if s == 1
        lg = legend(ax, [hb hh], {'Duong co so 3 m', sprintf('Thuan encoder (%d lan, odometry)', numel(RE)), ...
             sprintf('Vong IMU KP=3 (%d lan, odometry)', numel(RI)), sprintf('EKF (%d lan, camera)', numel(good))}, ...
             'Location', 'southwest');
        setleg(lg);
    else
        xlabel(ax, 'x doc duong co so (mm)', 'Color', INK);
    end
end
if SAVE_PNG, print(fig2, 'ekf_so_sanh_quy_dao.png', '-dpng', '-r130'); end

%% ===================== HINH 3: CHI SO =====================
fig3 = figure('Name', 'So sanh chi so', 'Color', 'w', 'Position', [70 100 1400 470]);
SHOW = [1 2 4];
CC = {C_ENC, C_IMU, C_EKF};
NM = {'Thuan enc*', 'IMU KP=3*', 'EKF'};
for s = 1:3
    i = SHOW(s);
    ax = axes('Parent', fig3, 'Position', [0.06 + (s - 1) * 0.33, 0.16, 0.26, 0.68]); hold(ax, 'on');
    style_ax(ax); grid(ax, 'on');
    V = {MET_O{1}(:, i), MET_O{2}(:, i), MET_E(good, i)};
    for g = 1:3
        v = V{g};
        bar(ax, g, mean(v), 0.6, 'FaceColor', CC{g}, 'EdgeColor', 'none');
        plot(ax, g + 0.12 * linspace(-1, 1, numel(v)), v, 'o', 'MarkerSize', 4, ...
             'MarkerFaceColor', 'w', 'MarkerEdgeColor', [0.2 0.2 0.2]);
        text(g, mean(v), sprintf('%.1f', mean(v)), 'HorizontalAlignment', 'center', ...
             'VerticalAlignment', 'bottom', 'Color', INK, 'FontSize', 9, 'FontWeight', 'bold');
    end
    set(ax, 'XTick', 1:3, 'XTickLabel', NM); xlim(ax, [0.4 3.6]);
    title(ax, M_NAME{i}, 'Color', INK, 'FontWeight', 'normal');
end
annotation(fig3, 'textbox', [0.06 0.01 0.9 0.06], 'String', ...
    '* Thuan encoder va IMU: toa do tu odometry cua robot (chua co camera). EKF: toa do camera. Cot = trung binh, cham = tung lan chay.', ...
    'EdgeColor', 'none', 'Color', INK2, 'FontSize', 8);
if SAVE_PNG, print(fig3, 'ekf_so_sanh_chi_so.png', '-dpng', '-r130'); end

%% ===================== HINH 4: CAC LAN LOI CAMERA =====================
if ~isempty(badk)
    fig4 = figure('Name', 'Lan chay loi camera', 'Color', 'w', 'Position', [90 120 1400 300 + 260 * numel(badk)]);
    for j = 1:numel(badk)
        k = badk(j); r = E(k); ts = r.t_ms / 1000;
        ax = subplot(numel(badk), 1, j, 'Parent', fig4); hold(ax, 'on'); style_ax(ax); grid(ax, 'on');
        frz = (r.camn == 0) & (r.cmd > 0);              % khong co khung moi khi dang chay
        yl = [0 max([r.x; r.xi; TARGET_MM]) * 1.08];
        for i = find(frz).'
            patch('Parent', ax, 'XData', ts(i) + [-0.05 0.05 0.05 -0.05], 'YData', [yl(1) yl(1) yl(2) yl(2)], ...
                  'FaceColor', [1 0.85 0.85], 'EdgeColor', 'none');
        end
        h1 = plot(ax, ts, r.x * 0.96, '-', 'Color', [0.35 0.35 0.35], 'LineWidth', 2);
        h2 = plot(ax, ts, r.xi, '-o', 'Color', C_EKF, 'LineWidth', 1.4, 'MarkerSize', 3, 'MarkerFaceColor', C_EKF);
        h3 = plot(ax, ts, r.xc, '-o', 'Color', C_BAD, 'LineWidth', 1.2, 'MarkerSize', 3, 'MarkerFaceColor', C_BAD);
        plot(ax, [ts(1) ts(end)], [TARGET_MM TARGET_MM], 'k--');
        ylim(ax, yl);
        ylabel(ax, 'quang duong (mm)', 'Color', INK);
        title(ax, sprintf('Lan %d - %s', k, why{k}), 'Color', INK, 'FontWeight', 'bold');
        if j == numel(badk), xlabel(ax, 't (s)', 'Color', INK); end
        lg = legend(ax, [h1 h2 h3], {'Encoder x 0.96 (quang duong that gan dung)', 'EKF', ...
                    'Camera (khung cuoi nhan duoc)'}, 'Location', 'northwest'); setleg(lg);
    end
    if SAVE_PNG, print(fig4, 'ekf_lan_loi_camera.png', '-dpng', '-r130'); end
end
