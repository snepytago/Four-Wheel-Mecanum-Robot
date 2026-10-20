%% plot_fwmr_runs.m
% Ve lai cac lan chay cua robot FWMR (ve ngay, khong animation), ghi ro toa do
% diem cuoi lay tu khoi KET QUA cua STM32, tu luu PNG: <ten log>_quy_dao.png,
% <ten log>_xung_banh.png, <ten log>_yaw.png.
%
% Can 2 file cung thu muc: fwmr_parse_log.m va file log (.txt).
% x tien, y trai, goc duong = quay trai. Log co IMU thi mac dinh ve theo
% odometry encoder + huong IMU (POSE_SRC), log cu thi ve theo encoder.
% Ve dung du lieu log: moi diem (100 ms) la 1 cham, cac cham noi bang doan
% thang, khong noi suy. Robot nhay tu diem log nay sang diem log ke tiep.

%clear; close all; clc;

%% ===================== CAU HINH =====================
LOG_FILE  = 'log_imu_kp3_07102026.txt';
RUNS_SHOW = [];        % [] = ve tat ca; vd [2 4] = chi lan 2 va lan 4
SAVE_PNG  = true;      % tu luu toan bo hinh ra PNG
TARGET_MM = 3000;      % quang duong muc tieu
TARGET_CNT = 21008;    % so xung moc dung
POSE_SRC  = 'auto';    % 'auto' = dung IMU neu log co, 'enc' = luon dung encoder
PT_SIZE   = 4;         % kich thuoc cham moi diem du lieu

% Mau theo thu tu co dinh: lan 1 xanh duong, 2 cam, 3 xanh ngoc, 4 vang ...
PALETTE = [0.165 0.471 0.839;   % #2a78d6
           0.922 0.408 0.204;   % #eb6834
           0.106 0.686 0.478;   % #1baf7a
           0.929 0.631 0.000;   % #eda100
           0.910 0.482 0.643;   % #e87ba4
           0.000 0.514 0.000;   % #008300
           0.290 0.227 0.655;   % #4a3aa7
           0.890 0.286 0.282];  % #e34948
INK   = [0 0 0];               % chu chinh: tieu de, nhan toa do (den)
LEG_INK = [1 1 1];             % chu trong bang chu thich (trang)
INK2  = [0.32 0.32 0.31];       % chu phu, truc
GRIDC = [0.85 0.85 0.84];

%% ===================== DOC LOG =====================
runs = fwmr_parse_log(LOG_FILE);
ids  = 1:numel(runs);
if ~isempty(RUNS_SHOW), ids = RUNS_SHOW; end
if numel(ids) > size(PALETTE, 1)
    error('Toi da %d lan chay moi hinh - chon bot bang RUNS_SHOW.', size(PALETTE, 1));
end
nR = numel(ids);

% Chon nguon toa do de ve: encoder + huong IMU (neu co) hoac thuan encoder
has_imu = false(1, numel(runs));
for k = 1:numel(runs)
    runs(k).th_enc_end = runs(k).th_end;
    runs(k).th_enc     = runs(k).th;
    if ~strcmp(POSE_SRC, 'enc') && ~isempty(runs(k).yaw) && ~any(isnan(runs(k).yaw))
        has_imu(k) = true;
        runs(k).x = runs(k).xi;   runs(k).y = runs(k).yi;   runs(k).th = runs(k).yaw;
        runs(k).x_end = runs(k).xi_end;  runs(k).y_end = runs(k).yi_end;
        runs(k).th_end = runs(k).yaw_end;
    end
end
src_txt = 'encoder';
if any(has_imu(ids)), src_txt = 'encoder + huong IMU'; end
fprintf('Doc duoc %d lan chay tu %s, ve %d lan.\n\n', numel(runs), LOG_FILE, nR);

%% ===================== BANG KET QUA =====================
names = {'FL', 'FR', 'RL', 'RR'};
fprintf('Toa do theo: %s\n', src_txt);
fprintf('%-5s %-5s %7s %7s %7s %7s %7s | %7s %7s %8s %8s | %7s %7s\n', 'Lan', 'Moc', 't (ms)', ...
        'FL', 'FR', 'RL', 'RR', 'x (mm)', 'y (mm)', 'goc (do)', 'th_enc', '|yaw|max', 'u_tb');
for i = 1:nR
    r = runs(ids(i));
    fprintf('%-5d %-5s %7.0f %7d %7d %7d %7d | %7.0f %7.0f %8.2f %8.2f | %7.2f %7.1f\n', ids(i), ...
            r.hit_wheel, r.t_hit, r.cnt_final, r.x_end, r.y_end, r.th_end, r.th_enc_end, ...
            r.yaw_max_abs, r.u_mean);
end
fprintf('\n');

%% ===================== HINH 1: QUY DAO =====================
% Gioi han truc
allx = []; ally = [];
for i = 1:nR
    allx = [allx; runs(ids(i)).x]; ally = [ally; runs(ids(i)).y]; %#ok<AGROW>
end
xl = [min([allx; 0]) - 250, max([allx; TARGET_MM]) + 1350];
yl = [min([ally; 0]) - 250, max([ally; 0]) + 250];

fig1 = figure('Name', 'FWMR - quy dao encoder', 'Color', 'w', 'Position', [40 80 1350 600]);
ax = axes('Parent', fig1); hold(ax, 'on');
set(ax, 'Color', 'w', 'XColor', INK2, 'YColor', INK2, 'GridColor', GRIDC, ...
        'GridAlpha', 1, 'FontSize', 10, 'Box', 'off');
grid(ax, 'on'); axis(ax, 'equal'); xlim(ax, xl); ylim(ax, yl);
xlabel(ax, 'x (mm)'); ylabel(ax, 'y (mm)');
title(ax, sprintf('FWMR - quy dao theo %s, %d lan chay %.1f m (dung khi 1 banh = %d xung)', ...
      src_txt, nR, TARGET_MM/1000, TARGET_CNT), 'Color', INK, 'FontWeight', 'normal');

% Moc tham chieu
plot(ax, [0 TARGET_MM], [0 0], '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1);
plot(ax, [TARGET_MM TARGET_MM], yl, ':', 'Color', [0.6 0.6 0.6], 'LineWidth', 1);
text(TARGET_MM + 20, yl(2) - 40, sprintf('x = %d mm', TARGET_MM), 'Color', INK2, ...
     'FontSize', 9, 'VerticalAlignment', 'top');
plot(ax, 0, 0, 'o', 'MarkerSize', 7, 'MarkerFaceColor', INK, 'MarkerEdgeColor', 'w');
text(-30, -60, 'xuat phat (0; 0)', 'Color', INK2, 'FontSize', 9, 'HorizontalAlignment', 'right');
% Ve ngay toan bo duong di cua tung lan chay
h_trail = zeros(1, nR);
for i = 1:nR
    c = PALETTE(i, :);
    r = runs(ids(i));
    h_trail(i) = plot(ax, r.x, r.y, '-o', 'Color', c, 'LineWidth', 1.25, ...
                      'MarkerSize', PT_SIZE, 'MarkerFaceColor', c, 'MarkerEdgeColor', c);
end

% Diem cuoi + nhan toa do (lay tu khoi KET QUA), nhan xep thanh 1 cot ben phai
% va gian cach de khong de chu len nhau khi cac diem cuoi gan nhau.
x_lab = -inf;  y_end = zeros(1, nR);
for i = 1:nR
    x_lab = max(x_lab, runs(ids(i)).x_end);
    y_end(i) = runs(ids(i)).y_end;
end
x_lab = max(x_lab, TARGET_MM) + 200;
gap = 0.075 * diff(yl);                       % khoang cach toi thieu giua 2 nhan
[~, order] = sort(y_end, 'descend');
y_lab = y_end;
for j = 2:nR
    a = order(j - 1);  b = order(j);
    y_lab(b) = min(y_lab(b), y_lab(a) - gap);
end
y_lab = y_lab + (mean(y_end) - mean(y_lab));  % can giua cum nhan quanh cac diem
y_lab = y_lab - max(0, max(y_lab) - (yl(2) - gap)) + max(0, (yl(1) + gap) - min(y_lab));

for i = 1:nR
    r = runs(ids(i));
    plot(ax, [r.x_end x_lab - 90 x_lab - 20], [r.y_end y_lab(i) y_lab(i)], ':', ...
         'Color', PALETTE(i, :), 'LineWidth', 1);
    plot(ax, r.x_end, r.y_end, 'o', 'MarkerSize', 9, 'MarkerFaceColor', PALETTE(i, :), ...
         'MarkerEdgeColor', 'w', 'LineWidth', 2);
    if has_imu(ids(i))
        lab = sprintf('Lan %d: x = %.0f mm, y = %.0f mm, yaw = %.1f^o  (\\theta_{enc} = %.1f^o)', ...
                      ids(i), r.x_end, r.y_end, r.th_end, r.th_enc_end);
    else
        lab = sprintf('Lan %d: x = %.0f mm, y = %.0f mm, \\theta = %.1f^o  (moc: %s)', ...
                      ids(i), r.x_end, r.y_end, r.th_end, r.hit_wheel);
    end
    text(x_lab, y_lab(i), lab, 'Color', INK, 'FontSize', 10, 'VerticalAlignment', 'middle');
end
legend(h_trail, arrayfun(@(k) sprintf('Lan %d', k), ids, 'UniformOutput', false), ...
       'Location', 'southwest', 'TextColor', LEG_INK);

%% ===================== HINH 2: XUNG CUOI CUNG TUNG BANH =====================
fig2 = figure('Name', 'FWMR - xung tung banh', 'Color', 'w', 'Position', [80 120 760 460]);
ax2 = axes('Parent', fig2); hold(ax2, 'on');
set(ax2, 'XColor', INK2, 'YColor', INK2, 'GridColor', GRIDC, 'GridAlpha', 1, 'FontSize', 10, 'Box', 'off');
grid(ax2, 'on');
off = linspace(-0.18, 0.18, max(nR, 2));
if nR == 1, off = 0; end
h_dot = zeros(1, nR);
for i = 1:nR
    r = runs(ids(i));
    h_dot(i) = plot(ax2, (1:4) + off(i), r.cnt_final, 'o', 'MarkerSize', 9, ...
                    'MarkerFaceColor', PALETTE(i, :), 'MarkerEdgeColor', 'w', 'LineWidth', 1.5);
end
plot(ax2, [0.5 4.5], [TARGET_CNT TARGET_CNT], '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1);
text(4.45, TARGET_CNT, sprintf('moc %d xung ', TARGET_CNT), 'Color', INK2, 'FontSize', 9, ...
     'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');
set(ax2, 'XTick', 1:4, 'XTickLabel', names, 'XLim', [0.5 4.5]);
ylabel(ax2, 'xung cuoi cung (sau khi troi)');
title(ax2, 'Xung tung banh - cung lenh, khac toc do', 'Color', INK, 'FontWeight', 'normal');
legend(h_dot, arrayfun(@(k) sprintf('Lan %d', k), ids, 'UniformOutput', false), ...
       'Location', 'northwest', 'TextColor', LEG_INK);

%% ===================== HINH 3: YAW IMU VA HIEU CHINH u (chi khi log co IMU) =====================
if any(has_imu(ids))
    fig3 = figure('Name', 'FWMR - yaw va hieu chinh', 'Color', 'w', 'Position', [120 160 900 620]);
    ax3 = subplot(2, 1, 1, 'Parent', fig3); hold(ax3, 'on');
    ax4 = subplot(2, 1, 2, 'Parent', fig3); hold(ax4, 'on');
    set([ax3 ax4], 'XColor', INK2, 'YColor', INK2, 'GridColor', GRIDC, 'GridAlpha', 1, ...
        'FontSize', 10, 'Box', 'off');
    grid(ax3, 'on'); grid(ax4, 'on');
    h_y = zeros(1, nR);
    for i = 1:nR
        r = runs(ids(i));
        if ~has_imu(ids(i)), continue; end
        ts = r.t_ms / 1000;
        c = PALETTE(i, :);
        h_y(i) = plot(ax3, ts, r.yaw, '-o', 'Color', c, 'LineWidth', 1.25, ...
                      'MarkerSize', PT_SIZE, 'MarkerFaceColor', c, 'MarkerEdgeColor', c);
        plot(ax3, ts, r.th_enc, '--o', 'Color', c, 'LineWidth', 0.75, ...
             'MarkerSize', PT_SIZE - 1, 'MarkerFaceColor', 'w', 'MarkerEdgeColor', c);
        plot(ax4, ts, r.u, '-o', 'Color', c, 'LineWidth', 1.25, ...
             'MarkerSize', PT_SIZE, 'MarkerFaceColor', c, 'MarkerEdgeColor', c);
    end
    plot(ax3, [0 max(cellfun(@(t) t(end), {runs(ids).t_ms})) / 1000], [0 0], '--', 'Color', [0.6 0.6 0.6]);
    ylabel(ax3, 'goc huong (do)');
    title(ax3, 'Huong xe: cham dac = IMU, cham rong net dut = \\theta encoder (muc tieu 0^o)', ...
          'Color', INK, 'FontWeight', 'normal');
    ylabel(ax4, 'u (duty)');  xlabel(ax4, 't (s)');
    title(ax4, 'Hieu chinh u: ben phai +u, ben trai -u', 'Color', INK, 'FontWeight', 'normal');
    keep = h_y ~= 0;
    legend(ax3, h_y(keep), arrayfun(@(k) sprintf('Lan %d', k), ids(keep), 'UniformOutput', false), ...
           'Location', 'southeast', 'TextColor', LEG_INK);
end

%% ===================== LUU HINH =====================
if SAVE_PNG
    [~, base] = fileparts(LOG_FILE);
    print(fig1, [base '_quy_dao.png'], '-dpng', '-r130');
    print(fig2, [base '_xung_banh.png'], '-dpng', '-r130');
    if exist('fig3', 'var') && ishandle(fig3), print(fig3, [base '_yaw.png'], '-dpng', '-r130'); end
    fprintf('Da luu hinh: %s_*.png\n', base);
end
