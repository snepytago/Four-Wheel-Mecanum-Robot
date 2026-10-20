%% plot_rot.m
% Bai hieu chuan quay tai cho (rot_cal_test), buoc 1 zigzag. Khong camera.
%   (a) goc theo thoi gian: IMU (lien) vs encoder (dut)
%   (b) k_rot = goc IMU / goc encoder tung lan quay (trai / phai)
%   (c) goc vot sau khi cat dong co (IMU)
%   (d) gyro z theo thoi gian
% Can cung thu muc: fwmr_parse_log.m va file log.

%clear; close all; clc;

LOG_FILE = 'log_rot_20102026.txt';
SAVE_PNG = true;

INK2  = [0.32 0.32 0.31];
GRIDC = [0.88 0.88 0.87];
PAL   = [0.000 0.600 0.330; 0.165 0.471 0.839; 0.922 0.408 0.204; 0.910 0.482 0.643; 0.290 0.227 0.655];
C_L   = [0.165 0.471 0.839];
C_R   = [0.922 0.408 0.204];
style_ax = @(a) set(a, 'Color', 'w', 'XColor', INK2, 'YColor', INK2, 'GridColor', GRIDC, ...
                    'GridAlpha', 1, 'FontSize', 9, 'Box', 'off');

R = fwmr_parse_log(LOG_FILE);
n = numel(R);

% Dong "QUAY:KQ" cua tung lan chay (lay lan in dau tien cua moi k)
KEYS = {'t_cat_ms', 't_dung_ms', 'enc_cat', 'enc_dung', 'imu_cat', 'imu_dung', 'vot', 'k_rot', ...
        'wz_tb', 'gz_max', 'tx_mm', 'ty_mm'};
Q = cell(1, n);
fid = fopen(LOG_FILE, 'r');
ir = 0;
while true
    ln = fgetl(fid);
    if ~ischar(ln), break; end
    ln = ln(ln >= 32 & ln <= 126);
    if ~isempty(strfind(ln, 'ENC,t_ms')), ir = ir + 1; Q{ir} = zeros(0, 2 + numel(KEYS)); continue; end
    p = strfind(ln, 'QUAY:KQ k=');
    if ir == 0 || ir > n || isempty(p), continue; end
    ln = ln(p(1):end);
    k = sscanf(ln(11:end), '%d', 1);
    if isempty(k) || any(Q{ir}(:, 1) == k), continue; end
    v = NaN(1, 2 + numel(KEYS));
    v(1) = k;
    v(2) = 1 - 2 * isempty(strfind(ln, 'huong=TRAI'));          % +1 trai, -1 phai
    for j = 1:numel(KEYS)
        q = strfind(ln, [' ' KEYS{j} '=']);
        if ~isempty(q), v(2 + j) = sscanf(ln(q(1) + numel(KEYS{j}) + 2:end), '%f', 1); end
    end
    Q{ir}(end + 1, :) = v;
end
fclose(fid);

col = @(name) 2 + find(strcmp(KEYS, name));
fprintf('\n%4s %3s %6s %9s %9s %9s %8s %7s %8s %8s\n', 'Lan', 'k', 'huong', 'enc_dung', 'imu_dung', ...
        'vot', 'k_rot', 'wz_tb', 'tx_mm', 'ty_mm');
for r = 1:n
    for i = 1:size(Q{r}, 1)
        v = Q{r}(i, :);
        hs = 'TRAI'; if v(2) < 0, hs = 'PHAI'; end
        fprintf('%4d %3d %6s %9.1f %9.1f %9.2f %8.3f %7.1f %8.0f %8.0f\n', r, v(1), hs, v(col('enc_dung')), ...
                v(col('imu_dung')), v(col('vot')), v(col('k_rot')), v(col('wz_tb')), v(col('tx_mm')), v(col('ty_mm')));
    end
end
A = cell2mat(Q(:));
if ~isempty(A)
    for s = [1 -1]
        m = A(:, 2) == s;
        if any(m)
            hs = 'TRAI'; if s < 0, hs = 'PHAI'; end
            fprintf('TB %s (%d lan): k_rot = %.3f +- %.3f   vot = %.2f +- %.2f do   imu_dung = %.1f do\n', hs, ...
                    sum(m), mean(A(m, col('k_rot'))), std(A(m, col('k_rot'))), mean(A(m, col('vot'))), ...
                    std(A(m, col('vot'))), mean(abs(A(m, col('imu_dung')))));
        end
    end
end

fig = figure('Color', 'w', 'Position', [60 60 1300 760]);

ax1 = subplot(2, 2, 1); hold(ax1, 'on'); style_ax(ax1); grid(ax1, 'on');
h = zeros(1, n);  lab = cell(1, n);
for r = 1:n
    c = PAL(1 + mod(r - 1, size(PAL, 1)), :);
    h(r) = plot(ax1, R(r).t_ms / 1000, R(r).yaw, '-', 'Color', c, 'LineWidth', 1.4);
    plot(ax1, R(r).t_ms / 1000, R(r).th, '--', 'Color', c, 'LineWidth', 0.9);
    lab{r} = sprintf('lan %d', r);
end
xlabel(ax1, 't (s)');  ylabel(ax1, 'goc (do)');
title(ax1, '(a) Goc: IMU (lien) vs encoder (dut)', 'FontWeight', 'normal');
if n > 0, legend(ax1, h, lab, 'Location', 'best'); end

ax2 = subplot(2, 2, 2); hold(ax2, 'on'); style_ax(ax2); grid(ax2, 'on');
ax3 = subplot(2, 2, 3); hold(ax3, 'on'); style_ax(ax3); grid(ax3, 'on');
for r = 1:n
    for i = 1:size(Q{r}, 1)
        v = Q{r}(i, :);
        c = C_L; if v(2) < 0, c = C_R; end
        x = v(1) + (r - (n + 1) / 2) * 0.12;
        plot(ax2, x, v(col('k_rot')), 'o', 'MarkerFaceColor', c, 'MarkerEdgeColor', 'w', 'MarkerSize', 7);
        plot(ax3, x, v(col('vot')), 'o', 'MarkerFaceColor', c, 'MarkerEdgeColor', 'w', 'MarkerSize', 7);
    end
end
hl = plot(ax2, NaN, NaN, 'o', 'MarkerFaceColor', C_L, 'MarkerEdgeColor', 'w', 'MarkerSize', 7);
hr = plot(ax2, NaN, NaN, 'o', 'MarkerFaceColor', C_R, 'MarkerEdgeColor', 'w', 'MarkerSize', 7);
legend(ax2, [hl hr], {'quay trai', 'quay phai'}, 'Location', 'best');
plot(ax2, [0.5 max([A(:, 1); 1]) + 0.5], [1 1], 'k-');
xlabel(ax2, 'lan quay k');  ylabel(ax2, 'goc IMU / goc encoder');
title(ax2, '(b) He so truot khi quay k\_rot', 'FontWeight', 'normal');
xlabel(ax3, 'lan quay k');  ylabel(ax3, 'vot (do)');
title(ax3, '(c) Goc vot sau khi cat dong co (IMU)', 'FontWeight', 'normal');

ax4 = subplot(2, 2, 4); hold(ax4, 'on'); style_ax(ax4); grid(ax4, 'on');
for r = 1:n
    c = PAL(1 + mod(r - 1, size(PAL, 1)), :);
    plot(ax4, R(r).t_ms / 1000, R(r).gz, '-', 'Color', c, 'LineWidth', 1.0);
end
plot(ax4, xlim(ax4), [250 250], 'k:');  plot(ax4, xlim(ax4), [-250 -250], 'k:');
xlabel(ax4, 't (s)');  ylabel(ax4, 'gyro z (do/s)');
title(ax4, '(d) Toc do quay IMU (cham = gioi han +-250)', 'FontWeight', 'normal');

if SAVE_PNG
    [~, base] = fileparts(LOG_FILE);
    try, print(fig, [base '_quay.png'], '-dpng', '-r130'); catch, end
end
