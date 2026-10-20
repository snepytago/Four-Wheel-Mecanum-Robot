%% compare_ekf_truoc_sau_sua.m
% So sanh bai chay thang 3 m EKF truoc va sau khi sua loi nap lai (19/10/2026).
%   18/10: nap lai = gan thang khung camera cu (~200 ms) cho hien tai
%   19/10: nap lai co bu tre (cong do lech tai thoi diem chup)
% Moi tham so khac giu nguyen -> khac biet chinh nam o phan vuot dich.
%
% Toa do: CAMERA (ArUco) trong he xuat phat, dich (3000; 0) mm.
% Lan chay khong hop le (tu nhan ra): khong co khung camera luc dung, hoac
% camera luc dung lech qua xa quang duong encoder (camera dong bang).
%
% Can cung thu muc: fwmr_parse_log.m, log_ekf_18102026.txt, log_ekf_19102026.txt

%clear; close all; clc;

%% ===================== CAU HINH =====================
LOGS      = {'log_ekf_18102026.txt', 'log_ekf_19102026.txt'};
NAME      = {'18/10 - truoc sua', '19/10 - sau sua'};
TARGET_MM = 3000;
TOL_MM    = 50;
STALE_MM  = 400;       % |camera cuoi - quang duong encoder| lon hon -> camera dong bang
SAVE_PNG  = true;

CC    = {[0.85 0.37 0.10], [0.00 0.60 0.33]};
C_BAD = [0.60 0.60 0.60];
INK   = [0 0 0];
INK2  = [0.32 0.32 0.31];
GRIDC = [0.88 0.88 0.87];
C_TOL = [0.106 0.686 0.478];

% RMS lech ngang theo camera (chi dong co khung moi, luc dang chay) - dung fwmr_metrics_xy.m
sel_mv  = @(r) find(r.camn > 0 & r.cmd > 0);
rms_lat = @(r) subsref(fwmr_metrics_xy(r.xc(sel_mv(r)), r.yc(sel_mv(r)), r.xc_end, r.yc_end, 0, 0), ...
                       struct('type', '()', 'subs', {{2}}));
welch_p = @(a, b) betainc((((std(a)^2/numel(a) + std(b)^2/numel(b))^2 / ((std(a)^2/numel(a))^2/(numel(a)-1) + (std(b)^2/numel(b))^2/(numel(b)-1))) / ...
                  (((std(a)^2/numel(a) + std(b)^2/numel(b))^2 / ((std(a)^2/numel(a))^2/(numel(a)-1) + (std(b)^2/numel(b))^2/(numel(b)-1))) + ...
                   (mean(a) - mean(b))^2 / (std(a)^2/numel(a) + std(b)^2/numel(b)))), ...
                  ((std(a)^2/numel(a) + std(b)^2/numel(b))^2 / ((std(a)^2/numel(a))^2/(numel(a)-1) + (std(b)^2/numel(b))^2/(numel(b)-1))) / 2, 0.5);

bar1 = @(ax, x, v, c) fill(ax, [x - 0.35, x + 0.35, x + 0.35, x - 0.35], [0 0 v v], c, 'EdgeColor', 'none');

style_ax = @(a) set(a, 'Color', 'w', 'XColor', INK2, 'YColor', INK2, 'GridColor', GRIDC, ...
                    'GridAlpha', 1, 'FontSize', 9, 'Box', 'off');

%% ===================== DOC LOG =====================
D = cell(1, 2);
for d = 1:2
    R = fwmr_parse_log(LOGS{d});
    n = numel(R);
    jumps = repmat({[]}, 1, n);                       % do nhay nap lai (chi log 19/10 co)
    parts = strsplit(fileread(LOGS{d}), 'ENC,t_ms');
    for k = 1:min(n, numel(parts) - 1)
        tok = regexp(parts{k + 1}, 'NAP_LAI nhay_mm=(\d+)', 'tokens');
        jumps{k} = cellfun(@(c) str2double(c{1}), tok);
    end
    ok = false(1, n);  why = cell(1, n);
    for k = 1:n
        r = R(k);
        enc = mean(r.cnt(end, :)) * (2 * pi * 34) / 1496;
        if isnan(r.xc_end)
            why{k} = 'khong co camera luc dung';
        elseif abs(r.xc_end - enc) > STALE_MM
            why{k} = sprintf('camera dong bang (cam %.0f / enc %.0f mm)', r.xc_end, enc);
        else
            ok(k) = true;  why{k} = '';
        end
    end
    D{d} = struct('R', R, 'ok', ok, 'why', {why}, 'jumps', {jumps});
end

%% ===================== CHI SO =====================
fprintf('\n%-18s %4s %9s %9s %8s %8s %7s %6s  %-22s %s\n', 'Ngay', 'Lan', 'x_cuoi', 'vuot', 'y_cuoi', 'RMS_y', 'loai%', 'naplai', 'nhay nap lai (mm)', 'ghi chu');
for d = 1:2
    S = D{d};
    for k = 1:numel(S.R)
        r = S.R(k);
        rms_y = rms_lat(r);
        rej = 100 * r.n_cam_rej / max(r.n_cam_rej + r.n_cam_ok, 1);
        fprintf('%-18s %4d %9.0f %9.0f %8.0f %8.0f %7.1f %6d  %-22s %s\n', NAME{d}, k, r.xc_end, ...
                r.xc_end - TARGET_MM, r.yc_end, rms_y, rej, r.n_reset, mat2str(S.jumps{k}), S.why{k});
    end
end
fprintf('\n%-18s %6s %16s %16s %16s %10s\n', 'Ngay', 'so lan', 'vuot dich (mm)', '|y| cuoi (mm)', 'RMS y (mm)', 'loai TB %');
M = cell(1, 2);
for d = 1:2
    S = D{d};  idx = find(S.ok);
    ov = [S.R(idx).xc_end] - TARGET_MM;
    ye = abs([S.R(idx).yc_end]);
    rm = arrayfun(@(r) rms_lat(r), S.R(idx));
    rj = arrayfun(@(r) 100 * r.n_cam_rej / max(r.n_cam_rej + r.n_cam_ok, 1), S.R);
    M{d} = struct('ov', ov, 'ye', ye, 'rm', rm);
    fprintf('%-18s %6d %8.0f +-%5.0f %8.1f +-%5.1f %8.1f +-%5.1f %10.1f\n', NAME{d}, numel(idx), ...
            mean(ov), std(ov), mean(ye), std(ye), mean(rm), std(rm), mean(rj));
end
p = welch_p(M{1}.ov, M{2}.ov);
fprintf('Welch t-test vuot dich 18/10 vs 19/10: p = %.5f\n\n', p);

%% ===================== HINH =====================
fig = figure('Color', 'w', 'Position', [60 60 1400 900]);

% (a) quy dao toan bo
ax1 = subplot(2, 2, 1); hold(ax1, 'on'); style_ax(ax1); grid(ax1, 'on');
fill(ax1, [0 TARGET_MM TARGET_MM 0], [-TOL_MM -TOL_MM TOL_MM TOL_MM], C_TOL, ...
     'FaceAlpha', 0.12, 'EdgeColor', 'none');
plot(ax1, [0 TARGET_MM], [0 0], '-', 'Color', INK, 'LineWidth', 1.2);
hL = zeros(1, 2);
for d = 1:2
    S = D{d};
    for k = find(S.ok)
        r = S.R(k);  s = [1; find(r.camn > 0)];
        h = plot(ax1, [r.xc(s); r.xc_end], [r.yc(s); r.yc_end], '.-', 'Color', CC{d}, ...
                 'MarkerSize', 6, 'LineWidth', 0.8);
        hL(d) = h;
    end
end
xlim(ax1, [-100 3400]);  ylim(ax1, [-110 110]);
xlabel(ax1, 'x doc duong (mm)');  ylabel(ax1, 'y lech ngang (mm)');
title(ax1, '(a) Quy dao theo camera - cac lan hop le', 'FontWeight', 'normal');
legend(ax1, hL, NAME, 'Location', 'southwest');

% (b) phong to quanh dich
ax2 = subplot(2, 2, 2); hold(ax2, 'on'); style_ax(ax2); grid(ax2, 'on');
fill(ax2, [2850 3000 3000 2850], [-TOL_MM -TOL_MM TOL_MM TOL_MM], C_TOL, 'FaceAlpha', 0.12, 'EdgeColor', 'none');
plot(ax2, [TARGET_MM TARGET_MM], [-100 110], '--', 'Color', INK, 'LineWidth', 1);
text(ax2, TARGET_MM + 4, 100, 'dich 3000 mm', 'Color', INK, 'FontSize', 9);
for d = 1:2
    S = D{d};
    for k = find(S.ok)
        r = S.R(k);  s = [1; find(r.camn > 0)];
        plot(ax2, [r.xc(s); r.xc_end], [r.yc(s); r.yc_end], '-', 'Color', 0.45 * CC{d} + 0.55, 'LineWidth', 0.8);
        plot(ax2, r.xc_end, r.yc_end, 'o', 'MarkerFaceColor', CC{d}, 'MarkerEdgeColor', 'w', 'MarkerSize', 8);
        text(ax2, r.xc_end + 5, r.yc_end + 6, sprintf('%d', k), 'Color', CC{d} * 0.8, 'FontSize', 8);
    end
end
for d = 1:2
    text(ax2, 2860, -60 - 16 * (d - 1), sprintf('%s: vuot TB %.0f mm', NAME{d}, mean(M{d}.ov)), ...
         'Color', CC{d} * 0.85, 'FontSize', 9, 'FontWeight', 'bold');
end
xlim(ax2, [2850 3200]);  ylim(ax2, [-100 110]);
xlabel(ax2, 'x (mm)');  ylabel(ax2, 'y (mm)');
title(ax2, '(b) Diem dung that (camera, trung binh khi dung yen)', 'FontWeight', 'normal');

% (c) vuot dich tung lan
ax3 = subplot(2, 2, 3); hold(ax3, 'on'); style_ax(ax3); grid(ax3, 'on');
pos = 0;  xt = [];  xl = {};
for d = 1:2
    S = D{d};  p0 = pos + 1;
    for k = 1:numel(S.R)
        pos = pos + 1;  xt(end+1) = pos;  xl{end+1} = sprintf('%d', k);
        if S.ok(k)
            v = S.R(k).xc_end - TARGET_MM;
            bar1(ax3, pos, v, CC{d});
            text(ax3, pos, v + 5, sprintf('%.0f', v), 'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', INK2);
        else
            plot(ax3, pos, 4, 'x', 'Color', C_BAD, 'MarkerSize', 8, 'LineWidth', 1.5);
            text(ax3, pos, 14, 'loi', 'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', C_BAD);
        end
    end
    m = mean(M{d}.ov);
    plot(ax3, [p0 - 0.4, pos + 0.4], [m m], '--', 'Color', CC{d} * 0.8, 'LineWidth', 1.2);
    text(ax3, (p0 + pos) / 2, 182, NAME{d}, 'HorizontalAlignment', 'center', 'Color', CC{d} * 0.85, 'FontSize', 9);
    pos = pos + 1;
end
set(ax3, 'XTick', xt, 'XTickLabel', xl);
ylim(ax3, [0 195]);  xlim(ax3, [0 pos]);
xlabel(ax3, 'lan chay  (x = loi camera, khong co vi tri that)');  ylabel(ax3, 'vuot dich = x_{cuoi} - 3000 (mm)');
title(ax3, sprintf('(c) Vuot dich: %.0f -> %.0f mm (p = %.4f)', mean(M{1}.ov), mean(M{2}.ov), p), 'FontWeight', 'normal');

% (d) tinh trang camera
ax4 = subplot(2, 2, 4); hold(ax4, 'on'); style_ax(ax4); grid(ax4, 'on');
pos = 0;  xt = [];  xl = {};
for d = 1:2
    S = D{d};
    for k = 1:numel(S.R)
        pos = pos + 1;  xt(end+1) = pos;  xl{end+1} = sprintf('%d', k);
        r = S.R(k);
        rej = 100 * r.n_cam_rej / max(r.n_cam_rej + r.n_cam_ok, 1);
        c = CC{d};  if ~S.ok(k), c = C_BAD; end
        bar1(ax4, pos, rej, c);
        text(ax4, pos, rej + 1.5, sprintf('%d', r.n_reset), 'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', INK);
        if r.cam_gap_ms > 1000
            text(ax4, pos, rej + 5, sprintf('%.1fs', r.cam_gap_ms / 1000), 'HorizontalAlignment', 'center', ...
                 'FontSize', 7, 'Color', [0.75 0.1 0.1]);
        end
    end
    pos = pos + 1;
end
set(ax4, 'XTick', xt, 'XTickLabel', xl);
ylim(ax4, [0 55]);  xlim(ax4, [0 pos]);
xlabel(ax4, 'lan chay  (so tren cot = so lan nap lai ; do = mat camera lau nhat)');
ylabel(ax4, 'khung camera bi loai (%)');
title(ax4, '(d) Tinh trang camera (xam = lan khong hop le)', 'FontWeight', 'normal');

if SAVE_PNG
    try, print(fig, 'ekf_truoc_sau_sua.png', '-dpng', '-r130'); catch, end
end
