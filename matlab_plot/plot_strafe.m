%% plot_strafe.m
% Bai chay ngang (strafe_cal_test) thuan encoder, khong camera.
%   (a) quy dao uoc luong: odometry K_SLIP + goc IMU (xi, yi), truc ngang = di ngang
%   (b) goc xoay theo thoi gian: IMU (that) so voi encoder (mo hinh banh xe)
%   (c) quang duong lan cua tung banh so voi trung binh 4 banh
% Can cung thu muc: fwmr_parse_log.m va file log.

%clear; close all; clc;

LOG_FILE = 'log_strafe_20102026.txt';
SAVE_PNG = true;
% Cat bo phan sau va cham (s); NaN = khong cat. Lan 2, 3 xe dam vao do (20/10/2026)
CUT_S    = [NaN 3.45 4.55 NaN];

INK2  = [0.32 0.32 0.31];
GRIDC = [0.88 0.88 0.87];
PAL   = [0.000 0.600 0.330; 0.165 0.471 0.839; 0.922 0.408 0.204; 0.910 0.482 0.643; 0.290 0.227 0.655];
style_ax = @(a) set(a, 'Color', 'w', 'XColor', INK2, 'YColor', INK2, 'GridColor', GRIDC, ...
                    'GridAlpha', 1, 'FontSize', 9, 'Box', 'off');

R = fwmr_parse_log(LOG_FILE);
n = numel(R);
hit = false(1, n);
for k = 1:min(n, numel(CUT_S))
    if ~isnan(CUT_S(k))
        m = R(k).t_ms <= CUT_S(k) * 1000;
        R(k).t_ms = R(k).t_ms(m);  R(k).xi = R(k).xi(m);  R(k).yi = R(k).yi(m);
        R(k).yaw  = R(k).yaw(m);   R(k).th = R(k).th(m);  R(k).cnt = R(k).cnt(m, :);
        hit(k) = true;
    end
end
fprintf('\n%4s %8s %10s %10s %10s %10s  %s\n', 'Lan', 't (s)', 'yaw IMU', 'th enc', 'ngang lan', 'RR/TB', 'ghi chu');
for k = 1:n
    r = R(k);
    c = abs(r.cnt(end, :));
    full = ~isnan(r.t_hit);
    ngang = (2 * pi * 34 / 1496) * (-r.cnt(end, 1) + r.cnt(end, 2) + r.cnt(end, 3) - r.cnt(end, 4)) / 4;
    note = '';  if hit(k), note = sprintf('dam vao do, cat o %.2f s', CUT_S(k)); elseif ~full, note = 'dung giua chung'; end
    fprintf('%4d %8.1f %10.1f %10.1f %10.0f %10.3f  %s\n', k, r.t_ms(end) / 1000, r.yaw(end), r.th(end), ...
            ngang, c(4) / mean(c), note);
end

fig = figure('Color', 'w', 'Position', [60 60 1500 520]);

ax1 = subplot(1, 3, 1); hold(ax1, 'on'); style_ax(ax1); grid(ax1, 'on');
h = zeros(1, n);  lab = cell(1, n);
for k = 1:n
    r = R(k);
    h(k) = plot(ax1, r.yi, r.xi, '.-', 'Color', PAL(k, :), 'MarkerSize', 6);
    if hit(k)
        plot(ax1, r.yi(end), r.xi(end), 'x', 'Color', PAL(k, :), 'MarkerSize', 10, 'LineWidth', 2);
        lab{k} = sprintf('lan %d (dam vao do)', k);
    else
        plot(ax1, r.yi(end), r.xi(end), 'o', 'MarkerFaceColor', PAL(k, :), 'MarkerEdgeColor', 'w', 'MarkerSize', 7);
        lab{k} = sprintf('lan %d', k);
    end
end
plot(ax1, [0 3000 * 0.75], [0 0], 'k-', 'LineWidth', 1.2);
axis(ax1, 'equal');
xlabel(ax1, 'ngang sang trai (mm)');  ylabel(ax1, 'doc than xe luc dau (mm)');
title(ax1, '(a) Quy dao uoc luong (encoder x he so truot + goc IMU)', 'FontWeight', 'normal');
legend(ax1, h, lab, 'Location', 'northwest');

ax2 = subplot(1, 3, 2); hold(ax2, 'on'); style_ax(ax2); grid(ax2, 'on');
for k = 1:n
    r = R(k);
    plot(ax2, r.t_ms / 1000, r.yaw, '-', 'Color', PAL(k, :), 'LineWidth', 1.4);
    plot(ax2, r.t_ms / 1000, r.th,  '--', 'Color', PAL(k, :), 'LineWidth', 0.9);
    if hit(k), plot(ax2, r.t_ms(end) / 1000, r.yaw(end), 'x', 'Color', PAL(k, :), 'MarkerSize', 10, 'LineWidth', 2); end
end
plot(ax2, [0 7], [0 0], 'k-');
xlabel(ax2, 't (s)');  ylabel(ax2, 'goc xoay (do)');
title(ax2, '(b) Goc xoay: IMU (lien) vs encoder (dut) ; x = va cham', 'FontWeight', 'normal');

ax3 = subplot(1, 3, 3); hold(ax3, 'on'); style_ax(ax3); grid(ax3, 'on');
W = {'FL', 'FR', 'RL', 'RR'};
M = zeros(n, 4);
for k = 1:n
    c = abs(R(k).cnt(end, :));
    M(k, :) = 100 * (c / mean(c) - 1);
end
for w = 1:4
    for k = 1:n
        x = w + (k - (n + 1) / 2) * 0.18;
        fill(ax3, [x - 0.08 x + 0.08 x + 0.08 x - 0.08], [0 0 M(k, w) M(k, w)], PAL(k, :), 'EdgeColor', 'none');
    end
end
set(ax3, 'XTick', 1:4, 'XTickLabel', W);  xlim(ax3, [0.4 4.6]);
ylabel(ax3, 'quang lan so voi TB 4 banh (%)');
title(ax3, '(c) Toc do tung banh (cung duty 400)', 'FontWeight', 'normal');

if SAVE_PNG
    try, print(fig, 'strafe_thuan_encoder.png', '-dpng', '-r130'); catch, end
end
