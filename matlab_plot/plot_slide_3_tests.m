%% plot_slide_3_tests.m
% Hinh cho cum slide so sanh 3 bai chay thang 3 m (khong ve than xe):
%   slide_enc_8lan.png   : du 8 lan thuan encoder        (log_enc_run_06102026.txt)
%   slide_imu_8lan.png   : du 8 lan encoder + IMU, KP=3  (log_imu_kp3_07102026.txt)
%   slide_ekf_9lan.png   : du 9 lan EKF + camera         (log_ekf_18102026.txt)
%   slide_so_sanh_tot_nhat.png : lan tot nhat cua moi bai tren cung 1 hinh
% Lan tot nhat = nho nhat (RMS lech ngang + khoang cach toi dich (3000; 0)).
% Toa do: enc, IMU = odometry cua robot (xi, yi) ; EKF = camera (xc, yc).
% Can cung thu muc: fwmr_parse_log.m, fwmr_metrics_xy.m va 3 file log.
% Chay trong MATLAB: mo 4 figure (1: encoder, 2: IMU, 3: EKF, 4: so sanh).
% Tu luu 4 hinh PNG (SAVE_PNG = false de tat).

%clear; close all; clc;

LOGS   = {'log_enc_run_06102026.txt', 'log_imu_kp3_07102026.txt', 'log_ekf_18102026.txt'};
NAME   = {'Thuan encoder', 'Encoder + IMU (KP = 3)', 'EKF + camera'};
FILE   = {'slide_enc_8lan.png', 'slide_imu_8lan.png', 'slide_ekf_9lan.png'};
SRC    = {'odometry cua robot (encoder + goc IMU)', 'odometry cua robot (encoder + goc IMU)', 'camera tran (vi tri that)'};
TARGET = 3000;
TOL    = 50;
SAVE_PNG = true;

CC    = {[0.85 0.37 0.10], [0.16 0.44 0.79], [0.05 0.54 0.31]};
PAL   = [0.12 0.47 0.71; 0.85 0.37 0.10; 0.17 0.63 0.17; 0.84 0.15 0.16; 0.58 0.40 0.74; ...
         0.55 0.34 0.29; 0.89 0.47 0.76; 0.50 0.50 0.50; 0.74 0.74 0.13];
INK2  = [0.32 0.32 0.31];
GRIDC = [0.88 0.88 0.87];
C_TOL = [0.85 0.94 0.89];
style_ax = @(a) set(a, 'Color', 'w', 'XColor', INK2, 'YColor', INK2, 'GridColor', GRIDC, ...
                    'GridAlpha', 1, 'FontSize', 11, 'Box', 'off');

%% ---------- doc log, chuan hoa, tinh chi so ----------
Q = cell(1, 3);
for m = 1:3
    R = fwmr_parse_log(LOGS{m});
    for k = 1:numel(R)
        r = R(k);
        q = struct('run', k, 'ok', true, 'note', '', 'px', [], 'py', [], 'hd', [], 'd', [], ...
                   'xe', NaN, 'ye', NaN, 'he', NaN, 'rms', NaN, 'dist', NaN, 'score', Inf);
        if m < 3
            mv = r.t_ms <= r.t_hit;
            q.px = r.xi(mv); q.py = r.yi(mv); q.hd = r.yaw(mv); q.d = r.t_ms(mv);
            q.xe = r.xi_end; q.ye = r.yi_end; q.he = r.yaw_end;
        else
            sel = unique([1; find(r.camn > 0 & r.cmd > 0)]);
            q.px = r.xc(sel); q.py = r.yc(sel); q.hd = r.yaw(sel); q.d = r.t_ms(sel);
            q.xe = r.xc_end; q.ye = r.yc_end; q.he = r.thc_end;
            dist_enc = mean(r.cnt(end, :)) * (2 * pi * 34) / 1496;
            if isnan(q.xe) || r.cam_gap_ms > 1000 || dist_enc > 1.3 * TARGET
                q.ok = false;
                q.note = sprintf('loi camera: encoder %.1f m', dist_enc / 1000);
                q.xe = q.px(end); q.ye = q.py(end); q.he = q.hd(end);
            end
        end
        if q.ok
            mt = fwmr_metrics_xy(q.px, q.py, q.xe, q.ye, q.hd, q.he);
            q.rms = mt(2); q.dist = hypot(q.xe - TARGET, q.ye); q.score = q.rms + q.dist;
        else
            q.rms = NaN; q.dist = NaN; q.score = Inf;
        end
        if k == 1, Q{m} = q; else, Q{m}(k) = q; end
    end
end

best = zeros(1, 3);
fprintf('\n%-24s %4s %8s %8s %8s %9s\n', 'Bai', 'Lan', 'y cuoi', 'RMS', 'cach dich', 'diem');
for m = 1:3
    [~, best(m)] = min([Q{m}.score]);
    for k = 1:numel(Q{m})
        q = Q{m}(k);
        mark = '';  if k == best(m), mark = '  <- tot nhat'; end
        if ~q.ok, mark = ['  ' q.note]; end
        fprintf('%-24s %4d %8.0f %8.1f %8.0f %9.1f%s\n', NAME{m}, k, q.ye, q.rms, q.dist, q.score, mark);
    end
end

%% ---------- 1 hinh cho moi bai: du cac lan chay ----------
for m = 1:3
    A = Q{m};  n = numel(A);
    ok = [A.ok];
    fig = figure('Color', 'w', 'Position', [40 40 1600 880], 'Name', sprintf('%d - %s', m, NAME{m}));

    ax1 = axes('Position', [0.085 0.54 0.675 0.39]); hold(ax1, 'on'); style_ax(ax1); grid(ax1, 'on');
    fill(ax1, [0 TARGET TARGET 0], [-TOL -TOL TOL TOL], C_TOL, 'EdgeColor', 'none');
    plot(ax1, [0 TARGET], [0 0], 'k-', 'LineWidth', 2);
    plot(ax1, TARGET, 0, 'kp', 'MarkerSize', 14, 'MarkerFaceColor', [1 0.84 0]);
    h = zeros(1, n);  lab = cell(1, n);
    for k = 1:n
        q = A(k);  c = PAL(k, :);
        if q.ok
            h(k) = plot(ax1, [q.px; q.xe], [q.py; q.ye], '.-', 'Color', c, 'LineWidth', 1.2, 'MarkerSize', 8);
            plot(ax1, q.xe, q.ye, 'o', 'MarkerFaceColor', c, 'MarkerEdgeColor', 'w', 'MarkerSize', 9);
            lab{k} = sprintf('Lan %d: (%.0f; %.0f) mm', k, q.xe, q.ye);
        else
            h(k) = plot(ax1, q.px, q.py, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1);
            plot(ax1, q.xe, q.ye, 'x', 'Color', [0.6 0.6 0.6], 'MarkerSize', 10, 'LineWidth', 2);
            lab{k} = sprintf('Lan %d: %s', k, q.note);
        end
    end
    ally = [];  for k = find(ok), ally = [ally; A(k).py; A(k).ye]; end %#ok<AGROW>
    yl = [min([ally; -TOL]) - 40, max([ally; TOL]) + 40];
    ylim(ax1, yl);  xlim(ax1, [-100 3500]);
    xlabel(ax1, 'x doc duong co so (mm)');  ylabel(ax1, 'y lech ngang (mm)');
    legend(ax1, h, lab, 'Location', 'eastoutside');
    title(ax1, sprintf('%s - %d lan chay 3 m (toa do: %s)', NAME{m}, n, SRC{m}), 'FontSize', 14);

    ax2 = axes('Position', [0.07 0.07 0.39 0.27]); hold(ax2, 'on'); style_ax(ax2); grid(ax2, 'on');
    for k = find(ok)
        q = A(k);
        plot(ax2, [q.px; q.xe], [q.hd; q.he], '-', 'Color', PAL(k, :), 'LineWidth', 1.2);
    end
    plot(ax2, [0 TARGET], [0 0], 'k-', 'LineWidth', 1.5);
    xlim(ax2, [-100 3200]);
    xlabel(ax2, 'quang duong x (mm)');  ylabel(ax2, 'goc huong (do)');
    title(ax2, 'Goc huong theo quang duong (muc tieu 0 do)', 'FontSize', 12);

    ax3 = axes('Position', [0.56 0.07 0.39 0.27]); hold(ax3, 'on'); style_ax(ax3); grid(ax3, 'on');
    for k = 1:n
        q = A(k);
        if q.ok
            fill(ax3, [k-0.35 k+0.35 k+0.35 k-0.35], [0 0 abs(q.ye) abs(q.ye)], CC{m}, 'EdgeColor', 'none');
            text(ax3, k, abs(q.ye), sprintf('%.0f', abs(q.ye)), 'HorizontalAlignment', 'center', ...
                 'VerticalAlignment', 'bottom', 'FontSize', 10, 'Color', INK2);
        else
            text(ax3, k, 0, 'loi', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
                 'FontSize', 10, 'Color', [0.6 0.6 0.6]);
        end
    end
    v = abs([A(ok).ye]);
    plot(ax3, [0.4 n + 0.6], mean(v) * [1 1], '--', 'Color', CC{m} * 0.8, 'LineWidth', 1.5);
    xlim(ax3, [0.4 n + 0.6]);  ylim(ax3, [0 max(v) * 1.25 + 5]);
    set(ax3, 'XTick', 1:n);
    xlabel(ax3, 'lan chay');  ylabel(ax3, '|y| cuoi (mm)');
    title(ax3, sprintf('Lech ngang cuoi: TB %.0f mm, RMS lech ngang TB %.0f mm', mean(v), mean([A(ok).rms])), 'FontSize', 12);

    if SAVE_PNG, print(fig, FILE{m}, '-dpng', '-r130'); end
end

%% ---------- hinh so sanh: lan tot nhat cua moi bai ----------
fig = figure('Color', 'w', 'Position', [40 40 1600 880], 'Name', '4 - So sanh lan tot nhat');
ax1 = axes('Position', [0.085 0.54 0.865 0.39]); hold(ax1, 'on'); style_ax(ax1); grid(ax1, 'on');
fill(ax1, [0 TARGET TARGET 0], [-TOL -TOL TOL TOL], C_TOL, 'EdgeColor', 'none');
plot(ax1, [0 TARGET], [0 0], 'k-', 'LineWidth', 2);
plot(ax1, TARGET, 0, 'kp', 'MarkerSize', 14, 'MarkerFaceColor', [1 0.84 0]);
h = zeros(1, 3);  lab = cell(1, 3);  ally = [];
for m = 1:3
    q = Q{m}(best(m));
    h(m) = plot(ax1, [q.px; q.xe], [q.py; q.ye], '.-', 'Color', CC{m}, 'LineWidth', 1.6, 'MarkerSize', 9);
    plot(ax1, q.xe, q.ye, 'o', 'MarkerFaceColor', CC{m}, 'MarkerEdgeColor', 'w', 'MarkerSize', 10);
    lab{m} = sprintf('%s, lan %d: (%.0f; %.0f) mm, RMS %.0f mm', NAME{m}, best(m), q.xe, q.ye, q.rms);
    ally = [ally; q.py; q.ye]; %#ok<AGROW>
end
ylim(ax1, [min([ally; -TOL]) - 40, max([ally; TOL]) + 40]);  xlim(ax1, [-100 3500]);
xlabel(ax1, 'x doc duong co so (mm)');  ylabel(ax1, 'y lech ngang (mm)');
legend(ax1, h, lab, 'Location', 'southwest');
title(ax1, 'Lan chay tot nhat cua moi bai (nho nhat: RMS lech ngang + khoang cach toi dich)', 'FontSize', 14);

ax2 = axes('Position', [0.07 0.07 0.39 0.27]); hold(ax2, 'on'); style_ax(ax2); grid(ax2, 'on');
for m = 1:3
    q = Q{m}(best(m));
    plot(ax2, [q.px; q.xe], [q.hd; q.he], '-', 'Color', CC{m}, 'LineWidth', 1.6);
end
plot(ax2, [0 TARGET], [0 0], 'k-', 'LineWidth', 1.5);
xlim(ax2, [-100 3200]);
xlabel(ax2, 'quang duong x (mm)');  ylabel(ax2, 'goc huong (do)');
title(ax2, 'Goc huong theo quang duong', 'FontSize', 12);

ax3 = axes('Position', [0.56 0.07 0.39 0.27]); hold(ax3, 'on'); style_ax(ax3); grid(ax3, 'on');
lbl = {'|y| cuoi', 'RMS lech ngang', 'cach dich'};
for i = 1:3
    for m = 1:3
        q = Q{m}(best(m));
        v = [abs(q.ye), q.rms, q.dist];
        x = i + (m - 2) * 0.27;
        fill(ax3, [x-0.12 x+0.12 x+0.12 x-0.12], [0 0 v(i) v(i)], CC{m}, 'EdgeColor', 'none');
        text(ax3, x, v(i), sprintf('%.0f', v(i)), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
             'FontSize', 10, 'Color', INK2);
    end
end
set(ax3, 'XTick', 1:3, 'XTickLabel', lbl);  xlim(ax3, [0.5 3.5]);
ylabel(ax3, 'mm');
title(ax3, 'Chi so cua 3 lan tot nhat (cam: encoder, xanh duong: IMU, xanh la: EKF)', 'FontSize', 12);

if SAVE_PNG, print(fig, 'slide_so_sanh_tot_nhat.png', '-dpng', '-r130'); end
