%% compare_3_methods.m
% So sanh tung lan chay cua 3 cach chay thang 3 m, tren nen duong co so tu
% (0; 0) toi (3000; 0):
%   1. Thuan encoder        (log_enc_run_...)  - toa do odometry cua robot
%   2. Vong IMU giu huong   (log_imu_kp3_...)  - toa do odometry cua robot
%   3. EKF bam duong        (log_ekf_...)      - toa do CAMERA (vi tri that)
%
% Bo ba thu j: lan j thuan encoder + lan j IMU + lan chay EKF binh thuong thu j
% (bo qua cac lan EKF loi camera). Moi bo ba 1 hinh:
%   tren       : quy dao 3 cach chay, duong co so 3 m, hanh lang, dich, lech cuoi
%   duoi trai  : lech ngang y theo quang duong x
%   duoi giua  : goc huong theo quang duong x
%   duoi phai  : bang chi so + muc cai thien cua EKF
% Het cac bo ba: hinh tong hop + bang trung binh, Welch t-test (EKF vs IMU).
%
% Can cung thu muc: fwmr_parse_log.m va 3 file log.

%clear; close all; clc;

%% ===================== CAU HINH =====================
LOG_ENC   = 'log_zigzag_mode0_19buoc_20102026.txt';
LOG_IMU   = 'log_zigzag_mode1_20102026_zigzag.txt';
LOG_EKF   = 'log_zigzag_mode2_20102026.txt';
NAME      = {'Thuan encoder', 'Vong IMU (KP=3)', 'EKF (camera)'};
EKF_RUNS  = [1 2 3];        % [] = tu chon cac lan EKF binh thuong ; vd [1 2 3 4 5 7 8]
TRIPLETS  = [];        % [] = tat ca bo ba ; vd [3] = chi bo ba 3
TARGET_MM = 3000;
TOL_MM    = 50;
SAVE_PNG  = true;      % so_sanh_3_lan_j.png + so_sanh_3_tong_hop.png
PT_SIZE   = 4;

CC     = {[0.922 0.408 0.204], [0.165 0.471 0.839], [0.000 0.600 0.330]};   % cam, xanh duong, xanh la
C_BASE = [0 0 0];
C_TOL  = [0.106 0.686 0.478];
C_GOOD = [0.00 0.55 0.25];
C_BAD  = [0.80 0.15 0.15];
INK    = [0 0 0];
INK2   = [0.32 0.32 0.31];
GRIDC  = [0.88 0.88 0.87];
LEG_INK = [1 1 1];  LEG_BG = [0.20 0.20 0.20];


%% ===================== DOC LOG =====================
RE = fwmr_parse_log(LOG_ENC);
RI = fwmr_parse_log(LOG_IMU);
RK = fwmr_parse_log(LOG_EKF);

% Lan EKF binh thuong: co khung camera luc dung, khong mat camera > 1 s,
% encoder khong cho thay xe chay qua 1.3 x quang duong
if isempty(EKF_RUNS)
    ok = true(1, numel(RK));
    for k = 1:numel(RK)
        r = RK(k);
        dist_enc = mean(r.cnt(end, :)) * (2 * pi * 34) / 1496;
        ok(k) = ~isnan(r.xc_end) && ~(r.cam_gap_ms > 1000) && dist_enc <= 1.3 * TARGET_MM;
    end
    EKF_RUNS = find(ok);
    if any(~ok)
        fprintf('Bo qua lan EKF loi camera: %s\n', mat2str(find(~ok)));
    end
end

nT = min([numel(RE), numel(RI), numel(EKF_RUNS)]);
if isempty(TRIPLETS), TRIPLETS = 1:nT; end
TRIPLETS = TRIPLETS(TRIPLETS <= nT);

% Chuan hoa: Q{m}(j) co px, py, hd (goc), t_ms, px_end, py_end, hd_end, mv (dang chay)
Q = cell(1, 3);
q = struct();
for j = 1:nT
    for m = 1:2
        if m == 1, r = RE(j); else, r = RI(j); end
        q.t_ms = r.t_ms;  q.px = r.xi;  q.py = r.yi;  q.hd = r.yaw;
        q.px_end = r.xi_end;  q.py_end = r.yi_end;  q.hd_end = r.yaw_end;
        q.mv = r.t_ms <= r.t_hit;
        q.run = j;
        Q{m}(j) = q;
    end
    r = RK(EKF_RUNS(j));
    sel = [1; find(r.camn > 0)];                 % chi lay dong co khung camera moi
    sel = unique(sel);
    q.t_ms = r.t_ms(sel);  q.px = r.xc(sel);  q.py = r.yc(sel);  q.hd = r.yaw(sel);
    q.px_end = r.xc_end;  q.py_end = r.yc_end;  q.hd_end = r.thc_end;
    q.mv = r.cmd(sel) > 0;
    q.run = EKF_RUNS(j);
    Q{3}(j) = q;
end

%% ===================== CHI SO =====================
M_NAME = {'|y| cuoi (mm)', 'Trong hanh lang (%)', 'RMS lech ngang (mm)', ...
          '|goc| cuoi (do)', '|goc| lon nhat (do)', 'Sai so toi dich (mm)'};
M_FMT  = {'%.0f', '%.0f', '%.0f', '%.1f', '%.1f', '%.0f'};
nM = numel(M_NAME);
MET = NaN(3, nT, nM);
for m = 1:3
    for j = 1:nT
        q = Q{m}(j);
        xs = [q.px(q.mv); q.px_end];  ys = [q.py(q.mv); q.py_end];
        [xs, o] = sort(xs);  ys = ys(o);
        [xs, iu] = unique(xs);  ys = ys(iu);
        rms_y = sqrt(trapz(xs, ys.^2) / max(xs(end) - xs(1), 1));
        dx = diff(xs);  ym = (ys(1:end-1) + ys(2:end)) / 2;
        in_tol = 100 * sum(dx(abs(ym) <= TOL_MM)) / max(sum(dx), 1);
        MET(m, j, :) = [abs(q.py_end), in_tol, rms_y, abs(q.hd_end), ...
                        max(abs([q.hd(q.mv); q.hd_end])), hypot(q.px_end - TARGET_MM, q.py_end)];
    end
end


%% ===================== GIOI HAN TRUC CHUNG =====================
ally = 0; allh = 0;
for m = 1:3
    for j = TRIPLETS
        ally = [ally; Q{m}(j).py; Q{m}(j).py_end]; %#ok<AGROW>
        allh = [allh; Q{m}(j).hd; Q{m}(j).hd_end]; %#ok<AGROW>
    end
end
XL  = [-200, TARGET_MM + 950];
YL  = [min(ally) - 120, max(max(ally), 0) + 170];
YL2 = [min(ally) - 40,  max(max(ally), 0) + 60];
HL  = [min(allh) - 2,   max(max(allh), 0) + 2];


style_ax = @(a) set(a, 'Color', 'w', 'XColor', INK2, 'YColor', INK2, 'GridColor', GRIDC, ...
                    'GridAlpha', 1, 'FontSize', 9, 'Box', 'off');

fig = figure('Name', 'FWMR - so sanh 3 cach chay', 'Color', 'w', 'Position', [30 50 1550 860]);

%% ===================== VONG LAP TUNG BO BA =====================
for j = TRIPLETS
        clf(fig);
    A = {Q{1}(j), Q{2}(j), Q{3}(j)};
    labs = {sprintf('%s (lan %d)', NAME{1}, j), sprintf('%s (lan %d)', NAME{2}, j), ...
            sprintf('%s (lan %d)', NAME{3}, A{3}.run)};

    % ---------- Tren: quy dao ----------
    ax = axes('Parent', fig, 'Position', [0.045 0.48 0.94 0.43]); hold(ax, 'on');
    style_ax(ax); grid(ax, 'on'); axis(ax, 'equal'); xlim(ax, XL); ylim(ax, YL);
    xlabel(ax, 'x (mm)', 'Color', INK); ylabel(ax, 'y (mm)', 'Color', INK);
    patch('Parent', ax, 'XData', [0 TARGET_MM TARGET_MM 0], 'YData', [-TOL_MM -TOL_MM TOL_MM TOL_MM], ...
          'FaceColor', C_TOL, 'FaceAlpha', 0.18, 'EdgeColor', C_TOL, 'LineStyle', ':');
    h_base = plot(ax, [0 TARGET_MM], [0 0], '-', 'Color', C_BASE, 'LineWidth', 3);
    plot(ax, [TARGET_MM TARGET_MM], YL, ':', 'Color', [0.55 0.55 0.55], 'LineWidth', 1);
    text(TARGET_MM / 2, TOL_MM + 25, sprintf('DUONG CO SO %.0f m  (hanh lang \\pm%d mm)', ...
         TARGET_MM / 1000, TOL_MM), 'Color', INK, 'FontSize', 10, 'FontWeight', 'bold', ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    plot(ax, TARGET_MM, 0, 'p', 'MarkerSize', 16, 'MarkerFaceColor', [1 0.85 0], 'MarkerEdgeColor', INK);
    text(TARGET_MM + 30, 40, sprintf('dich (%d; 0)', TARGET_MM), 'Color', INK, 'FontSize', 9, ...
         'VerticalAlignment', 'bottom');
    plot(ax, 0, 0, 'o', 'MarkerSize', 8, 'MarkerFaceColor', INK, 'MarkerEdgeColor', 'w');
    text(-30, 40, 'xuat phat', 'Color', INK, 'FontSize', 9, 'HorizontalAlignment', 'right', ...
         'VerticalAlignment', 'bottom');
    title(ax, sprintf('Bo %d/%d:  %s  vs  %s  vs  %s', j, nT, labs{1}, labs{2}, labs{3}), ...
          'Color', INK, 'FontWeight', 'bold', 'FontSize', 12);

    h_tr = zeros(1, 3);
    for m = 1:3                                         % ve ngay toan bo duong di (ca doan troi)
        c = CC{m};
        h_tr(m) = plot(ax, A{m}.px, A{m}.py, '-o', 'Color', c, 'LineWidth', 1.5, 'MarkerSize', PT_SIZE, ...
                       'MarkerFaceColor', c, 'MarkerEdgeColor', c);
    end

    % ---------- Diem cuoi + nhan (tach nhan neu gan nhau) ----------
    ye = [A{1}.py_end, A{2}.py_end, A{3}.py_end];
    [~, o] = sort(ye, 'descend');
    ylab = ye;  gap = 0.085 * diff(YL);
    for i = 2:3
        ylab(o(i)) = min(ylab(o(i)), ylab(o(i - 1)) - gap);
    end
    ylab = ylab + max(0, (YL(1) + gap / 2) - min(ylab));
    x_lab = max([A{1}.px_end, A{2}.px_end, A{3}.px_end, TARGET_MM]) + 90;
    for m = 1:3
        r = A{m};  c = CC{m};
        plot(ax, [r.px_end r.px_end], [0 r.py_end], '-', 'Color', c, 'LineWidth', 2.5);
        plot(ax, r.px_end, r.py_end, 'o', 'MarkerSize', 10, 'MarkerFaceColor', c, ...
             'MarkerEdgeColor', 'w', 'LineWidth', 2);
        text(x_lab, ylab(m), sprintf('(%.0f; %.0f)  \\Deltay=%.0f mm, %.1f^o', ...
             r.px_end, r.py_end, r.py_end, r.hd_end), 'Color', c * 0.75, 'FontSize', 9, ...
             'FontWeight', 'bold', 'VerticalAlignment', 'middle');
    end
    lg = legend(ax, [h_base h_tr], [{sprintf('Duong co so %.0f m', TARGET_MM / 1000)}, labs], ...
                'Location', 'southwest');
    set(lg, 'TextColor', LEG_INK, 'Color', LEG_BG, 'EdgeColor', LEG_BG);

    % ---------- Duoi trai: lech ngang ----------
    ax2 = axes('Parent', fig, 'Position', [0.045 0.07 0.27 0.30]); hold(ax2, 'on');
    style_ax(ax2); grid(ax2, 'on');
    patch('Parent', ax2, 'XData', [0 TARGET_MM TARGET_MM 0], 'YData', [-TOL_MM -TOL_MM TOL_MM TOL_MM], ...
          'FaceColor', C_TOL, 'FaceAlpha', 0.18, 'EdgeColor', 'none');
    plot(ax2, [0 TARGET_MM], [0 0], '-', 'Color', C_BASE, 'LineWidth', 2.5);
    for m = 1:3
        r = A{m};  c = CC{m};
        plot(ax2, [r.px; r.px_end], [r.py; r.py_end], '-o', 'Color', c, 'LineWidth', 1.25, ...
             'MarkerSize', PT_SIZE - 1, 'MarkerFaceColor', c, 'MarkerEdgeColor', c);
    end
    xlim(ax2, [0 TARGET_MM + 200]); ylim(ax2, YL2);
    xlabel(ax2, 'quang duong x (mm)', 'Color', INK); ylabel(ax2, 'lech ngang y (mm)', 'Color', INK);
    title(ax2, 'Lech ngang so voi duong co so', 'Color', INK, 'FontWeight', 'normal');

    % ---------- Duoi giua: goc huong ----------
    ax3 = axes('Parent', fig, 'Position', [0.36 0.07 0.22 0.30]); hold(ax3, 'on');
    style_ax(ax3); grid(ax3, 'on');
    plot(ax3, [0 TARGET_MM], [0 0], '-', 'Color', C_BASE, 'LineWidth', 2.5);
    for m = 1:3
        r = A{m};  c = CC{m};
        plot(ax3, [r.px; r.px_end], [r.hd; r.hd_end], '-o', 'Color', c, 'LineWidth', 1.25, ...
             'MarkerSize', PT_SIZE - 1, 'MarkerFaceColor', c, 'MarkerEdgeColor', c);
    end
    xlim(ax3, [0 TARGET_MM + 200]); ylim(ax3, HL);
    xlabel(ax3, 'quang duong x (mm)', 'Color', INK); ylabel(ax3, 'goc huong (do)', 'Color', INK);
    title(ax3, 'Goc huong (muc tieu 0^o)', 'Color', INK, 'FontWeight', 'normal');

    % ---------- Duoi phai: bang chi so ----------
    ax4 = axes('Parent', fig, 'Position', [0.61 0.08 0.38 0.32]);
    axis(ax4, 'off'); xlim(ax4, [0 1]); ylim(ax4, [0 1]); hold(ax4, 'on');
    text(ax4, 0, 0.98, sprintf('Lech ngang cuoi: %.0f  |  %.0f  |  %.0f mm', ...
         MET(1, j, 1), MET(2, j, 1), MET(3, j, 1)), 'Color', INK, 'FontSize', 13, ...
         'FontWeight', 'bold', 'VerticalAlignment', 'top');
    text(ax4, 0, 0.86, '(thuan encoder  |  vong IMU  |  EKF)', 'Color', INK2, 'FontSize', 9, ...
         'VerticalAlignment', 'top');
    cx = [0 0.33 0.46 0.59 0.73 0.88];
    yy = 0.70;
    hdr = {'Chi so', 'Enc', 'IMU', 'EKF', 'EKF/enc', 'EKF/IMU'};
    hc  = {INK, CC{1} * 0.8, CC{2} * 0.8, CC{3} * 0.8, INK, INK};
    for i = 1:6
        text(ax4, cx(i), yy, hdr{i}, 'Color', hc{i}, 'FontSize', 9, 'FontWeight', 'bold');
    end
    plot(ax4, [0 1], [yy - 0.05 yy - 0.05], '-', 'Color', INK2, 'LineWidth', 0.75);
    for i = 1:nM
        yi = yy - 0.04 - 0.10 * i;
        text(ax4, cx(1), yi, M_NAME{i}, 'Color', INK, 'FontSize', 9);
        for m = 1:3
            text(ax4, cx(m + 1), yi, sprintf(M_FMT{i}, MET(m, j, i)), 'Color', CC{m} * 0.8, ...
                 'FontSize', 9, 'FontWeight', 'bold');
        end
        for b = 1:2
            if i == 2                                  % hanh lang: chenh diem %
                v = MET(3, j, i) - MET(b, j, i);
            else                                       % % giam so voi phuong phap cu
                v = 100 * (MET(b, j, i) - MET(3, j, i)) / max(MET(b, j, i), 1e-9);
            end
            if v >= 0, ci = C_GOOD; else, ci = C_BAD; end
            if i == 2
                s = sprintf('%+.0f diem', v);
            elseif v >= 0
                s = sprintf('-%.0f%%', v);
            else
                s = sprintf('+%.0f%%', -v);
            end
            text(ax4, cx(4 + b), yi, s, 'Color', ci, 'FontSize', 9, 'FontWeight', 'bold');
        end
    end
    text(ax4, 0, -0.04, {'EKF/enc, EKF/IMU: muc thay doi cua EKF, dau (-) = giam sai so. Hanh lang: chenh diem %.', ...
         'Enc, IMU: toa do odometry cua robot (chua co camera). EKF: toa do camera (vi tri that).'}, ...
         'Color', INK2, 'FontSize', 8, 'VerticalAlignment', 'top', 'Clipping', 'off');
    drawnow;

    if SAVE_PNG
        try, print(fig, sprintf('so_sanh_3_lan_%d.png', j), '-dpng', '-r130');
        catch err, warning('Khong luu duoc PNG: %s', err.message); end
    end

    fprintf('Bo %d (EKF lan %d): |y| cuoi %.0f / %.0f / %.0f mm, RMS %.0f / %.0f / %.0f mm\n', j, A{3}.run, ...
            MET(1, j, 1), MET(2, j, 1), MET(3, j, 1), MET(1, j, 3), MET(2, j, 3), MET(3, j, 3));

end

%% ===================== TONG HOP =====================
fprintf('\n=============== TONG HOP %d BO BA ===============\n', numel(TRIPLETS));
fprintf('%-22s %16s %16s %16s %9s\n', 'Chi so', NAME{1}, NAME{2}, NAME{3}, 'p EKF/IMU');
pv = NaN(1, nM);
for i = 1:nM
    v = cell(1, 3);
    for m = 1:3, v{m} = squeeze(MET(m, TRIPLETS, i)); v{m} = v{m}(:); end
    a = v{2}; b = v{3};
    ma = mean(a); mb = mean(b); sa = std(a); sb = std(b); na = numel(a); nb = numel(b);
    if na > 1 && nb > 1 && (sa > 0 || sb > 0)
        se2 = sa^2 / na + sb^2 / nb;
        tW  = (ma - mb) / sqrt(se2);
        df  = se2^2 / ((sa^2 / na)^2 / (na - 1) + (sb^2 / nb)^2 / (nb - 1));
        pv(i) = betainc(df / (df + tW^2), df / 2, 0.5);
    end
    fprintf('%-22s %8.1f +-%6.1f %8.1f +-%6.1f %8.1f +-%6.1f %9.4f\n', M_NAME{i}, ...
            mean(v{1}), std(v{1}), mean(v{2}), std(v{2}), mean(v{3}), std(v{3}), pv(i));
end
fprintf('(Enc, IMU: odometry ; EKF: camera ; p = Welch t-test giua EKF va vong IMU)\n\n');

if ~ishandle(fig) || numel(TRIPLETS) < 2, return; end

fig2 = figure('Name', 'FWMR - tong hop 3 cach chay', 'Color', 'w', 'Position', [60 80 1450 520]);
SHOW = [1 3 5];
for s = 1:numel(SHOW)
    i = SHOW(s);
    ax = axes('Parent', fig2, 'Position', [0.06 + (s - 1) * 0.33, 0.13, 0.26, 0.68]); hold(ax, 'on');
    style_ax(ax); grid(ax, 'on');
    V = zeros(numel(TRIPLETS), 3);
    for m = 1:3, V(:, m) = squeeze(MET(m, TRIPLETS, i)); end
    hb = bar(ax, 1:numel(TRIPLETS), V, 'grouped');
    for m = 1:3
        set(hb(m), 'FaceColor', CC{m}, 'EdgeColor', 'none');
        plot(ax, [0.5 numel(TRIPLETS) + 0.5], mean(V(:, m)) * [1 1], '--', 'Color', CC{m} * 0.8, 'LineWidth', 1.5);
    end
    set(ax, 'XTick', 1:numel(TRIPLETS), 'XTickLabel', arrayfun(@(k) sprintf('%d', k), TRIPLETS, 'UniformOutput', false));
    xlim(ax, [0.5 numel(TRIPLETS) + 0.5]);
    xlabel(ax, 'bo ba lan chay', 'Color', INK);  ylabel(ax, M_NAME{i}, 'Color', INK);
    title(ax, {M_NAME{i}, sprintf('TB  %.1f  |  %.1f  |  %.1f', mean(V(:, 1)), mean(V(:, 2)), mean(V(:, 3)))}, ...
          'Color', INK, 'FontWeight', 'normal');
    if s == 1
        lg = legend(ax, hb, NAME, 'Location', 'northeast');
        set(lg, 'TextColor', LEG_INK, 'Color', LEG_BG, 'EdgeColor', LEG_BG);
    end
end
annotation(fig2, 'textbox', [0.06 0.0 0.9 0.05], 'String', ...
    'Thuan encoder va vong IMU: toa do odometry cua robot (chua co camera). EKF: toa do camera.', ...
    'EdgeColor', 'none', 'Color', INK2, 'FontSize', 8);
if SAVE_PNG
    try, print(fig2, 'so_sanh_3_tong_hop.png', '-dpng', '-r130'); catch, end
end
