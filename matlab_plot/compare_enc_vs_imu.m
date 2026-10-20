%% compare_enc_vs_imu.m
% So sanh tung cap lan chay: lan k thuan encoder  vs  lan k co vong IMU giu
% huong (k = 1..8), tren nen duong co so thang 3 m tu (0; 0) toi (3000; 0).
%
% Moi cap 1 hinh:
%   tren       : quy dao 2 cach chay (cham = diem log 100 ms), duong co so 3 m,
%                hanh lang +-TOL_MM, diem dich, do lech ngang cuoi
%   duoi trai  : lech ngang y theo quang duong x
%   duoi giua  : goc huong theo quang duong x
%   duoi phai  : bang chi so + % cai thien
% Het 8 cap: 1 hinh tong hop + bang trung binh, do lech chuan, Welch t-test
% in ra Command Window.
%
% Can cung thu muc: fwmr_parse_log.m va 2 file log.
% Toa do la odometry robot tu bao (khong phai do bang thuoc).

%clear; close all; clc;

%% ===================== CAU HINH =====================
LOG_ENC   = 'log_enc_run_06102026.txt';   % thuan encoder (ENC_RUN_IMU_HOLD = 0)
LOG_IMU   = 'log_imu_kp3_07102026.txt';   % co vong IMU   (ENC_RUN_IMU_HOLD = 1)
NAME_ENC  = 'Thuan encoder';
NAME_IMU  = 'Vong IMU (KP=3)';
PAIRS     = [];        % [] = tat ca cap (1..8); vd [3] = chi cap 3; [1 5] = cap 1 va 5
POSE_SRC  = 'imu';     % 'imu' = quang duong encoder + huong IMU (cong bang cho ca 2)
                       % 'enc' = thuan encoder (x, y, theta encoder)
TARGET_MM = 3000;      % duong co so
TOL_MM    = 50;        % hanh lang cho phep quanh duong co so (+- mm)
SAVE_PNG  = true;      % luu moi cap thanh so_sanh_lan_k.png (+ so_sanh_tong_hop.png)
PT_SIZE   = 4;

C_ENC  = [0.922 0.408 0.204];   % cam  - thuan encoder
C_IMU  = [0.165 0.471 0.839];   % xanh - vong IMU
C_BASE = [0 0 0];               % duong co so
C_TOL  = [0.106 0.686 0.478];   % hanh lang
C_GOOD = [0.00 0.55 0.25];
C_BAD  = [0.80 0.15 0.15];
INK    = [0 0 0];               % tieu de, nhan
INK2   = [0.32 0.32 0.31];      % truc
GRIDC  = [0.88 0.88 0.87];
LEG_INK = [1 1 1];              % chu bang chu thich (trang, nen toi)
LEG_BG  = [0.20 0.20 0.20];


%% ===================== DOC LOG =====================
RE = fwmr_parse_log(LOG_ENC);
RI = fwmr_parse_log(LOG_IMU);
nP = min(numel(RE), numel(RI));
if numel(RE) ~= numel(RI)
    warning('So lan chay khac nhau: %s co %d, %s co %d -> so sanh %d cap.', ...
            LOG_ENC, numel(RE), LOG_IMU, numel(RI), nP);
end
if isempty(PAIRS), PAIRS = 1:nP; end
PAIRS = PAIRS(PAIRS <= nP);

% Chuan hoa: P{m}(k) co x, y, hd (goc huong), t, x_end, y_end, hd_end, t_hit
% m = 1: thuan encoder, m = 2: vong IMU
P = {RE, RI};
Q = cell(1, 2);
for m = 1:2
    for k = 1:numel(P{m})
        r = P{m}(k);
        use_imu = strcmp(POSE_SRC, 'imu') && ~isempty(r.yaw) && ~any(isnan(r.yaw));
        if use_imu
            r.px = r.xi;  r.py = r.yi;  r.hd = r.yaw;
            r.px_end = r.xi_end;  r.py_end = r.yi_end;  r.hd_end = r.yaw_end;
        else
            r.px = r.x;   r.py = r.y;   r.hd = r.th;
            r.px_end = r.x_end;   r.py_end = r.y_end;   r.hd_end = r.th_end;
        end
        Q{m}(k) = r; %#ok<SAGROW>
    end
end
if strcmp(POSE_SRC, 'imu')
    src_txt = 'quang duong encoder + huong IMU';
else
    src_txt = 'thuan encoder';
end

%% ===================== CHI SO MOI LAN CHAY =====================
% 1 |y cuoi|        lech ngang luc dung han (mm)
% 2 Trong hanh lang % quang duong (tu xuat phat toi cham moc) co |y| <= TOL_MM
%                  (cang LON cang tot; cai thien tinh bang diem %)
% 3 RMS lech ngang  tich phan theo quang duong x, tu xuat phat toi cham moc (mm)
% 4 |goc cuoi|      (do)
% 5 |goc| lon nhat  (do)
% 6 Sai so toi dich khoang cach tu diem cuoi toi (TARGET_MM; 0) (mm)
M_NAME = {'|y| cuoi (mm)', 'Trong hanh lang (%)', 'RMS lech ngang (mm)', ...
          '|goc| cuoi (do)', '|goc| lon nhat (do)', 'Sai so toi dich (mm)'};
M_FMT  = {'%.0f', '%.0f', '%.0f', '%.1f', '%.1f', '%.0f'};
nM = numel(M_NAME);
MET = NaN(2, nP, nM);
for m = 1:2
    for k = 1:nP
        r = Q{m}(k);
        mv = true(size(r.t_ms));
        if ~isnan(r.t_hit), mv = r.t_ms <= r.t_hit; end
        xs = [r.px(mv); r.px_end];  ys = [r.py(mv); r.py_end];
        [xs, o] = sort(xs);  ys = ys(o);
        rms_y = sqrt(trapz(xs, ys.^2) / max(xs(end) - xs(1), 1));
        dx = diff(xs);  ym = (ys(1:end-1) + ys(2:end)) / 2;
        in_tol = 100 * sum(dx(abs(ym) <= TOL_MM)) / max(sum(dx), 1);
        MET(m, k, :) = [abs(r.py_end), in_tol, rms_y, ...
                        abs(r.hd_end), max(abs([r.hd; r.hd_end])), ...
                        hypot(r.px_end - TARGET_MM, r.py_end)];
    end
end
IMP = 100 * (MET(1, :, :) - MET(2, :, :)) ./ MET(1, :, :);   % % cai thien, > 0 = tot hon
IMP(1, :, 2) = MET(2, :, 2) - MET(1, :, 2);                    % hanh lang: chenh diem %

%% ===================== GIOI HAN TRUC CHUNG (de 8 hinh cung ti le) =====================
ally = 0; allh = 0;
for m = 1:2
    for k = PAIRS
        ally = [ally; Q{m}(k).py; Q{m}(k).py_end]; %#ok<AGROW>
        allh = [allh; Q{m}(k).hd; Q{m}(k).hd_end]; %#ok<AGROW>
    end
end
XL  = [-200, TARGET_MM + 550];
YL  = [min(ally) - 120, max(max(ally), 0) + 170];
YL2 = [min(ally) - 40,  max(max(ally), 0) + 60];
HL  = [min(allh) - 2,   max(max(allh), 0) + 2];


CC   = {C_ENC, C_IMU};
NAME = {NAME_ENC, NAME_IMU};
style_ax = @(a) set(a, 'Color', 'w', 'XColor', INK2, 'YColor', INK2, 'GridColor', GRIDC, ...
                    'GridAlpha', 1, 'FontSize', 9, 'Box', 'off');

fig = figure('Name', 'FWMR - so sanh thuan encoder vs vong IMU', 'Color', 'w', ...
             'Position', [30 50 1500 830]);

%% ===================== VONG LAP TUNG CAP =====================
for k = PAIRS
        clf(fig);
    A = {Q{1}(k), Q{2}(k)};

    % ---------- Truc tren: quy dao ----------
    ax = axes('Parent', fig, 'Position', [0.045 0.47 0.94 0.44]); hold(ax, 'on');
    style_ax(ax); grid(ax, 'on'); axis(ax, 'equal'); xlim(ax, XL); ylim(ax, YL);
    xlabel(ax, 'x (mm)', 'Color', INK); ylabel(ax, 'y (mm)', 'Color', INK);

    % Duong co so 3 m: hanh lang + duong dam + dich
    patch('Parent', ax, 'XData', [0 TARGET_MM TARGET_MM 0], 'YData', [-TOL_MM -TOL_MM TOL_MM TOL_MM], ...
          'FaceColor', C_TOL, 'FaceAlpha', 0.18, 'EdgeColor', C_TOL, 'LineStyle', ':');
    h_base = plot(ax, [0 TARGET_MM], [0 0], '-', 'Color', C_BASE, 'LineWidth', 3);
    plot(ax, [TARGET_MM TARGET_MM], YL, ':', 'Color', [0.55 0.55 0.55], 'LineWidth', 1);
    text(TARGET_MM / 2, TOL_MM + 25, sprintf('DUONG CO SO %.0f m  (hanh lang \\pm%d mm)', ...
         TARGET_MM / 1000, TOL_MM), 'Color', INK, 'FontSize', 10, 'FontWeight', 'bold', ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    plot(ax, TARGET_MM, 0, 'p', 'MarkerSize', 16, 'MarkerFaceColor', [1 0.85 0], ...
         'MarkerEdgeColor', INK, 'LineWidth', 1);
    text(TARGET_MM + 30, 40, sprintf('dich (%d; 0)', TARGET_MM), 'Color', INK, 'FontSize', 9, ...
         'VerticalAlignment', 'bottom');
    plot(ax, 0, 0, 'o', 'MarkerSize', 8, 'MarkerFaceColor', INK, 'MarkerEdgeColor', 'w');
    text(-30, 40, 'xuat phat', 'Color', INK, 'FontSize', 9, 'HorizontalAlignment', 'right', ...
         'VerticalAlignment', 'bottom');

    title(ax, sprintf('Cap %d/%d:  %s  vs  %s   -   toa do theo %s', k, nP, NAME_ENC, NAME_IMU, src_txt), ...
          'Color', INK, 'FontWeight', 'bold', 'FontSize', 12);

    h_tr = zeros(1, 2);
    for m = 1:2                                         % ve ngay toan bo duong di (ca doan troi)
        c = CC{m};
        h_tr(m) = plot(ax, A{m}.px, A{m}.py, '-o', 'Color', c, 'LineWidth', 1.5, 'MarkerSize', PT_SIZE, ...
                       'MarkerFaceColor', c, 'MarkerEdgeColor', c);
    end

    % ---------- Diem cuoi + do lech ngang so voi duong co so ----------
    ye = [A{1}.py_end, A{2}.py_end];
    ylab = ye;
    gap = 0.07 * diff(YL);
    if abs(ylab(1) - ylab(2)) < gap                     % 2 nhan qua gan -> tach ra
        s = sign(ylab(2) - ylab(1)); if s == 0, s = 1; end
        mid = mean(ylab);  ylab = mid + [-s s] * gap / 2;
    end
    for m = 1:2
        r = A{m};  c = CC{m};
        plot(ax, [r.px_end r.px_end], [0 r.py_end], '-', 'Color', c, 'LineWidth', 2.5);   % do lech
        plot(ax, r.px_end, r.py_end, 'o', 'MarkerSize', 10, 'MarkerFaceColor', c, ...
             'MarkerEdgeColor', 'w', 'LineWidth', 2);
        text(TARGET_MM + 120, ylab(m), sprintf('(%.0f; %.0f)\n\\Deltay = %.0f mm, goc %.1f^o', ...
             r.px_end, r.py_end, r.py_end, r.hd_end), 'Color', c * 0.75, 'FontSize', 9, ...
             'FontWeight', 'bold', 'VerticalAlignment', 'middle');
    end
    lg = legend(ax, [h_base h_tr], {sprintf('Duong co so %.0f m', TARGET_MM / 1000), ...
                sprintf('%s (lan %d)', NAME_ENC, k), sprintf('%s (lan %d)', NAME_IMU, k)}, ...
                'Location', 'southwest');
    set(lg, 'TextColor', LEG_INK, 'Color', LEG_BG, 'EdgeColor', LEG_BG);

    % ---------- Duoi trai: lech ngang theo quang duong ----------
    ax2 = axes('Parent', fig, 'Position', [0.045 0.07 0.30 0.30]); hold(ax2, 'on');
    style_ax(ax2); grid(ax2, 'on');
    patch('Parent', ax2, 'XData', [0 TARGET_MM TARGET_MM 0], 'YData', [-TOL_MM -TOL_MM TOL_MM TOL_MM], ...
          'FaceColor', C_TOL, 'FaceAlpha', 0.18, 'EdgeColor', 'none');
    plot(ax2, [0 TARGET_MM], [0 0], '-', 'Color', C_BASE, 'LineWidth', 2.5);
    for m = 1:2
        r = A{m};  c = CC{m};
        plot(ax2, [r.px; r.px_end], [r.py; r.py_end], '-o', 'Color', c, 'LineWidth', 1.25, ...
             'MarkerSize', PT_SIZE - 1, 'MarkerFaceColor', c, 'MarkerEdgeColor', c);
    end
    xlim(ax2, [0 TARGET_MM + 100]); ylim(ax2, YL2);
    xlabel(ax2, 'quang duong x (mm)', 'Color', INK); ylabel(ax2, 'lech ngang y (mm)', 'Color', INK);
    title(ax2, 'Lech ngang so voi duong co so', 'Color', INK, 'FontWeight', 'normal');

    % ---------- Duoi giua: goc huong theo quang duong ----------
    ax3 = axes('Parent', fig, 'Position', [0.40 0.07 0.25 0.30]); hold(ax3, 'on');
    style_ax(ax3); grid(ax3, 'on');
    plot(ax3, [0 TARGET_MM], [0 0], '-', 'Color', C_BASE, 'LineWidth', 2.5);
    for m = 1:2
        r = A{m};  c = CC{m};
        plot(ax3, [r.px; r.px_end], [r.hd; r.hd_end], '-o', 'Color', c, 'LineWidth', 1.25, ...
             'MarkerSize', PT_SIZE - 1, 'MarkerFaceColor', c, 'MarkerEdgeColor', c);
    end
    xlim(ax3, [0 TARGET_MM + 100]); ylim(ax3, HL);
    xlabel(ax3, 'quang duong x (mm)', 'Color', INK); ylabel(ax3, 'goc huong (do)', 'Color', INK);
    title(ax3, 'Goc huong (muc tieu 0^o)', 'Color', INK, 'FontWeight', 'normal');

    % ---------- Duoi phai: bang chi so ----------
    ax4 = axes('Parent', fig, 'Position', [0.68 0.08 0.31 0.32]);
    axis(ax4, 'off'); xlim(ax4, [0 1]); ylim(ax4, [0 1]); hold(ax4, 'on');
    imp1 = IMP(1, k, 1);
    if imp1 >= 0
        big = sprintf('Lech ngang cuoi giam %.0f%%', imp1);  cb = C_GOOD;
    else
        big = sprintf('Lech ngang cuoi TANG %.0f%%', -imp1); cb = C_BAD;
    end
    text(ax4, 0, 0.97, big, 'Color', cb, 'FontSize', 14, 'FontWeight', 'bold', 'VerticalAlignment', 'top');
    text(ax4, 0, 0.84, sprintf('%.0f mm  ->  %.0f mm', MET(1, k, 1), MET(2, k, 1)), ...
         'Color', INK, 'FontSize', 10, 'VerticalAlignment', 'top');
    cx = [0 0.47 0.68 0.88];
    yy = 0.68;
    hdr = {'Chi so', 'Thuan enc', 'Vong IMU', 'Cai thien'};
    for j = 1:4
        text(ax4, cx(j), yy, hdr{j}, 'Color', INK, 'FontSize', 9, 'FontWeight', 'bold');
    end
    plot(ax4, [0 1], [yy - 0.05 yy - 0.05], '-', 'Color', INK2, 'LineWidth', 0.75);
    for i = 1:nM
        yi = yy - 0.04 - 0.10 * i;
        text(ax4, cx(1), yi, M_NAME{i}, 'Color', INK, 'FontSize', 9);
        text(ax4, cx(2), yi, sprintf(M_FMT{i}, MET(1, k, i)), 'Color', C_ENC * 0.8, 'FontSize', 9, 'FontWeight', 'bold');
        text(ax4, cx(3), yi, sprintf(M_FMT{i}, MET(2, k, i)), 'Color', C_IMU * 0.8, 'FontSize', 9, 'FontWeight', 'bold');
        v = IMP(1, k, i);
        if v >= 0, ci = C_GOOD; else, ci = C_BAD; end
        if i == 2
            s = sprintf('%+.0f diem', v);
        elseif v >= 0
            s = sprintf('-%.0f%%', v);
        else
            s = sprintf('+%.0f%%', -v);
        end
        text(ax4, cx(4), yi, s, 'Color', ci, 'FontSize', 9, 'FontWeight', 'bold');
    end
    text(ax4, 0, -0.04, {'Cai thien = (enc - IMU) / enc, dau (-) = giam sai so.', ...
         sprintf('Hanh lang: chenh lech diem %% quang duong co |y| <= %d mm.', TOL_MM)}, ...
         'Color', INK2, 'FontSize', 8, 'VerticalAlignment', 'top', 'Clipping', 'off');
    drawnow;

    if SAVE_PNG
        try
            print(fig, sprintf('so_sanh_lan_%d.png', k), '-dpng', '-r130');
        catch err
            warning('Khong luu duoc PNG: %s', err.message);
        end
    end

    fprintf('Cap %d: |y| cuoi %.0f -> %.0f mm (%+.0f%%), |goc| max %.1f -> %.1f do (%+.0f%%)\n', ...
            k, MET(1, k, 1), MET(2, k, 1), -IMP(1, k, 1), MET(1, k, 5), MET(2, k, 5), -IMP(1, k, 5));

end

%% ===================== TONG HOP 8 CAP =====================
fprintf('\n=============== TONG HOP %d CAP (toa do theo %s) ===============\n', numel(PAIRS), src_txt);
fprintf('%-22s %18s %18s %10s %8s\n', 'Chi so', NAME_ENC, NAME_IMU, 'Cai thien', 'p Welch');
pv = NaN(1, nM);
for i = 1:nM
    a = squeeze(MET(1, PAIRS, i));  b = squeeze(MET(2, PAIRS, i));
    a = a(:);  b = b(:);
    ma = mean(a); mb = mean(b); sa = std(a); sb = std(b); na = numel(a); nb = numel(b);
    if na > 1 && nb > 1 && (sa > 0 || sb > 0)
        se2 = sa^2 / na + sb^2 / nb;
        tW  = (ma - mb) / sqrt(se2);
        df  = se2^2 / ((sa^2 / na)^2 / (na - 1) + (sb^2 / nb)^2 / (nb - 1));
        pv(i) = betainc(df / (df + tW^2), df / 2, 0.5);     % 2 phia
    end
    if i == 2
        imp_s = sprintf('%+.0f diem', mb - ma);
    else
        imp_s = sprintf('-%.0f%%', 100 * (ma - mb) / ma);
    end
    fprintf('%-22s %9.1f +- %6.1f %9.1f +- %6.1f %10s %8.4f\n', M_NAME{i}, ma, sa, mb, sb, imp_s, pv(i));
end
fprintf('(gia tri = trung binh +- do lech chuan; p < 0.05 -> khac biet co y nghia thong ke)\n\n');

if ~ishandle(fig) || numel(PAIRS) < 2
    return;
end

fig2 = figure('Name', 'FWMR - tong hop so sanh', 'Color', 'w', 'Position', [60 80 1400 520]);
SHOW = [1 3 5];                                    % |y| cuoi, RMS, |goc| max
for s = 1:numel(SHOW)
    i = SHOW(s);
    ax = axes('Parent', fig2, 'Position', [0.06 + (s - 1) * 0.33, 0.13, 0.26, 0.68]); hold(ax, 'on');
    style_ax(ax); grid(ax, 'on');
    a = squeeze(MET(1, PAIRS, i));  b = squeeze(MET(2, PAIRS, i));
    a = a(:).';  b = b(:).';
    hb = bar(ax, 1:numel(PAIRS), [a; b].', 'grouped');
    set(hb(1), 'FaceColor', C_ENC, 'EdgeColor', 'none');
    set(hb(2), 'FaceColor', C_IMU, 'EdgeColor', 'none');
    plot(ax, [0.5 numel(PAIRS) + 0.5], mean(a) * [1 1], '--', 'Color', C_ENC * 0.8, 'LineWidth', 1.5);
    plot(ax, [0.5 numel(PAIRS) + 0.5], mean(b) * [1 1], '--', 'Color', C_IMU * 0.8, 'LineWidth', 1.5);
    set(ax, 'XTick', 1:numel(PAIRS), 'XTickLabel', arrayfun(@(k) sprintf('%d', k), PAIRS, 'UniformOutput', false));
    xlim(ax, [0.5 numel(PAIRS) + 0.5]);
    xlabel(ax, 'cap lan chay', 'Color', INK);  ylabel(ax, M_NAME{i}, 'Color', INK);
    ptxt = '';
    if ~isnan(pv(i)), ptxt = sprintf(',  p = %.3g', pv(i)); end
    title(ax, {M_NAME{i}, sprintf('TB %.1f -> %.1f  (giam %.0f%%%s)', mean(a), mean(b), ...
          100 * (mean(a) - mean(b)) / mean(a), ptxt)}, 'Color', INK, 'FontWeight', 'normal');
    if s == 1
        lg = legend(ax, hb, {NAME_ENC, NAME_IMU}, 'Location', 'northeast');
        set(lg, 'TextColor', LEG_INK, 'Color', LEG_BG, 'EdgeColor', LEG_BG);
    end
end
if SAVE_PNG
    try, print(fig2, 'so_sanh_tong_hop.png', '-dpng', '-r130'); catch, end
end
