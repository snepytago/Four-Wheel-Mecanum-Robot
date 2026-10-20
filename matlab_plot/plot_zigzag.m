%% plot_zigzag.m
% Bai zigzag (zigzag_test): di W0->W5, quay dau 180, ve W5->W0 (19 buoc). Log cu 9 buoc van doc duoc.
% Moi file log = 1 mode.
%   (a) quy dao tren khung zigzag ly thuyet + hanh lang
%   (b) sai so vi tri tai cac diem dich W1..W5 (di), W4'..W0' (ve) - camera neu co, khong thi uoc luong encoder + IMU / EKF
%   (c) sai so huong sau moi buoc
% Can cung thu muc: fwmr_parse_log.m va file log.

%clear; close all; clc;

LOG_FILE = 'log_zigzag_mode2_20102026.txt';   % V1: 'log_zigzag_mode0_19buoc_20102026.txt' ; V2: 'log_zigzag_mode1_20102026.txt'
SAVE_PNG = true;
CORRIDOR = 100;                 % nua be rong hanh lang (mm)
SHOW_ENC = false;               % true = ve them quy dao encoder thuan (net manh, nhat)
ARROW_L  = 55;                  % kich thuoc mui ten chieu chay (mm), 1 mui ten / doan thang / lan chay

INK2  = [0.32 0.32 0.31];
GRIDC = [0.88 0.88 0.87];
PAL   = [0.000 0.600 0.330; 0.165 0.471 0.839; 0.922 0.408 0.204; 0.910 0.482 0.643; 0.290 0.227 0.655; ...
         0.600 0.450 0.100; 0.200 0.650 0.700; 0.500 0.500 0.500; 0.700 0.200 0.300];
style_ax = @(a) set(a, 'Color', 'w', 'XColor', INK2, 'YColor', INK2, 'GridColor', GRIDC, ...
                    'GridAlpha', 1, 'FontSize', 9, 'Box', 'off');

W  = [0 0; 2400 0; 2400 600; 0 600; 0 1200; 2400 1200];     % W0..W5 (mm)
% Buoc giong firmware: [loai (0 thang, 1 quay), gia tri (thang: 0 dai 1 ngan ; quay: do)]
MV = [0 0; 1 90; 0 1; 1 90; 0 0; 1 -90; 0 1; 1 -90; 0 0; 1 180; 0 0; 1 90; 0 1; 1 90; 0 0; 1 -90; 0 1; 1 -90; 0 0];
NS = size(MV, 1);
HD = cumsum(MV(:, 1) .* MV(:, 2)).';                        % huong dich sau moi buoc (do, cong don)
KS = find(MV(:, 1) == 0).';                                 % cac buoc thang = diem dich
WL = {'W1', 'W2', 'W3', 'W4', 'W5', 'W4''', 'W3''', 'W2''', 'W1''', 'W0'''};

R = fwmr_parse_log(LOG_FILE);
n = numel(R);

% Dong ZZ:KQ (lan in dau tien cua moi buoc) va mode
KQ = cell(1, n);  MODE = NaN(1, n);
fid = fopen(LOG_FILE, 'r');
ir = 0;  md = NaN;
while true
    ln = fgetl(fid);
    if ~ischar(ln), break; end
    ln = ln(ln >= 32 & ln <= 126);
    p = strfind(ln, 'ZZ:START mode=');
    if ~isempty(p), md = sscanf(ln(p(1) + 14:end), '%d', 1); continue; end
    if ~isempty(strfind(ln, 'ENC,t_ms')), ir = ir + 1; KQ{ir} = NaN(NS, 15); MODE(ir) = md; continue; end
    p = strfind(ln, 'ZZ:KQ k=');
    if ir == 0 || ir > n || isempty(p), continue; end
    ln = ln(p(1):end);
    k = sscanf(ln(9:end), '%d', 1);
    if isempty(k) || k < 1 || k > NS || ~isnan(KQ{ir}(k, 1)), continue; end
    v = NaN(1, 15);
    v(1) = k;
    q = regexp(ln, 'dich=(-?\d+),(-?\d+),(-?\d+)', 'tokens', 'once');      if ~isempty(q), v(2:4)  = str2double(q); end
    q = regexp(ln, '(?:imu|ekf)=(-?\d+),(-?\d+),(-?[\d.]+)', 'tokens', 'once'); if ~isempty(q), v(5:7) = str2double(q); end
    q = regexp(ln, ' cam=(-?\d+),(-?\d+),(-?[\d.]+)', 'tokens', 'once');    if ~isempty(q), v(8:10) = str2double(q); end
    q = regexp(ln, 'enc=(-?\d+),(-?\d+),(-?[\d.]+)', 'tokens', 'once');     if ~isempty(q), v(11:13) = str2double(q); end
    q = regexp(ln, 't_bd_ms=(\d+) t_cat_ms=(\d+)', 'tokens', 'once');      if ~isempty(q), v(14:15) = str2double(q); end
    KQ{ir}(k, :) = v;
end
fclose(fid);
md = MODE(find(~isnan(MODE), 1));  if isempty(md), md = NaN; end
TEN = {'thuan encoder', 'encoder + IMU', 'EKF + camera'};
ten = 'mode ?';  if ~isnan(md) && md >= 0 && md <= 2, ten = sprintf('mode %d - %s', md, TEN{md + 1}); end

% Bang sai so: uu tien camera, khong co thi lay uoc luong encoder + IMU (mode 0/1) hoac EKF (mode 2)
fprintf('\n%s (%s)\n%4s %5s %10s %10s %10s %9s  %s\n', LOG_FILE, ten, 'Lan', 'Buoc', 'dx (mm)', 'dy (mm)', ...
        '|d| (mm)', 'dh (do)', 'nguon');
EP = NaN(n, numel(KS));  EH = NaN(n, NS);
for r = 1:n
    A = KQ{r};
    for k = 1:NS
        if isnan(A(k, 1)), continue; end
        if ~isnan(A(k, 8)), P = A(k, 8:10); src = 'camera'; else, P = A(k, 5:7); src = 'xe'; end
        d = P(1:2) - A(k, 2:3);
        dh = mod(P(3) - HD(k) + 180, 360) - 180;
        EH(r, k) = dh;
        j = find(KS == k);
        if ~isempty(j)
            EP(r, j) = norm(d);
            fprintf('%4d %5s %10.0f %10.0f %10.0f %9.1f  %s\n', r, WL{j}, d, norm(d), dh, src);
        end
    end
end

fig = figure('Color', 'w', 'Position', [40 40 1500 800]);

ax1 = subplot(2, 3, [1 2 4 5]); hold(ax1, 'on'); style_ax(ax1); grid(ax1, 'on');
for i = 1:5
    a = W(i, :);  b = W(i + 1, :);  t = (b - a) / norm(b - a);  m = [-t(2) t(1)] * CORRIDOR;
    fill(ax1, [a(1) + m(1), b(1) + m(1), b(1) - m(1), a(1) - m(1)], ...
              [a(2) + m(2), b(2) + m(2), b(2) - m(2), a(2) - m(2)], [0.93 0.95 0.93], 'EdgeColor', 'none');
end
plot(ax1, W(:, 1), W(:, 2), 'k-', 'LineWidth', 2.8);
plot(ax1, W(:, 1), W(:, 2), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
for i = 1:6, text(W(i, 1) + 40, W(i, 2) + 60, sprintf('W%d', i - 1), 'Parent', ax1, 'FontSize', 9, 'Color', INK2); end
h = zeros(1, n);  lab = cell(1, n);
for r = 1:n
    c  = PAL(1 + mod(r - 1, size(PAL, 1)), :);
    cl = 0.45 * c + 0.55;                                   % mau nhat cua lan chay
    if SHOW_ENC, plot(ax1, R(r).x, R(r).y, '-', 'Color', cl, 'LineWidth', 1.0); end
    m = R(r).camn > 0;
    if any(m)                                                % co camera: camera la duong chinh
        plot(ax1, R(r).xi, R(r).yi, '-', 'Color', cl, 'LineWidth', 1.2);
        px = R(r).xc(m);  py = R(r).yc(m);
    else
        px = R(r).xi;  py = R(r).yi;
    end
    if any(m), pt = R(r).t_ms(m); else, pt = R(r).t_ms; end
    ok = ~isnan(px) & ~isnan(py);  px = px(ok);  py = py(ok);  pt = pt(ok);
    h(r) = plot(ax1, px, py, '-', 'Color', c, 'LineWidth', 2.4);
    % Mui ten chieu chay: 1 mui ten giua moi doan thang, lech vi tri giua cac lan chay de khong chong nhau
    A = KQ{r};
    f = 0.2 + 0.12 * mod(r - 1, 3);                         % chieu ve lay doi xung nen khong trung chieu di
    for k = KS
        if isnan(A(k, 14)), continue; end
        mk = find(pt >= A(k, 14) & pt <= A(k, 14) + A(k, 15));
        if numel(mk) < 3, continue; end
        sx = px(mk);  sy = py(mk);
        sl = [0; cumsum(hypot(diff(sx), diff(sy)))];
        if sl(end) < 3 * ARROW_L, continue; end
        i = find(sl >= f * sl(end), 1);  i = min(max(i, 2), numel(sx) - 1);
        d = [sx(i + 1) - sx(i - 1), sy(i + 1) - sy(i - 1)];
        if norm(d) < 1, continue; end
        d = d / norm(d);  q = [-d(2) d(1)];  p0 = [sx(i) sy(i)];
        tip = p0 + d * ARROW_L * 0.6;  b1 = p0 - d * ARROW_L * 0.4 + q * ARROW_L * 0.35;  b2 = p0 - d * ARROW_L * 0.4 - q * ARROW_L * 0.35;
        fill(ax1, [tip(1) b1(1) b2(1)], [tip(2) b1(2) b2(2)], c, 'EdgeColor', c);
    end
    A = KQ{r};
    if ~all(isnan(A(:, 8))), plot(ax1, A(:, 8), A(:, 9), 'o', 'MarkerFaceColor', c, 'MarkerEdgeColor', 'w', 'MarkerSize', 7); end
    lab{r} = sprintf('lan %d', r);
end
ax_x = [W(:, 1); vertcat(R.xi)];  ax_y = [W(:, 2); vertcat(R.yi)];
if SHOW_ENC, ax_x = [ax_x; vertcat(R.x)];  ax_y = [ax_y; vertcat(R.y)]; end
for r = 1:n, if ~isempty(R(r).xc), m = R(r).camn > 0; ax_x = [ax_x; R(r).xc(m)]; ax_y = [ax_y; R(r).yc(m)]; end, end
axis(ax1, 'equal');
xlim(ax1, [min(ax_x) - 200, max(ax_x) + 200]);  ylim(ax1, [min(ax_y) - 200, max(ax_y) + 500]);
xlabel(ax1, 'x (mm, huong xuat phat)');  ylabel(ax1, 'y (mm, sang trai)');
if md == 2, ten_uoc = 'EKF'; else, ten_uoc = 'encoder + goc IMU'; end
if any(vertcat(R.camn) > 0)
    title(ax1, sprintf('(a) %s: camera (dam), %s (nhat), mui ten = chieu chay', ten, ten_uoc), 'FontWeight', 'normal');
else
    title(ax1, sprintf('(a) %s: %s, mui ten = chieu chay', ten, ten_uoc), 'FontWeight', 'normal');
end
if n > 0, legend(ax1, h, lab, 'Location', 'north', 'Orientation', 'horizontal'); end

ax2 = subplot(2, 3, 3); hold(ax2, 'on'); style_ax(ax2); grid(ax2, 'on');
for r = 1:n
    c = PAL(1 + mod(r - 1, size(PAL, 1)), :);
    plot(ax2, (1:numel(KS)) + (r - (n + 1) / 2) * 0.08, EP(r, :), 'o-', 'Color', c, 'MarkerFaceColor', c, 'MarkerSize', 5);
end
plot(ax2, [5.5 5.5], ylim(ax2), 'k:');
set(ax2, 'XTick', 1:numel(KS), 'XTickLabel', WL);  xlim(ax2, [0.5 numel(KS) + 0.5]);
ylabel(ax2, 'sai so vi tri (mm)');
title(ax2, '(b) Sai so tai diem dich (trai: di, phai: ve)', 'FontWeight', 'normal');

ax3 = subplot(2, 3, 6); hold(ax3, 'on'); style_ax(ax3); grid(ax3, 'on');
for r = 1:n
    c = PAL(1 + mod(r - 1, size(PAL, 1)), :);
    plot(ax3, (1:NS) + (r - (n + 1) / 2) * 0.06, EH(r, :), 'o-', 'Color', c, 'MarkerFaceColor', c, 'MarkerSize', 4);
end
plot(ax3, [0.5 NS + 0.5], [0 0], 'k-');  plot(ax3, [9.5 9.5], ylim(ax3), 'k:');
set(ax3, 'XTick', 1:2:NS);  xlim(ax3, [0.5 NS + 0.5]);
xlabel(ax3, 'buoc (chan = quay, 10 = quay dau)');  ylabel(ax3, 'sai so huong (do)');
title(ax3, '(c) Sai so huong sau moi buoc', 'FontWeight', 'normal');

if SAVE_PNG
    [~, base] = fileparts(LOG_FILE);
    try, print(fig, [base '_zigzag.png'], '-dpng', '-r130'); catch, end
end
