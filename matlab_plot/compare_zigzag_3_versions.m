%% compare_zigzag_3_versions.m
% So sanh bai zigzag 19 buoc giua 3 phien ban, moi phien ban lay 1 lan chay:
%   V1 thuan encoder, V2 encoder + IMU, V3 EKF + camera
%   (a) quy dao tren khung zigzag (mui ten = chieu chay)
%   (b) sai so vi tri tai 10 diem dich (truc log)
%   (c) sai so huong |dh| sau moi buoc (truc log)
%   + bang tong hop in ra Command Window
% Nguon danh gia: camera neu log co camera (V3), khong thi uoc luong encoder + goc IMU (V1, V2).
% Can cung thu muc: fwmr_parse_log.m, fwmr_zigzag_kq.m va 3 file log.

%clear; close all; clc;

LOGS = {'log_zigzag_mode0_19buoc_20102026.txt', 'log_zigzag_mode1_20102026.txt', 'log_zigzag_mode2_20102026.txt'};
TEN  = {'V1 thuan encoder', 'V2 encoder + IMU', 'V3 EKF + camera'};
RUN  = [0 0 0];                 % lan chay dung cho moi phien ban; 0 = tu chon lan tot nhat (sai so TB nho nhat)
SAVE_PNG = true;
CORRIDOR = 100;                 % nua be rong hanh lang (mm)
ARROW_L  = 55;                  % kich thuoc mui ten (mm)

INK2  = [0.32 0.32 0.31];
GRIDC = [0.88 0.88 0.87];
COL   = [0.922 0.408 0.204; 0.165 0.471 0.839; 0.000 0.600 0.330];   % V1 cam, V2 xanh duong, V3 xanh la
style_ax = @(a) set(a, 'Color', 'w', 'XColor', INK2, 'YColor', INK2, 'GridColor', GRIDC, ...
                    'GridAlpha', 1, 'FontSize', 9, 'Box', 'off');

W  = [0 0; 2400 0; 2400 600; 0 600; 0 1200; 2400 1200];
MV = [0 0; 1 90; 0 1; 1 90; 0 0; 1 -90; 0 1; 1 -90; 0 0; 1 180; 0 0; 1 90; 0 1; 1 90; 0 0; 1 -90; 0 1; 1 -90; 0 0];
NS = size(MV, 1);
HD = cumsum(MV(:, 1) .* MV(:, 2)).';
KS = find(MV(:, 1) == 0).';
WL = {'W1', 'W2', 'W3', 'W4', 'W5', 'W4''', 'W3''', 'W2''', 'W1''', 'W0'''};

% ---------- Doc log, tinh sai so, chon lan chay ----------
S = struct('R', {}, 'A', {}, 'ep', {}, 'eh', {}, 'src', {}, 'run', {}, 't', {});
for v = 1:3
    Rall = fwmr_parse_log(LOGS{v});
    [KQ, ~] = fwmr_zigzag_kq(LOGS{v}, NS);
    n = min(numel(Rall), numel(KQ));
    EP = NaN(n, numel(KS));  EH = NaN(n, NS);  SRC = cell(1, n);
    for r = 1:n
        A = KQ{r};
        SRC{r} = 'encoder + goc IMU';
        for k = 1:NS
            if isnan(A(k, 1)), continue; end
            if ~isnan(A(k, 8)), P = A(k, 8:10); SRC{r} = 'camera'; else, P = A(k, 5:7); end
            EH(r, k) = mod(P(3) - HD(k) + 180, 360) - 180;
            j = find(KS == k);
            if ~isempty(j), EP(r, j) = norm(P(1:2) - A(k, 2:3)); end
        end
    end
    if RUN(v) > 0
        rb = RUN(v);
    else
        sc = mean(EP, 2);  sc(any(isnan(EP), 2)) = Inf;
        [~, rb] = min(sc);
    end
    S(v).R = Rall(rb);  S(v).A = KQ{rb};  S(v).ep = EP(rb, :);  S(v).eh = EH(rb, :);
    S(v).src = SRC{rb};  S(v).run = rb;  S(v).t = Rall(rb).t_ms(end) / 1000;
end

% ---------- Bang tong hop ----------
fprintf('\n%-18s %4s %-18s %10s %10s %10s %10s %10s\n', 'Phien ban', 'Lan', 'Nguon', 'TB (mm)', 'Max (mm)', 'W5 (mm)', 'W0 (mm)', '|dh|max');
for v = 1:3
    fprintf('%-18s %4d %-18s %10.0f %10.0f %10.0f %10.0f %10.1f\n', TEN{v}, S(v).run, S(v).src, ...
            mean(S(v).ep), max(S(v).ep), S(v).ep(5), S(v).ep(end), max(abs(S(v).eh)));
end

fig = figure('Color', 'w', 'Position', [40 40 1500 800]);

% ---------- (a) Quy dao ----------
ax1 = subplot(2, 3, [1 2 4 5]); hold(ax1, 'on'); style_ax(ax1); grid(ax1, 'on');
for i = 1:5
    a = W(i, :);  b = W(i + 1, :);  t = (b - a) / norm(b - a);  m = [-t(2) t(1)] * CORRIDOR;
    fill(ax1, [a(1) + m(1), b(1) + m(1), b(1) - m(1), a(1) - m(1)], ...
              [a(2) + m(2), b(2) + m(2), b(2) - m(2), a(2) - m(2)], [0.93 0.95 0.93], 'EdgeColor', 'none');
end
plot(ax1, W(:, 1), W(:, 2), 'k-', 'LineWidth', 2.8);
plot(ax1, W(:, 1), W(:, 2), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
for i = 1:6, text(W(i, 1) + 40, W(i, 2) + 60, sprintf('W%d', i - 1), 'Parent', ax1, 'FontSize', 9, 'Color', INK2); end
h = zeros(1, 3);  lab = cell(1, 3);  ax_x = W(:, 1);  ax_y = W(:, 2);
for v = 1:3
    c = COL(v, :);  R = S(v).R;  A = S(v).A;
    m = R.camn > 0;
    if any(m), px = R.xc(m); py = R.yc(m); pt = R.t_ms(m); else, px = R.xi; py = R.yi; pt = R.t_ms; end
    ok = ~isnan(px) & ~isnan(py);  px = px(ok);  py = py(ok);  pt = pt(ok);
    h(v) = plot(ax1, px, py, '-', 'Color', c, 'LineWidth', 2.4);
    ax_x = [ax_x; px];  ax_y = [ax_y; py];
    f = 0.2 + 0.12 * (v - 1);
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
    lab{v} = sprintf('%s (lan %d, %s)', TEN{v}, S(v).run, S(v).src);
end
axis(ax1, 'equal');
xlim(ax1, [min(ax_x) - 200, max(ax_x) + 200]);  ylim(ax1, [min(ax_y) - 200, max(ax_y) + 600]);
xlabel(ax1, 'x (mm, huong xuat phat)');  ylabel(ax1, 'y (mm, sang trai)');
title(ax1, '(a) Zigzag 19 buoc: di W0 -> W5, quay dau, ve W0 (mui ten = chieu chay)', 'FontWeight', 'normal');
legend(ax1, h, lab, 'Location', 'north');

% ---------- (b) Sai so vi tri ----------
ax2 = subplot(2, 3, 3); hold(ax2, 'on'); style_ax(ax2); grid(ax2, 'on');
for v = 1:3
    plot(ax2, 1:numel(KS), max(S(v).ep, 1), 'o-', 'Color', COL(v, :), 'MarkerFaceColor', COL(v, :), 'LineWidth', 1.6, 'MarkerSize', 5);
end
set(ax2, 'YScale', 'log', 'XTick', 1:numel(KS), 'XTickLabel', WL);  xlim(ax2, [0.5 numel(KS) + 0.5]);
yl = ylim(ax2);  plot(ax2, [5.5 5.5], yl, 'k:');  ylim(ax2, yl);
ylabel(ax2, 'sai so vi tri (mm, log)');
title(ax2, '(b) Sai so tai diem dich (trai: di, phai: ve)', 'FontWeight', 'normal');

% ---------- (c) Sai so huong ----------
ax3 = subplot(2, 3, 6); hold(ax3, 'on'); style_ax(ax3); grid(ax3, 'on');
for v = 1:3
    plot(ax3, 1:NS, max(abs(S(v).eh), 0.1), 'o-', 'Color', COL(v, :), 'MarkerFaceColor', COL(v, :), 'LineWidth', 1.6, 'MarkerSize', 4);
end
set(ax3, 'YScale', 'log', 'XTick', 1:2:NS);  xlim(ax3, [0.5 NS + 0.5]);
yl = ylim(ax3);  plot(ax3, [9.5 9.5], yl, 'k:');  ylim(ax3, yl);
xlabel(ax3, 'buoc (chan = quay, 10 = quay dau)');  ylabel(ax3, '|sai so huong| (do, log)');
title(ax3, '(c) Sai so huong sau moi buoc', 'FontWeight', 'normal');

if SAVE_PNG
    try, print(fig, 'so_sanh_zigzag_3_phien_ban.png', '-dpng', '-r130'); catch, end
end
