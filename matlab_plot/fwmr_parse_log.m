function runs = fwmr_parse_log(filename)
% FWMR_PARSE_LOG  Doc log Serial Monitor cua bai enc_run_test (FWMR).
%
%   runs = fwmr_parse_log('log_enc_run_06102026.txt')
%
% Chap nhan log copy tu Serial Monitor cua esp32Server hoac esp32R (co tien
% to "[STM] ", co hoac khong co timestamp cua Arduino IDE), hoac log UART
% tho. Moi lan chay bat dau bang dong tieu de "ENC,t_ms,...".
%
% Moi phan tu runs(k) gom:
%   t_ms      : thoi gian (ms), cot
%   cmd       : lenh (v m/s hoac duty), cot
%   cnt       : xung luy ke [FL FR RL RR], N x 4
%   x, y, th  : toa do encoder (mm, mm, do), cot
%   hit_wheel : banh cham moc ('RL', ...) hoac 'HET_GIO' / '?'
%   t_hit     : thoi gian chay (ms)
%   cnt_final : xung cuoi cung [FL FR RL RR]
%   x_end, y_end, th_end : toa do cuoi cung lay tu khoi KET QUA
%   Log co IMU (14 cot) them:
%   yaw, gz, u, xi, yi   : yaw IMU (do), gyro z (do/s), hieu chinh (duty),
%                          toa do encoder + huong IMU (mm)
%   xi_end, yi_end, yaw_end, yaw_max_abs, u_mean, kp, ki, kd : tu khoi KET QUA

fid = fopen(filename, 'r');
if fid < 0
    error('Khong mo duoc file log: %s', filename);
end

runs = new_run();
runs(1) = [];
k = 0;

while true
    ln = fgetl(fid);
    if ~ischar(ln), break; end

    p = strfind(ln, '[STM] ');
    if ~isempty(p)
        ln = ln(p(1) + 6:end);
    elseif ~isempty(strfind(ln, '[SRV]')) || ~isempty(strfind(ln, '[R29]'))
        continue;                                  % dong trang thai cua ESP32
    end
    ln = ln(ln >= 32 & ln <= 126);                 % bo ky tu rac (vd luc boot)
    ln = strtrim(ln);

    if strncmp(ln, 'ENC,t_ms', 8)                  % bat dau 1 lan chay moi
        k = k + 1;
        runs(k) = new_run();
        continue;
    end
    if k == 0, continue; end

    if strncmp(ln, 'ENC,', 4)
        v = sscanf(ln(5:end), '%f,');
        if numel(v) == 9
            v = [v; NaN(5, 1)];                    % log cu, chua co cot IMU
        elseif numel(v) ~= 14
            continue;
        end
        if ~isempty(runs(k).t_ms) && v(1) == runs(k).t_ms(end)
            continue;                              % dong bi gui lap qua ESP-NOW
        end
        runs(k).t_ms(end+1, 1)  = v(1);
        runs(k).cmd(end+1, 1)   = v(2);
        runs(k).cnt(end+1, :)   = v(3:6).';
        runs(k).x(end+1, 1)     = v(7);
        runs(k).y(end+1, 1)     = v(8);
        runs(k).th(end+1, 1)    = v(9);
        runs(k).yaw(end+1, 1)   = v(10);
        runs(k).gz(end+1, 1)    = v(11);
        runs(k).u(end+1, 1)     = v(12);
        runs(k).xi(end+1, 1)    = v(13);
        runs(k).yi(end+1, 1)    = v(14);

    elseif strncmp(ln, 'ket_qua:', 8) && isempty(runs(k).hit_wheel)
        tok = regexp(ln, 'banh (\w+)', 'tokens', 'once');
        if ~isempty(tok)
            runs(k).hit_wheel = tok{1};
        elseif ~isempty(strfind(ln, 'HET_GIO'))
            runs(k).hit_wheel = 'HET_GIO';
        end

    elseif strncmp(ln, 'thoi_gian_chay_ms=', 18) && isnan(runs(k).t_hit)
        runs(k).t_hit = sscanf(ln(19:end), '%f', 1);

    elseif strncmp(ln, 'xung_cuoi_cung:', 15) && any(isnan(runs(k).cnt_final))
        tok = regexp(ln, 'FL=(-?\d+)\s+FR=(-?\d+)\s+RL=(-?\d+)\s+RR=(-?\d+)', 'tokens', 'once');
        if numel(tok) == 4
            runs(k).cnt_final = str2double(tok);
        end

    elseif strncmp(ln, 'toa_do_imu_cuoi_cung:', 21) && isnan(runs(k).xi_end)
        tok = regexp(ln, 'x=(-?\d+)\s*mm\s+y=(-?\d+)\s*mm\s+yaw=(-?[\d.]+)', 'tokens', 'once');
        if numel(tok) == 3
            runs(k).xi_end  = str2double(tok{1});
            runs(k).yi_end  = str2double(tok{2});
            runs(k).yaw_end = str2double(tok{3});
        end

    elseif strncmp(ln, 'imu:', 4) && isnan(runs(k).yaw_max_abs)
        tok = regexp(ln, 'yaw_max_abs=(-?[\d.]+).*u_tb_on_dinh=(-?[\d.]+)', 'tokens', 'once');
        if numel(tok) == 2
            runs(k).yaw_max_abs = str2double(tok{1});
            runs(k).u_mean      = str2double(tok{2});
        end
        tok = regexp(ln, 'KP=(-?[\d.]+)\s+KI=(-?[\d.]+)\s+KD=(-?[\d.]+)', 'tokens', 'once');
        if numel(tok) == 3
            runs(k).kp = str2double(tok{1});
            runs(k).ki = str2double(tok{2});
            runs(k).kd = str2double(tok{3});
        end

    elseif strncmp(ln, 'toa_do_cuoi_cung:', 17) && isnan(runs(k).x_end)
        tok = regexp(ln, 'x=(-?\d+)\s*mm\s+y=(-?\d+)\s*mm\s+th=(-?[\d.]+)', 'tokens', 'once');
        if numel(tok) == 3
            runs(k).x_end  = str2double(tok{1});
            runs(k).y_end  = str2double(tok{2});
            runs(k).th_end = str2double(tok{3});
        end
    end
end
fclose(fid);

% Bo lan chay khong co du lieu; lan nao thieu khoi KET QUA thi lay diem cuoi
keep = true(1, numel(runs));
for k = 1:numel(runs)
    r = runs(k);
    if isempty(r.t_ms)
        keep(k) = false;
        continue;
    end
    if isnan(r.x_end)
        runs(k).x_end = r.x(end); runs(k).y_end = r.y(end); runs(k).th_end = r.th(end);
    end
    if any(isnan(r.cnt_final)), runs(k).cnt_final = r.cnt(end, :); end
    if isnan(r.xi_end) && ~isnan(r.xi(end))
        runs(k).xi_end = r.xi(end); runs(k).yi_end = r.yi(end); runs(k).yaw_end = r.yaw(end);
    end
    if isempty(r.hit_wheel),    runs(k).hit_wheel = '?';           end
end
runs = runs(keep);
end

function r = new_run()
r = struct('t_ms', zeros(0, 1), 'cmd', zeros(0, 1), 'cnt', zeros(0, 4), ...
           'x', zeros(0, 1), 'y', zeros(0, 1), 'th', zeros(0, 1), ...
           'hit_wheel', '', 't_hit', NaN, 'cnt_final', NaN(1, 4), ...
           'x_end', NaN, 'y_end', NaN, 'th_end', NaN, ...
           'yaw', zeros(0, 1), 'gz', zeros(0, 1), 'u', zeros(0, 1), ...
           'xi', zeros(0, 1), 'yi', zeros(0, 1), ...
           'xi_end', NaN, 'yi_end', NaN, 'yaw_end', NaN, ...
           'yaw_max_abs', NaN, 'u_mean', NaN, 'kp', NaN, 'ki', NaN, 'kd', NaN);
end
