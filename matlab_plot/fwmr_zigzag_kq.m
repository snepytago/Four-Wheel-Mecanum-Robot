function [KQ, MODE] = fwmr_zigzag_kq(filename, NS)
% FWMR_ZIGZAG_KQ  Doc cac dong "ZZ:KQ k=.." cua log zigzag (lan in dau tien cua moi buoc).
%
%   [KQ, MODE] = fwmr_zigzag_kq('log_zigzag_mode2_20102026.txt', 19)
%
% KQ{r} : NS x 15 cho lan chay r (thu tu giong fwmr_parse_log):
%   1 k | 2-4 dich x,y,huong | 5-7 imu/ekf x,y,huong | 8-10 camera x,y,huong
%   11-13 encoder x,y,huong | 14 t_bd_ms | 15 t_cat_ms      (mm, do, ms ; NaN neu khong co)
% MODE(r) : ZZ_MODE cua lan chay r (0, 1, 2)

if nargin < 2, NS = 19; end
fid = fopen(filename, 'r');
if fid < 0, error('Khong mo duoc file log: %s', filename); end
KQ = {};  MODE = [];
ir = 0;  md = NaN;
while true
    ln = fgetl(fid);
    if ~ischar(ln), break; end
    ln = ln(ln >= 32 & ln <= 126);
    p = strfind(ln, 'ZZ:START mode=');
    if ~isempty(p), md = sscanf(ln(p(1) + 14:end), '%d', 1); continue; end
    if ~isempty(strfind(ln, 'ENC,t_ms')), ir = ir + 1; KQ{ir} = NaN(NS, 15); MODE(ir) = md; continue; end
    p = strfind(ln, 'ZZ:KQ k=');
    if ir == 0 || isempty(p), continue; end
    ln = ln(p(1):end);
    k = sscanf(ln(9:end), '%d', 1);
    if isempty(k) || k < 1 || k > NS || ~isnan(KQ{ir}(k, 1)), continue; end
    v = NaN(1, 15);
    v(1) = k;
    q = regexp(ln, 'dich=(-?\d+),(-?\d+),(-?\d+)', 'tokens', 'once');           if ~isempty(q), v(2:4)   = str2double(q); end
    q = regexp(ln, '(?:imu|ekf)=(-?\d+),(-?\d+),(-?[\d.]+)', 'tokens', 'once'); if ~isempty(q), v(5:7)   = str2double(q); end
    q = regexp(ln, ' cam=(-?\d+),(-?\d+),(-?[\d.]+)', 'tokens', 'once');        if ~isempty(q), v(8:10)  = str2double(q); end
    q = regexp(ln, 'enc=(-?\d+),(-?\d+),(-?[\d.]+)', 'tokens', 'once');         if ~isempty(q), v(11:13) = str2double(q); end
    q = regexp(ln, 't_bd_ms=(\d+) t_cat_ms=(\d+)', 'tokens', 'once');           if ~isempty(q), v(14:15) = str2double(q); end
    KQ{ir}(k, :) = v;
end
fclose(fid);
end
