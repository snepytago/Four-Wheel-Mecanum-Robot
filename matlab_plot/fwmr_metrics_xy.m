function m = fwmr_metrics_xy(x, y, xe, ye, hd, hde)
% FWMR_METRICS_XY  Chi so lech ngang cua 1 lan chay (he xuat phat, mm / do).
%   m = [|y| cuoi, RMS lech ngang (tich phan theo x), |y| max, |goc| max, |goc| cuoi]
[xs, o] = sort([x(:); xe]);
ys = [y(:); ye];
ys = ys(o);
[xs, iu] = unique(xs);
ys = ys(iu);
rms_y = sqrt(trapz(xs, ys.^2) / max(xs(end) - xs(1), 1));
m = [abs(ye), rms_y, max(abs([y(:); ye])), max(abs(hd(:))), abs(hde)];
end
