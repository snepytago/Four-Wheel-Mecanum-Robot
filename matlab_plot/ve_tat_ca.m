%% ve_tat_ca.m
% Chay lan luot moi script ve, khong animation, khong cho bam phim.
% Moi script tu luu PNG vao thu muc nay; dong hinh sau moi script cho nhe may.

SCRIPTS = {'plot_fwmr_runs', 'compare_enc_vs_imu', 'compare_3_methods', 'plot_ekf_runs', ...
           'compare_ekf_truoc_sau_sua', 'plot_slide_3_tests', 'plot_strafe', 'plot_rot', 'plot_zigzag', 'compare_zigzag_3_versions'};
for i = 1:numel(SCRIPTS)
    fprintf('\n===== %s =====\n', SCRIPTS{i});
    try
        run(SCRIPTS{i});
    catch err
        fprintf(2, 'Loi o %s: %s\n', SCRIPTS{i}, err.message);
    end
    close all;
end
fprintf('\nXong. Anh PNG da luu trong thu muc %s\n', pwd);
