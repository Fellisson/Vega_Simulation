function results = compare_pid_vs_no_pid(mode_name)
% Compare attitude response and mission evolution with and without PID.
%
% Usage:
%   results = compare_pid_vs_no_pid
%   results = compare_pid_vs_no_pid('no_render')

    if nargin < 1 || isempty(mode_name)
        mode_name = 'no_render';
    end

    close all;
    clc;

    [data_pid, metrics_pid] = modele_vega_PID_ok(mode_name, struct('pid_enabled', true));
    [data_open, metrics_open] = modele_vega_PID_ok(mode_name, struct('pid_enabled', false));

    paths = build_compare_paths();
    ensure_compare_dirs(paths);

    results = build_results(data_pid, metrics_pid, data_open, metrics_open);

    writetable(results.pitch_table, fullfile(paths.logs_dir, 'compare_pid_pitch_table.csv'));
    writetable(results.mission_table, fullfile(paths.logs_dir, 'compare_pid_mission_table.csv'));
    export_excel_workbook(fullfile(paths.logs_dir, 'compare_pid_vs_no_pid.xlsx'), results, mode_name);
    write_text_table(fullfile(paths.logs_dir, 'compare_pid_pitch_table.txt'), 'Comparaison tangage', results.pitch_table);
    write_text_table(fullfile(paths.logs_dir, 'compare_pid_mission_table.txt'), 'Comparaison mission', results.mission_table);
    write_summary_report(fullfile(paths.logs_dir, 'compare_pid_summary.txt'), results);

    export_dashboard_figure(fullfile(paths.images_dir, 'compare_pid_dashboard.png'), data_pid, data_open, results);
    export_table_figure(fullfile(paths.images_dir, 'compare_pid_pitch_table.png'), 'Comparaison tangage', results.pitch_table);
    export_table_figure(fullfile(paths.images_dir, 'compare_pid_mission_table.png'), 'Comparaison mission', results.mission_table);
    export_comparison_video(fullfile(paths.videos_dir, 'compare_pid_vs_no_pid.mp4'), data_pid, data_open, results);

    fprintf('\n===== COMPARAISON PID / SANS PID =====\n');
    fprintf('Tableau tangage  : %s\n', fullfile(paths.logs_dir, 'compare_pid_pitch_table.csv'));
    fprintf('Tableau mission  : %s\n', fullfile(paths.logs_dir, 'compare_pid_mission_table.csv'));
    fprintf('Classeur Excel   : %s\n', fullfile(paths.logs_dir, 'compare_pid_vs_no_pid.xlsx'));
    fprintf('Synthese         : %s\n', fullfile(paths.logs_dir, 'compare_pid_summary.txt'));
    fprintf('Image dashboard  : %s\n', fullfile(paths.images_dir, 'compare_pid_dashboard.png'));
    fprintf('Image tableau    : %s\n', fullfile(paths.images_dir, 'compare_pid_pitch_table.png'));
    fprintf('Video comparaison: %s\n', fullfile(paths.videos_dir, 'compare_pid_vs_no_pid.mp4'));
    fprintf('RMS theta (PID / sans PID) = %.3f / %.3f deg\n', ...
        results.pitch.with_pid.rms_error_deg, results.pitch.without_pid.rms_error_deg);
end

function paths = build_compare_paths()
    base_dir = fileparts(mfilename('fullpath'));
    project_dir = fileparts(base_dir);
    paths.images_dir = fullfile(project_dir, 'out', 'images');
    paths.logs_dir = fullfile(project_dir, 'out', 'logs');
    paths.videos_dir = fullfile(project_dir, 'out', 'videos');
end

function ensure_compare_dirs(paths)
    if ~exist(paths.images_dir, 'dir')
        mkdir(paths.images_dir);
    end
    if ~exist(paths.logs_dir, 'dir')
        mkdir(paths.logs_dir);
    end
    if ~exist(paths.videos_dir, 'dir')
        mkdir(paths.videos_dir);
    end
end

function results = build_results(data_pid, metrics_pid, data_open, metrics_open)
    pitch_pid = compute_pitch_metrics(data_pid);
    pitch_open = compute_pitch_metrics(data_open);

    results.pitch.with_pid = pitch_pid;
    results.pitch.without_pid = pitch_open;
    results.pitch_table = build_pitch_table(pitch_pid, pitch_open);
    results.mission_table = build_mission_table(metrics_pid, metrics_open);
    results.data_pid = data_pid;
    results.data_open = data_open;
    results.metrics_pid = metrics_pid;
    results.metrics_open = metrics_open;
end

function export_excel_workbook(output_path, results, mode_name)
    inputs_table = build_inputs_table(results, mode_name);
    pid_series = build_time_series_table(results.data_pid, 'avec_pid');
    open_series = build_time_series_table(results.data_open, 'sans_pid');

    writetable(inputs_table, output_path, 'Sheet', 'Entrees');
    writetable(results.pitch_table, output_path, 'Sheet', 'Tangage');
    writetable(results.mission_table, output_path, 'Sheet', 'Mission');
    writetable(pid_series, output_path, 'Sheet', 'Serie_Avec_PID');
    writetable(open_series, output_path, 'Sheet', 'Serie_Sans_PID');
end

function T = build_inputs_table(results, mode_name)
    params = { ...
        'Mode simulation'; ...
        'PID active'; ...
        'kp_phi'; ...
        'ki_phi'; ...
        'kd_phi'; ...
        'kp_theta'; ...
        'ki_theta'; ...
        'kd_theta'; ...
        'kp_psi'; ...
        'ki_psi'; ...
        'kd_psi'};

    pid = read_pid_gains('D:\MATLAB\modele_vega_PID_ok.m');
    values = { ...
        mode_name; ...
        'comparaison avec / sans PID'; ...
        pid.phi.kp; ...
        pid.phi.ki; ...
        pid.phi.kd; ...
        pid.theta.kp; ...
        pid.theta.ki; ...
        pid.theta.kd; ...
        pid.psi.kp; ...
        pid.psi.ki; ...
        pid.psi.kd};

    T = table(params, values, 'VariableNames', {'Parametre', 'Valeur'});
end

function pid = read_pid_gains(file_path)
    txt = fileread(file_path);
    pid.phi = parse_axis(txt, 'phi');
    pid.theta = parse_axis(txt, 'theta');
    pid.psi = parse_axis(txt, 'psi');
end

function axis = parse_axis(txt, axis_name)
    expr = sprintf('cfg\\.pid_%s = struct\\(''kp'',\\s*([\\deE+\\.-]+),\\s*''ki'',\\s*([\\deE+\\.-]+),\\s*''kd'',\\s*([\\deE+\\.-]+)\\);', axis_name);
    tokens = regexp(txt, expr, 'tokens', 'once');
    axis.kp = str2double(tokens{1});
    axis.ki = str2double(tokens{2});
    axis.kd = str2double(tokens{3});
end

function T = build_time_series_table(data, label_name)
    n = numel(data.t);
    case_name = repmat({label_name}, n, 1);
    speed = sqrt(data.vx(:).^2 + data.vy(:).^2 + data.vz(:).^2);
    if isfield(data, 'guidance_active')
        guidance_active = data.guidance_active(:);
    else
        guidance_active = false(n, 1);
    end
    if isfield(data, 'control_active')
        control_active = data.control_active(:);
    else
        control_active = false(n, 1);
    end

    T = table( ...
        case_name, data.t(:), ...
        data.x(:), data.y(:), data.z(:), ...
        data.vx(:), data.vy(:), data.vz(:), speed, ...
        rad2deg(data.phi(:)), rad2deg(data.theta(:)), rad2deg(data.psi(:)), ...
        rad2deg(data.phi_ref(:)), rad2deg(data.theta_ref(:)), rad2deg(data.psi_ref(:)), ...
        data.Tcmd(:), data.mass(:), guidance_active, control_active, ...
        data.pid_m_phi(:), data.pid_m_theta(:), data.pid_m_psi(:), ...
        'VariableNames', { ...
        'cas', 'time_s', ...
        'x_m', 'y_m', 'z_m', ...
        'vx_mps', 'vy_mps', 'vz_mps', 'speed_mps', ...
        'phi_deg', 'theta_deg', 'psi_deg', ...
        'phi_ref_deg', 'theta_ref_deg', 'psi_ref_deg', ...
        'thrust_n', 'mass_kg', 'guidance_active', 'control_active', ...
        'moment_phi_nm', 'moment_theta_nm', 'moment_psi_nm'});
end

function pitch = compute_pitch_metrics(data)
    idx = data.guidance_active;
    t = data.t(idx);
    ref = rad2deg(data.theta_ref(idx));
    theta = rad2deg(data.theta(idx));
    effort = data.pid_m_theta(idx);

    pitch.rise_time_s = NaN;
    pitch.overshoot_pct = NaN;
    pitch.settling_time_s = NaN;
    pitch.static_error_deg = NaN;
    pitch.rms_error_deg = NaN;
    pitch.command_effort_rms = NaN;

    if isempty(t)
        return;
    end

    target = ref(end);
    initial = theta(1);
    delta = target - initial;

    pitch.static_error_deg = target - theta(end);
    pitch.rms_error_deg = sqrt(mean((ref - theta) .^ 2));
    pitch.command_effort_rms = sqrt(mean(effort .^ 2));

    if abs(delta) < 1e-9
        pitch.overshoot_pct = 0;
        pitch.settling_time_s = 0;
        pitch.rise_time_s = 0;
        return;
    end

    lower = initial + 0.1 * delta;
    upper = initial + 0.9 * delta;
    i10 = find_crossing(theta, lower, delta);
    i90 = find_crossing(theta, upper, delta);
    if ~isempty(i10) && ~isempty(i90) && i90 >= i10
        pitch.rise_time_s = t(i90) - t(i10);
    end

    if delta > 0
        pitch.overshoot_pct = max(0, (max(theta) - target) / abs(delta) * 100);
    else
        pitch.overshoot_pct = max(0, (target - min(theta)) / abs(delta) * 100);
    end

    tol = max(0.05 * abs(delta), 0.5);
    last_outside = find(abs(theta - target) > tol, 1, 'last');
    if isempty(last_outside)
        pitch.settling_time_s = 0;
    elseif last_outside < numel(t)
        pitch.settling_time_s = t(last_outside + 1) - t(1);
    end
end

function idx = find_crossing(y, threshold, delta)
    if delta > 0
        idx = find(y >= threshold, 1, 'first');
    else
        idx = find(y <= threshold, 1, 'first');
    end
end

function T = build_pitch_table(with_pid, without_pid)
    metrics = { ...
        'Temps de montee (s)'; ...
        'Depassement maximal (%)'; ...
        'Temps d''etablissement (s)'; ...
        'Erreur statique (deg)'; ...
        'RMS de l''erreur (deg)'; ...
        'Effort de commande RMS (N.m)'};

    with_vals = [ ...
        with_pid.rise_time_s; ...
        with_pid.overshoot_pct; ...
        with_pid.settling_time_s; ...
        with_pid.static_error_deg; ...
        with_pid.rms_error_deg; ...
        with_pid.command_effort_rms];

    without_vals = [ ...
        without_pid.rise_time_s; ...
        without_pid.overshoot_pct; ...
        without_pid.settling_time_s; ...
        without_pid.static_error_deg; ...
        without_pid.rms_error_deg; ...
        without_pid.command_effort_rms];

    gain = 100 * (without_vals - with_vals) ./ max(abs(without_vals), 1e-9);
    T = table(metrics, with_vals, without_vals, gain, ...
        'VariableNames', {'Metrique', 'Avec_PID', 'Sans_PID', 'Gain_pct'});
end

function T = build_mission_table(metrics_pid, metrics_open)
    metric_names = { ...
        'Temps de vol (s)'; ...
        'Portee horizontale (m)'; ...
        'Altitude maximale (m)'; ...
        'Vitesse max (m/s)'; ...
        'Temps de separation satellite (s)'};

    pid_vals = [ ...
        metrics_pid.flight_time; ...
        metrics_pid.range_xy; ...
        metrics_pid.max_altitude; ...
        metrics_pid.max_speed; ...
        metrics_pid.satellite_release_time];

    open_vals = [ ...
        metrics_open.flight_time; ...
        metrics_open.range_xy; ...
        metrics_open.max_altitude; ...
        metrics_open.max_speed; ...
        metrics_open.satellite_release_time];

    T = table(metric_names, pid_vals, open_vals, ...
        'VariableNames', {'Metrique', 'Avec_PID', 'Sans_PID'});
end

function write_text_table(output_path, title_text, T)
    fid = fopen(output_path, 'w');
    if fid < 0
        warning('Impossible d''ecrire le tableau texte : %s', output_path);
        return;
    end
    fprintf(fid, '%s\n\n', title_text);
    fprintf(fid, '%s\n', evalc('disp(T)'));
    fclose(fid);
end

function write_summary_report(output_path, results)
    fid = fopen(output_path, 'w');
    if fid < 0
        warning('Impossible d''ecrire la synthese : %s', output_path);
        return;
    end

    pid_rms = results.pitch.with_pid.rms_error_deg;
    open_rms = results.pitch.without_pid.rms_error_deg;
    improvement_pct = 100 * (open_rms - pid_rms) / max(abs(open_rms), 1e-9);

    if improvement_pct >= 50
        verdict = 'Gain fort du PID';
    elseif improvement_pct >= 20
        verdict = 'Gain net du PID';
    elseif improvement_pct >= 5
        verdict = 'Gain modere du PID';
    else
        verdict = 'Gain limite du PID';
    end

    fprintf(fid, '===== SYNTHESE COMPARAISON PID / SANS PID =====\n');
    fprintf(fid, 'Verdict                : %s\n', verdict);
    fprintf(fid, 'RMS theta PID          : %.4f deg\n', pid_rms);
    fprintf(fid, 'RMS theta sans PID     : %.4f deg\n', open_rms);
    fprintf(fid, 'Amelioration RMS theta : %.2f %%\n', improvement_pct);
    fprintf(fid, 'Temps de montee PID    : %.4f s\n', results.pitch.with_pid.rise_time_s);
    fprintf(fid, 'Temps de montee sans   : %.4f s\n', results.pitch.without_pid.rise_time_s);
    fprintf(fid, 'Depassement PID        : %.4f %%\n', results.pitch.with_pid.overshoot_pct);
    fprintf(fid, 'Depassement sans PID   : %.4f %%\n', results.pitch.without_pid.overshoot_pct);
    fprintf(fid, 'Effort RMS PID         : %.4e N.m\n', results.pitch.with_pid.command_effort_rms);
    fprintf(fid, 'Effort RMS sans PID    : %.4e N.m\n', results.pitch.without_pid.command_effort_rms);
    fprintf(fid, '================================================\n');
    fclose(fid);
end

function export_dashboard_figure(output_path, data_pid, data_open, results)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 60 1380 880]);
    tl = tiledlayout(fig, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile(tl, 1);
    hold(ax1, 'on'); grid(ax1, 'on'); box(ax1, 'on');
    plot(ax1, data_pid.t, rad2deg(data_pid.theta_ref), 'k--', 'LineWidth', 1.2);
    plot(ax1, data_pid.t, rad2deg(data_pid.theta), 'b', 'LineWidth', 1.5);
    plot(ax1, data_open.t, rad2deg(data_open.theta), 'r', 'LineWidth', 1.3);
    title(ax1, 'Tangage');
    xlabel(ax1, 'Temps (s)');
    ylabel(ax1, 'Theta (deg)');
    legend(ax1, 'Reference', 'Avec PID', 'Sans PID', 'Location', 'best');

    ax2 = nexttile(tl, 2);
    hold(ax2, 'on'); grid(ax2, 'on'); box(ax2, 'on');
    err_pid = rad2deg(data_pid.theta_ref - data_pid.theta);
    err_open = rad2deg(data_open.theta_ref - data_open.theta);
    plot(ax2, data_pid.t, err_pid, 'b', 'LineWidth', 1.4);
    plot(ax2, data_open.t, err_open, 'r', 'LineWidth', 1.2);
    title(ax2, 'Erreur de tangage');
    xlabel(ax2, 'Temps (s)');
    ylabel(ax2, 'Erreur (deg)');
    legend(ax2, 'Avec PID', 'Sans PID', 'Location', 'best');

    ax3 = nexttile(tl, 3);
    hold(ax3, 'on'); grid(ax3, 'on'); box(ax3, 'on');
    plot3(ax3, data_pid.x, data_pid.y, data_pid.z, 'b', 'LineWidth', 1.5);
    plot3(ax3, data_open.x, data_open.y, data_open.z, 'r', 'LineWidth', 1.2);
    title(ax3, 'Trajectoire 3D');
    xlabel(ax3, 'X (m)'); ylabel(ax3, 'Y (m)'); zlabel(ax3, 'Z (m)');
    legend(ax3, 'Avec PID', 'Sans PID', 'Location', 'best');
    view(ax3, 38, 24);

    ax4 = nexttile(tl, 4);
    hold(ax4, 'on'); grid(ax4, 'on'); box(ax4, 'on');
    plot(ax4, data_pid.t, data_pid.pid_m_theta, 'b', 'LineWidth', 1.3);
    plot(ax4, data_open.t, data_open.pid_m_theta, 'r', 'LineWidth', 1.1);
    title(ax4, 'Effort de commande theta');
    xlabel(ax4, 'Temps (s)');
    ylabel(ax4, 'Moment (N.m)');
    legend(ax4, 'Avec PID', 'Sans PID', 'Location', 'best');

    ax5 = nexttile(tl, 5);
    hold(ax5, 'on'); grid(ax5, 'on'); box(ax5, 'on');
    plot(ax5, data_pid.t, data_pid.z, 'b', 'LineWidth', 1.4);
    plot(ax5, data_open.t, data_open.z, 'r', 'LineWidth', 1.2);
    title(ax5, 'Altitude');
    xlabel(ax5, 'Temps (s)');
    ylabel(ax5, 'Z (m)');
    legend(ax5, 'Avec PID', 'Sans PID', 'Location', 'best');

    ax6 = nexttile(tl, 6);
    axis(ax6, 'off');
    lines = { ...
        sprintf('Temps de montee theta (PID / sans PID) : %.2f / %.2f s', ...
            results.pitch.with_pid.rise_time_s, results.pitch.without_pid.rise_time_s), ...
        sprintf('Depassement theta (PID / sans PID)     : %.2f / %.2f %%', ...
            results.pitch.with_pid.overshoot_pct, results.pitch.without_pid.overshoot_pct), ...
        sprintf('Temps d''etablissement                  : %.2f / %.2f s', ...
            results.pitch.with_pid.settling_time_s, results.pitch.without_pid.settling_time_s), ...
        sprintf('Erreur statique theta                  : %.2f / %.2f deg', ...
            results.pitch.with_pid.static_error_deg, results.pitch.without_pid.static_error_deg), ...
        sprintf('RMS erreur theta                       : %.2f / %.2f deg', ...
            results.pitch.with_pid.rms_error_deg, results.pitch.without_pid.rms_error_deg), ...
        sprintf('Effort RMS theta                       : %.2e / %.2e N.m', ...
            results.pitch.with_pid.command_effort_rms, results.pitch.without_pid.command_effort_rms)};
    text(ax6, 0.02, 0.95, strjoin(lines, '\n'), 'Units', 'normalized', ...
        'VerticalAlignment', 'top', 'FontName', 'Consolas', 'FontSize', 11);

    sgtitle(tl, 'Comparaison avec et sans PID');
    saveas(fig, output_path);
    close(fig);
end

function export_table_figure(output_path, title_text, T)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [120 120 1160 360]);
    ax = axes(fig, 'Position', [0 0 1 1], 'Visible', 'off');

    lines = table_to_lines(T);
    text(ax, 0.02, 0.95, title_text, 'Units', 'normalized', ...
        'FontWeight', 'bold', 'FontSize', 13, 'VerticalAlignment', 'top');
    text(ax, 0.02, 0.86, strjoin(lines, '\n'), 'Units', 'normalized', ...
        'FontName', 'Consolas', 'FontSize', 10.5, 'VerticalAlignment', 'top', ...
        'Interpreter', 'none');
    saveas(fig, output_path);
    close(fig);
end

function lines = table_to_lines(T)
    headers = T.Properties.VariableNames;
    rows = cell(size(T, 1) + 1, 1);
    rows{1} = sprintf('%-32s | %14s | %14s | %12s', headers{1}, headers{2}, headers{3}, headers{4});
    if width(T) == 3
        rows{1} = sprintf('%-32s | %14s | %14s', headers{1}, headers{2}, headers{3});
    end

    for i = 1:size(T, 1)
        first = string(T{i, 1});
        if width(T) == 4
            rows{i + 1} = sprintf('%-32s | %14.4f | %14.4f | %12.2f', ...
                first, T{i, 2}, T{i, 3}, T{i, 4});
        else
            rows{i + 1} = sprintf('%-32s | %14.4f | %14.4f', ...
                first, T{i, 2}, T{i, 3});
        end
    end

    lines = rows;
end

function export_comparison_video(output_path, data_pid, data_open, results)
    if exist('OCTAVE_VERSION', 'builtin') ~= 0
        warning('Export video indisponible sous Octave pour ce comparateur.');
        return;
    end

    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [60 60 1380 860]);
    tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile(tl, 1);
    hold(ax1, 'on'); grid(ax1, 'on'); box(ax1, 'on');
    plot3(ax1, data_pid.x, data_pid.y, data_pid.z, 'Color', [0.75 0.85 1.0], 'LineWidth', 1.0);
    hTracePid = plot3(ax1, NaN, NaN, NaN, 'b', 'LineWidth', 2);
    hCurPid = plot3(ax1, NaN, NaN, NaN, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 6);
    title(ax1, 'Avec PID');
    xlabel(ax1, 'X'); ylabel(ax1, 'Y'); zlabel(ax1, 'Z');
    view(ax1, 38, 24);

    ax2 = nexttile(tl, 2);
    hold(ax2, 'on'); grid(ax2, 'on'); box(ax2, 'on');
    plot3(ax2, data_open.x, data_open.y, data_open.z, 'Color', [1.0 0.85 0.85], 'LineWidth', 1.0);
    hTraceOpen = plot3(ax2, NaN, NaN, NaN, 'r', 'LineWidth', 2);
    hCurOpen = plot3(ax2, NaN, NaN, NaN, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
    title(ax2, 'Sans PID');
    xlabel(ax2, 'X'); ylabel(ax2, 'Y'); zlabel(ax2, 'Z');
    view(ax2, 38, 24);

    ax3 = nexttile(tl, 3, [1 2]);
    hold(ax3, 'on'); grid(ax3, 'on'); box(ax3, 'on');
    plot(ax3, data_pid.t, rad2deg(data_pid.theta_ref), 'k--', 'LineWidth', 1.0);
    hThetaPid = plot(ax3, NaN, NaN, 'b', 'LineWidth', 1.6);
    hThetaOpen = plot(ax3, NaN, NaN, 'r', 'LineWidth', 1.4);
    hMarkPid = plot(ax3, NaN, NaN, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 6);
    hMarkOpen = plot(ax3, NaN, NaN, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
    title(ax3, sprintf('Tangage | RMS theta PID = %.2f deg | sans PID = %.2f deg', ...
        results.pitch.with_pid.rms_error_deg, results.pitch.without_pid.rms_error_deg));
    xlabel(ax3, 'Temps (s)');
    ylabel(ax3, 'Theta (deg)');
    legend(ax3, 'Reference', 'Avec PID', 'Sans PID', 'Location', 'best');

    all_x = [data_pid.x(:); data_open.x(:)];
    all_y = [data_pid.y(:); data_open.y(:)];
    all_z = [data_pid.z(:); data_open.z(:)];
    xlim(ax1, [min(all_x), max(all_x)]); ylim(ax1, [min(all_y), max(all_y)]); zlim(ax1, [0, max(all_z)]);
    xlim(ax2, [min(all_x), max(all_x)]); ylim(ax2, [min(all_y), max(all_y)]); zlim(ax2, [0, max(all_z)]);
    xlim(ax3, [0, max([data_pid.t(end), data_open.t(end)])]);

    writer = VideoWriter(output_path, 'MPEG-4');
    writer.FrameRate = 18;
    writer.Quality = 80;
    open(writer);

    t_end = max([data_pid.t(end), data_open.t(end)]);
    t_video = linspace(0, t_end, 220);
    for i = 1:numel(t_video)
        tk = t_video(i);
        idx_pid = data_pid.t <= tk;
        idx_open = data_open.t <= tk;

        set(hTracePid, 'XData', data_pid.x(idx_pid), 'YData', data_pid.y(idx_pid), 'ZData', data_pid.z(idx_pid));
        set(hTraceOpen, 'XData', data_open.x(idx_open), 'YData', data_open.y(idx_open), 'ZData', data_open.z(idx_open));

        set(hCurPid, 'XData', hold_last(data_pid.t, data_pid.x, tk), ...
            'YData', hold_last(data_pid.t, data_pid.y, tk), ...
            'ZData', hold_last(data_pid.t, data_pid.z, tk));
        set(hCurOpen, 'XData', hold_last(data_open.t, data_open.x, tk), ...
            'YData', hold_last(data_open.t, data_open.y, tk), ...
            'ZData', hold_last(data_open.t, data_open.z, tk));

        set(hThetaPid, 'XData', data_pid.t(idx_pid), 'YData', rad2deg(data_pid.theta(idx_pid)));
        set(hThetaOpen, 'XData', data_open.t(idx_open), 'YData', rad2deg(data_open.theta(idx_open)));
        set(hMarkPid, 'XData', tk, 'YData', rad2deg(hold_last(data_pid.t, data_pid.theta, tk)));
        set(hMarkOpen, 'XData', tk, 'YData', rad2deg(hold_last(data_open.t, data_open.theta, tk)));

        drawnow;
        writeVideo(writer, getframe(fig));
    end

    close(writer);
    close(fig);
end

function yk = hold_last(t, y, tk)
    if tk <= t(1)
        yk = y(1);
        return;
    end
    idx = find(t <= tk, 1, 'last');
    yk = y(idx);
end
