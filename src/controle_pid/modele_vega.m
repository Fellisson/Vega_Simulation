function modele_vega(mode_name)
% Modele Vega 3D open-loop.
% This retained "sans PID" version simulates a Vega-inspired propelled
% vehicle in 3D and can render either a lightweight preview or a video.
%
% Usage examples:
%   modele_vega
%   modele_vega('preview')
%   modele_vega('video')
%   modele_vega('ultra_light')
%   modele_vega('debug')
%   modele_vega('no_render')
%
% Outputs:
%   - out/images/modele_vega.png
%   - out/logs/modele_vega.txt
%   - out/videos/modele_vega.mp4 (only when video export is enabled)

    if nargin < 1 || isempty(mode_name)
        mode_name = 'preview';
    end

    clc; close all;

    cfg = build_config(mode_name);
    paths = build_paths();
    ensure_output_dirs(paths);
    display_vega_c_reference_table();

    data = simulate_open_loop_3d(cfg);
    metrics = compute_flight_metrics(data);

    if cfg.render_enabled
        render_outputs(data, metrics, cfg, paths);
    end
    write_metrics_log(fullfile(paths.logs_dir, 'modele_vega.txt'), metrics, cfg);
    write_model_presentation(fullfile(paths.logs_dir, 'modele_vega_presentation.txt'));

    fprintf('\nMode : %s\n', cfg.mode_name);
    if cfg.render_enabled
        fprintf('Image enregistree : %s\n', fullfile(paths.images_dir, 'modele_vega.png'));
    else
        fprintf('Image : rendu desactive pour ce mode.\n');
    end
    if cfg.export_video
        fprintf('Video enregistree : %s\n', fullfile(paths.videos_dir, 'modele_vega.mp4'));
    else
        fprintf('Video : export desactive pour ce mode.\n');
    end
    fprintf('Temps de vol : %.2f s | Portee horizontale : %.2f m | Altitude max : %.2f m\n', ...
        metrics.flight_time, metrics.range_xy, metrics.max_altitude);
    fprintf('Fin de simulation : %s\n', metrics.termination_reason);
    if metrics.satellite_released
        fprintf('Satellite deploye a t = %.2f s | altitude = %.2f m\n', ...
            metrics.satellite_release_time, metrics.satellite_release_altitude);
    end
    if isfinite(metrics.invalid_index)
        fprintf('Premier index invalide detecte : %d\n', metrics.invalid_index);
    end
end

function cfg = build_config(mode_name)
    cfg.mode_name = lower(string(mode_name));
    cfg.is_batch = ~usejava('desktop');

    cfg.figure_position = [90 60 1180 760];
    cfg.terrain_resolution = 36;
    cfg.sky_resolution = 20;
    cfg.show_decor = false;
    cfg.frame_rate = 18;
    cfg.video_quality = 60;
    cfg.export_video = false;
    cfg.capture_divisor = 90;
    cfg.min_step = 4;
    cfg.sim_t_end = 18;
    cfg.pause_dt = 0;
    cfg.vehicle_name = 'vega_c';
    cfg.sim_dt = 1.0;
    cfg.max_frames = inf;
    cfg.render_enabled = true;
    cfg.max_speed = 4500;
    cfg.max_angular_rate = deg2rad(20);
    cfg.max_control_moment = 2.0e6;
    cfg.stop_before_avum = true;
    cfg.stop_at_stage3_end = false;
    cfg.target_altitude = 250e3;
    cfg.enable_satellite_release = true;
    cfg.satellite_name = 'Satellite deploye';
    cfg.deorbit_horizontal_scale = 0.35;
    cfg.deorbit_vertical_speed = -250;
    cfg.satellite_horizontal_scale = 1.05;
    cfg.satellite_vertical_scale = 0.15;
    cfg.satellite_gravity = 0.12;
    cfg.satellite_min_altitude = 0.92 * cfg.target_altitude;
    cfg.satellite_turn_rate = deg2rad(0.18);
    cfg.satellite_color = [0.05 0.78 0.95];
    cfg.satellite_label_duration = 12;

    switch cfg.mode_name
        case "debug"
            cfg.export_video = false;
            cfg.show_decor = false;
            cfg.capture_divisor = 200;
            cfg.min_step = 12;
            cfg.sim_t_end = 80;
            cfg.figure_position = [100 80 960 620];
            cfg.terrain_resolution = 20;
            cfg.max_frames = 10;
            cfg.sim_dt = 0.5;
        case "no_render"
            cfg.export_video = false;
            cfg.show_decor = false;
            cfg.capture_divisor = 200;
            cfg.min_step = 20;
            cfg.sim_t_end = 520;
            cfg.figure_position = [100 80 960 620];
            cfg.terrain_resolution = 16;
            cfg.max_frames = 1;
            cfg.render_enabled = false;
            cfg.sim_dt = 0.25;
        case "preview"
            cfg.export_video = false;
            cfg.show_decor = false;
            cfg.capture_divisor = 120;
            cfg.sim_t_end = 520;
            cfg.sim_dt = 0.25;
        case "video"
            cfg.export_video = true;
            cfg.show_decor = false;
            cfg.capture_divisor = 120;
            cfg.frame_rate = 15;
            cfg.video_quality = 50;
            cfg.sim_t_end = 520;
            cfg.sim_dt = 0.25;
        case "ultra_light"
            cfg.export_video = false;
            cfg.show_decor = false;
            cfg.capture_divisor = 160;
            cfg.min_step = 6;
            cfg.sim_t_end = 320;
            cfg.figure_position = [100 80 960 620];
            cfg.terrain_resolution = 24;
            cfg.sim_dt = 0.25;
        case "full"
            cfg.export_video = true;
            cfg.show_decor = true;
            cfg.capture_divisor = 100;
            cfg.min_step = 3;
            cfg.frame_rate = 20;
            cfg.video_quality = 75;
            cfg.sim_t_end = 800;
            cfg.sim_dt = 0.25;
        otherwise
            error('Mode inconnu: %s. Utilisez debug, no_render, preview, video, ultra_light ou full.', cfg.mode_name);
    end
end

function paths = build_paths()
    base_dir = fileparts(mfilename('fullpath'));
    project_dir = fileparts(fileparts(base_dir));
    paths.videos_dir = fullfile(project_dir, 'out', 'videos');
    paths.images_dir = fullfile(project_dir, 'out', 'images');
    paths.logs_dir = fullfile(project_dir, 'out', 'logs');
end

function ensure_output_dirs(paths)
    if ~exist(paths.videos_dir, 'dir')
        mkdir(paths.videos_dir);
    end
    if ~exist(paths.images_dir, 'dir')
        mkdir(paths.images_dir);
    end
    if ~exist(paths.logs_dir, 'dir')
        mkdir(paths.logs_dir);
    end
end

function data = simulate_open_loop_3d(cfg)
    rocket = vega_c_parameters();
    g = 9.81;
    cw = 8.0e4;
    h_scale = 8500;
    rho0 = 1.225;
    Cd = 0.45;
    reference_area = pi * (rocket.body_diameter / 2)^2;

    dt = cfg.sim_dt;
    Tend = cfg.sim_t_end;
    t = 0:dt:Tend;
    N = numel(t);
    tburn = rocket.t_stage1_end + rocket.t_stage2_burn + rocket.t_stage3_burn;
    tilt_max = deg2rad(75);

    x = zeros(1, N); y = zeros(1, N); z = zeros(1, N);
    vx = zeros(1, N); vy = zeros(1, N); vz = zeros(1, N);
    phi = zeros(1, N); theta = zeros(1, N); psi = zeros(1, N);
    p = zeros(1, N); q = zeros(1, N); r = zeros(1, N);
    Tcmd = zeros(1, N);
    m_hist = zeros(1, N);
    stage_name = strings(1, N);
    sat_x = NaN(1, N); sat_y = NaN(1, N); sat_z = NaN(1, N);

    theta_ref = zeros(1, N);
    psi_ref = zeros(1, N);

    theta_ref(t >= 15) = deg2rad(5);
    theta_ref(t >= 70) = deg2rad(18);
    theta_ref(t >= 140) = deg2rad(35);
    theta_ref(t >= 220) = deg2rad(52);
    theta_ref(t >= 300) = deg2rad(63);
    theta_ref(t >= 360) = deg2rad(70);

    psi_ref(t >= 60) = deg2rad(1);
    psi_ref(t >= 180) = deg2rad(2);
    psi_ref(t >= 260) = deg2rad(3);

    wx = zeros(1, N); wy = zeros(1, N); wz = zeros(1, N);
    mp = zeros(1, N); mq = zeros(1, N); mr = zeros(1, N);

    for k = 1:N
        tk = t(k);
        if tk >= 2.5 && tk < 5.5
            wx(k) = 0.6;
        end
        if tk >= 6 && tk < 9
            wy(k) = -0.5;
        end
        if tk >= 7 && tk < 11
            wz(k) = -0.8;
        end
        if tk >= 3 && tk < 6
            mq(k) = 0.05 * sin(2 * pi * 0.8 * tk);
        end
        if tk >= 8 && tk < 12
            mr(k) = 0.04 * sin(2 * pi * 0.5 * tk);
        end
    end

    vx(1) = 0;
    vy(1) = 0;
    vz(1) = 2;
    m_hist(1) = rocket.m0;

    impact_idx = N;
    termination_reason = "simulation_complete";
    invalid_index = NaN;
    satellite_released = false;
    satellite_release_idx = NaN;
    satellite_release_time = NaN;
    launcher_mass_after_release = NaN;
    launcher_impacted = false;
    launcher_impact_idx = NaN;
    launcher_impact_time = NaN;
    sat_vx = NaN;
    sat_vy = NaN;
    sat_vz = NaN;
    sat_heading = NaN;
    sat_speed_xy = NaN;

    for k = 2:N
        tk = t(k);

        if launcher_impacted
            x(k) = x(k - 1);
            y(k) = y(k - 1);
            z(k) = 0;
            vx(k) = 0;
            vy(k) = 0;
            vz(k) = 0;
            p(k) = 0;
            q(k) = 0;
            r(k) = 0;
            phi(k) = phi(k - 1);
            theta(k) = theta(k - 1);
            psi(k) = psi(k - 1);
            Tcmd(k) = 0;
            m_hist(k) = m_hist(k - 1);
            stage_name(k) = "Lanceur au sol";

            sat_heading = sat_heading + cfg.satellite_turn_rate * dt;
            sat_vx = sat_speed_xy * cos(sat_heading);
            sat_vy = sat_speed_xy * sin(sat_heading);
            sat_vz = sat_vz - cfg.satellite_gravity * dt;
            sat_x(k) = sat_x(k - 1) + sat_vx * dt;
            sat_y(k) = sat_y(k - 1) + sat_vy * dt;
            sat_z(k) = max(sat_z(k - 1) + sat_vz * dt, cfg.satellite_min_altitude);
            continue;
        end

        if cfg.stop_at_stage3_end && tk >= rocket.t_stage3_end
            impact_idx = max(1, k - 1);
            termination_reason = "stage3_end";
            break;
        end

        if cfg.stop_before_avum && tk >= rocket.stage4.t_start && ~satellite_released
            impact_idx = max(1, k - 1);
            termination_reason = "mission_profile_end";
            break;
        end

        phi_ref = deg2rad(3) * sin(2 * pi * 0.35 * tk);
        theta_ref_k = clamp(theta_ref(k), -tilt_max, tilt_max);
        psi_ref_k = psi_ref(k);

        [Tcmd(k), current_mass, stage_info] = vega_c_thrust_mass(tk, rocket);
        if satellite_released
            Tcmd(k) = 0;
            current_mass = launcher_mass_after_release;
            stage_info = 'Retour balistique';
        end
        m_hist(k) = current_mass;
        stage_name(k) = string(stage_info);
        if Tcmd(k) <= 0 && tk > rocket.t_stage3_end && z(k - 1) <= 0
            impact_idx = k - 1;
            break;
        end

        if tk <= tburn
            Mx_ref = 6.0e4 * (phi_ref - phi(k - 1)) - 1.2e5 * p(k - 1);
            My_ref = 8.5e4 * (theta_ref_k - theta(k - 1)) - 1.6e5 * q(k - 1);
            Mz_ref = 5.5e4 * wrap_to_pi_local(psi_ref_k - psi(k - 1)) - 9.5e4 * r(k - 1);
        else
            Mx_ref = 0;
            My_ref = 0;
            Mz_ref = 0;
        end
        Mx_ref = clamp(Mx_ref, -cfg.max_control_moment, cfg.max_control_moment);
        My_ref = clamp(My_ref, -cfg.max_control_moment, cfg.max_control_moment);
        Mz_ref = clamp(Mz_ref, -cfg.max_control_moment, cfg.max_control_moment);

        R = eulerZYX(psi(k - 1), theta(k - 1), phi(k - 1));
        thrust_world = R * [0; 0; Tcmd(k)];

        vel = [vx(k - 1); vy(k - 1); vz(k - 1)];
        rho = rho0 * exp(-max(z(k - 1), 0) / h_scale);
        drag = 0.5 * rho * Cd * reference_area * norm(vel) * vel;
        acc = (thrust_world - drag) / current_mass - [0; 0; g] + [wx(k); wy(k); wz(k)] / current_mass;

        omega = [p(k - 1); q(k - 1); r(k - 1)];
        tau = [Mx_ref; My_ref; Mz_ref] + [mp(k); mq(k); mr(k)];
        I = cylinder_inertia(current_mass, rocket.body_length, rocket.body_diameter);
        omega_dot = I \ (tau - cross(omega, I * omega) - cw * omega);
        euler_dot = euler_rates(phi(k - 1), theta(k - 1), omega);

        vx(k) = vx(k - 1) + acc(1) * dt;
        vy(k) = vy(k - 1) + acc(2) * dt;
        vz(k) = vz(k - 1) + acc(3) * dt;
        speed_k = norm([vx(k); vy(k); vz(k)]);
        if speed_k > cfg.max_speed
            vel_scale = cfg.max_speed / speed_k;
            vx(k) = vx(k) * vel_scale;
            vy(k) = vy(k) * vel_scale;
            vz(k) = vz(k) * vel_scale;
        end

        x(k) = x(k - 1) + vx(k) * dt;
        y(k) = y(k - 1) + vy(k) * dt;
        z(k) = z(k - 1) + vz(k) * dt;

        if satellite_released
            sat_heading = sat_heading + cfg.satellite_turn_rate * dt;
            sat_vx = sat_speed_xy * cos(sat_heading);
            sat_vy = sat_speed_xy * sin(sat_heading);
            sat_vz = sat_vz - cfg.satellite_gravity * dt;
            sat_x(k) = sat_x(k - 1) + sat_vx * dt;
            sat_y(k) = sat_y(k - 1) + sat_vy * dt;
            sat_z(k) = max(sat_z(k - 1) + sat_vz * dt, cfg.satellite_min_altitude);
        end

        p(k) = p(k - 1) + omega_dot(1) * dt;
        q(k) = q(k - 1) + omega_dot(2) * dt;
        r(k) = r(k - 1) + omega_dot(3) * dt;
        p(k) = clamp(p(k), -cfg.max_angular_rate, cfg.max_angular_rate);
        q(k) = clamp(q(k), -cfg.max_angular_rate, cfg.max_angular_rate);
        r(k) = clamp(r(k), -cfg.max_angular_rate, cfg.max_angular_rate);

        euler_dot = euler_rates(phi(k - 1), theta(k - 1), [p(k); q(k); r(k)]);
        phi(k) = phi(k - 1) + euler_dot(1) * dt;
        theta(k) = clamp(theta(k - 1) + euler_dot(2) * dt, -tilt_max, tilt_max);
        psi(k) = wrap_to_pi_local(psi(k - 1) + euler_dot(3) * dt);

        state_k = [x(k), y(k), z(k), vx(k), vy(k), vz(k), ...
            p(k), q(k), r(k), phi(k), theta(k), psi(k), current_mass];
        if any(~isfinite(state_k))
            impact_idx = max(1, k - 1);
            termination_reason = "invalid_state";
            invalid_index = k;
            break;
        end

        if z(k) <= 0 && k > 20 && tk > 30 && ~satellite_released
            z(k) = 0;
            impact_idx = k;
            termination_reason = "ground_impact";
            break;
        end

        if cfg.enable_satellite_release && ~satellite_released && z(k) >= cfg.target_altitude
            satellite_released = true;
            satellite_release_idx = k;
            satellite_release_time = tk;
            sat_x(k) = x(k);
            sat_y(k) = y(k);
            sat_z(k) = z(k);
            sat_speed_xy = cfg.satellite_horizontal_scale * hypot(vx(k), vy(k));
            sat_heading = atan2(vy(k), vx(k));
            sat_vx = sat_speed_xy * cos(sat_heading);
            sat_vy = sat_speed_xy * sin(sat_heading);
            sat_vz = cfg.satellite_vertical_scale * vz(k);
            launcher_mass_after_release = max(current_mass - rocket.payload_mass, rocket.stage4.dry + rocket.interstage_mass);
            vx(k) = cfg.deorbit_horizontal_scale * vx(k);
            vy(k) = cfg.deorbit_horizontal_scale * vy(k);
            vz(k) = min(vz(k), cfg.deorbit_vertical_speed);
            m_hist(k) = launcher_mass_after_release;
            stage_name(k) = "Separation satellite";
        end

        if satellite_released && z(k) <= 0 && tk > satellite_release_time
            z(k) = 0;
            launcher_impacted = true;
            launcher_impact_idx = k;
            launcher_impact_time = tk;
            termination_reason = "satellite_deployed_then_ground_impact";
        end
    end

    data.t = t(1:impact_idx);
    data.x = x(1:impact_idx);
    data.y = y(1:impact_idx);
    data.z = z(1:impact_idx);
    data.vx = vx(1:impact_idx);
    data.vy = vy(1:impact_idx);
    data.vz = vz(1:impact_idx);
    data.phi = phi(1:impact_idx);
    data.theta = theta(1:impact_idx);
    data.psi = psi(1:impact_idx);
    data.Tcmd = Tcmd(1:impact_idx);
    data.tburn = tburn;
    data.mass = m_hist(1:impact_idx);
    data.stage_name = stage_name(1:impact_idx);
    data.vehicle_name = rocket.name;
    data.termination_reason = termination_reason;
    data.invalid_index = invalid_index;
    data.satellite_released = satellite_released;
    data.satellite_release_idx = satellite_release_idx;
    data.satellite_release_time = satellite_release_time;
    data.satellite_name = cfg.satellite_name;
    data.sat_x = sat_x(1:impact_idx);
    data.sat_y = sat_y(1:impact_idx);
    data.sat_z = sat_z(1:impact_idx);
    data.launcher_impacted = launcher_impacted;
    data.launcher_impact_idx = launcher_impact_idx;
    data.launcher_impact_time = launcher_impact_time;
end

function render_outputs(data, metrics, cfg, paths)
    if cfg.is_batch && cfg.export_video
        export_batch_video(data, metrics, cfg, paths);
        return;
    end
    if cfg.is_batch
        export_static_scene(data, metrics, cfg, paths);
        return;
    end
    fig = create_scene(data, cfg);
    animate_scene(fig, data, metrics, cfg, paths);
end

function export_static_scene(data, metrics, cfg, paths)
    rocket = vega_c_parameters();
    sat_color = cfg.satellite_color;
    fig = figure('Position', cfg.figure_position, 'Color', 'w', ...
        'Visible', 'off');
    ax = axes(fig);
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    view(ax, 36, 24);
    xlabel(ax, 'X (m)');
    ylabel(ax, 'Y (m)');
    zlabel(ax, 'Z (m)');
    title(ax, sprintf('Modele Vega - %s - %s', upper(char(cfg.mode_name)), rocket.name));
    set(ax, 'Color', [0.78 0.88 0.98]);

    plot3(ax, data.x, data.y, data.z, 'r-', 'LineWidth', 2);
    plot3(ax, data.x(1), data.y(1), data.z(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 7);
    plot3(ax, data.x(end), data.y(end), data.z(end), 'rx', 'LineWidth', 2, 'MarkerSize', 10);
    if data.satellite_released
        valid_sat = isfinite(data.sat_x) & isfinite(data.sat_y) & isfinite(data.sat_z);
        plot3(ax, data.sat_x(valid_sat), data.sat_y(valid_sat), data.sat_z(valid_sat), ...
            '--', 'Color', sat_color, 'LineWidth', 2.2);
        plot3(ax, data.sat_x(find(valid_sat, 1, 'last')), data.sat_y(find(valid_sat, 1, 'last')), ...
            data.sat_z(find(valid_sat, 1, 'last')), 'p', 'Color', sat_color, ...
            'MarkerFaceColor', sat_color, 'MarkerSize', 10);
        text(ax, data.sat_x(data.satellite_release_idx), data.sat_y(data.satellite_release_idx), ...
            data.sat_z(data.satellite_release_idx), 'Satellite deploye', ...
            'Color', sat_color, 'FontWeight', 'bold', 'VerticalAlignment', 'bottom');
    end

    xlim(ax, [min(data.x) - 3, max(data.x) + 3]);
    ylim(ax, [min(data.y) - 3, max(data.y) + 3]);
    zlim(ax, [0, max(data.z) + 4]);

    text(ax, data.x(end), data.y(end), max(data.z), sprintf([ ...
        'Vehicule : %s\nTemps : %.2f s\nAltitude max : %.2f m\nPortee : %.2f m\nFin : %s'], ...
        rocket.name, metrics.flight_time, metrics.max_altitude, ...
        metrics.range_xy, char(metrics.termination_reason)), ...
        'VerticalAlignment', 'bottom', 'FontWeight', 'bold');

    saveas(fig, fullfile(paths.images_dir, 'modele_vega.png'));
    close(fig);
end

function export_batch_video(data, metrics, cfg, paths)
    rocket = vega_c_parameters();
    sat_color = cfg.satellite_color;
    fig = figure('Position', cfg.figure_position, 'Color', 'w', ...
        'Visible', 'off');
    ax = axes(fig);
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    view(ax, 36, 24);
    xlabel(ax, 'X (m)');
    ylabel(ax, 'Y (m)');
    zlabel(ax, 'Z (m)');
    title(ax, sprintf('Modele Vega - %s - %s', upper(char(cfg.mode_name)), rocket.name));
    set(ax, 'Color', [0.78 0.88 0.98]);

    plot3(ax, data.x, data.y, data.z, '--', 'Color', [0.7 0.7 0.7], 'LineWidth', 1.1);
    hTrace = plot3(ax, data.x(1), data.y(1), data.z(1), 'r-', 'LineWidth', 2);
    hBody = plot3(ax, NaN, NaN, NaN, 'Color', [0.08 0.20 0.82], 'LineWidth', 4);
    hNose1 = plot3(ax, NaN, NaN, NaN, 'Color', [0.95 0.95 0.98], 'LineWidth', 2.5);
    hNose2 = plot3(ax, NaN, NaN, NaN, 'Color', [0.95 0.95 0.98], 'LineWidth', 2.5);
    hFin1 = plot3(ax, NaN, NaN, NaN, 'Color', [0.92 0.18 0.18], 'LineWidth', 2.2);
    hFin2 = plot3(ax, NaN, NaN, NaN, 'Color', [0.92 0.18 0.18], 'LineWidth', 2.2);
    hCenter = plot3(ax, data.x(1), data.y(1), data.z(1), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
    hSatTrace = plot3(ax, NaN, NaN, NaN, '--', 'Color', sat_color, 'LineWidth', 2.2);
    hSat = plot3(ax, NaN, NaN, NaN, 'p', 'Color', sat_color, 'MarkerFaceColor', sat_color, 'MarkerSize', 10);
    hSatLabel = text(ax, NaN, NaN, NaN, 'Satellite deploye', 'Color', sat_color, ...
        'FontWeight', 'bold', 'Visible', 'off');
    hText = text(ax, data.x(1), data.y(1), max(data.z) + 0.02 * max(max(data.z), 1), '', ...
        'FontWeight', 'bold', 'VerticalAlignment', 'bottom');

    xlim(ax, [min(data.x) - 3, max(data.x) + 3]);
    ylim(ax, [min(data.y) - 3, max(data.y) + 3]);
    zlim(ax, [0, max(data.z) + 4]);

    temp_video_path = fullfile(paths.videos_dir, 'modele_vega_tmp.mp4');
    final_video_path = fullfile(paths.videos_dir, 'modele_vega.mp4');
    if exist(temp_video_path, 'file')
        delete(temp_video_path);
    end

    vw = VideoWriter(temp_video_path, 'MPEG-4');
    vw.FrameRate = cfg.frame_rate;
    vw.Quality = cfg.video_quality;
    open(vw);

    N = numel(data.t);
    step = max(cfg.min_step, ceil(N / cfg.capture_divisor));
    frame_indices = unique([1:step:N, N]);
    L = max(rocket.height * 0.36, 9.0);
    W = max(rocket.body_diameter * 0.60, 1.8);

    for idx = 1:numel(frame_indices)
        k = frame_indices(idx);
        R = eulerZYX(data.psi(k), data.theta(k), data.phi(k));
        body_pts = rocket_wireframe_points([data.x(k); data.y(k); data.z(k)], R, L, W);
        set(hTrace, 'XData', data.x(1:k), 'YData', data.y(1:k), 'ZData', data.z(1:k));
        set(hBody, 'XData', body_pts.body(1, :), 'YData', body_pts.body(2, :), 'ZData', body_pts.body(3, :));
        set(hNose1, 'XData', body_pts.nose1(1, :), 'YData', body_pts.nose1(2, :), 'ZData', body_pts.nose1(3, :));
        set(hNose2, 'XData', body_pts.nose2(1, :), 'YData', body_pts.nose2(2, :), 'ZData', body_pts.nose2(3, :));
        set(hFin1, 'XData', body_pts.fin1(1, :), 'YData', body_pts.fin1(2, :), 'ZData', body_pts.fin1(3, :));
        set(hFin2, 'XData', body_pts.fin2(1, :), 'YData', body_pts.fin2(2, :), 'ZData', body_pts.fin2(3, :));
        set(hCenter, 'XData', data.x(k), 'YData', data.y(k), 'ZData', data.z(k));
        if data.satellite_released
            valid_sat = isfinite(data.sat_x(1:k)) & isfinite(data.sat_y(1:k)) & isfinite(data.sat_z(1:k));
            set(hSatTrace, 'XData', data.sat_x(valid_sat), 'YData', data.sat_y(valid_sat), 'ZData', data.sat_z(valid_sat));
            last_sat = find(valid_sat, 1, 'last');
            if ~isempty(last_sat)
                set(hSat, 'XData', data.sat_x(last_sat), 'YData', data.sat_y(last_sat), 'ZData', data.sat_z(last_sat));
            end
            if k >= data.satellite_release_idx && data.t(k) <= data.satellite_release_time + cfg.satellite_label_duration
                set(hSatLabel, 'Position', [data.sat_x(data.satellite_release_idx), data.sat_y(data.satellite_release_idx), ...
                    data.sat_z(data.satellite_release_idx)], 'Visible', 'on');
            else
                set(hSatLabel, 'Visible', 'off');
            end
        end
        set(hText, 'Position', [data.x(k), data.y(k), max(data.z) + 0.02 * max(max(data.z), 1)], ...
            'String', sprintf('t = %.2f s | z = %.2f m | portee = %.2f m', ...
            data.t(k), data.z(k), hypot(data.x(k), data.y(k))));
        drawnow;
        writeVideo(vw, getframe(fig));
    end

    saveas(fig, fullfile(paths.images_dir, 'modele_vega.png'));
    close(vw);
    if exist(final_video_path, 'file')
        delete(final_video_path);
    end
    movefile(temp_video_path, final_video_path, 'f');
    close(fig);
end

function fig = create_scene(data, cfg)
    x = data.x; y = data.y; z = data.z;
    rocket = vega_c_parameters();
    sat_color = cfg.satellite_color;

    fig = figure('Position', cfg.figure_position, 'Color', 'w', ...
        'Visible', figure_visibility(cfg.is_batch));
    tl = tiledlayout(fig, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax3 = nexttile(tl, 1, [2 2]);
    hold(ax3, 'on'); grid(ax3, 'on'); box(ax3, 'on');
    view(ax3, 36, 24);
    xlabel(ax3, 'X (m)');
    ylabel(ax3, 'Y (m)');
    zlabel(ax3, 'Z (m)');
    title(ax3, sprintf('Modele Vega - %s - %s', upper(char(cfg.mode_name)), rocket.name));
    set(ax3, 'Color', [0.78 0.88 0.98]);

    xmin = min(x) - 3; xmax = max(x) + 3;
    ymin = min(y) - 3; ymax = max(y) + 3;
    zmax = max(z) + 4;

    xlim(ax3, [xmin xmax]);
    ylim(ax3, [ymin ymax]);
    zlim(ax3, [0 zmax]);
    daspect(ax3, [1 1 1]);

    [Xg, Yg] = meshgrid(linspace(xmin, xmax, cfg.terrain_resolution), ...
                        linspace(ymin, ymax, cfg.terrain_resolution));
    terrain_shape = 0.18 * sin(0.12 * Xg) .* cos(0.18 * Yg) + ...
        0.12 * exp(-((Xg - mean(x)).^2 + (Yg - mean(y)).^2) / 180);
    Zg = max(0, terrain_shape);
    surf(ax3, Xg, Yg, Zg, 'FaceColor', [0.42 0.62 0.30], ...
        'EdgeColor', 'none', 'FaceAlpha', 1.0);
    colormap(ax3, summer);

    if cfg.show_decor
        add_scene_decor(ax3, xmin, xmax, ymin, ymax, zmax, x, y, cfg);
    end

    plot3(ax3, x, y, z, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.2);

    handles.ax3 = ax3;
    handles.hTrace = plot3(ax3, NaN, NaN, NaN, 'r', 'LineWidth', 2);
    handles.hBody = plot3(ax3, NaN, NaN, NaN, 'Color', [0.08 0.20 0.82], 'LineWidth', 4);
    handles.hNose1 = plot3(ax3, NaN, NaN, NaN, 'Color', [0.95 0.95 0.98], 'LineWidth', 2.5);
    handles.hNose2 = plot3(ax3, NaN, NaN, NaN, 'Color', [0.95 0.95 0.98], 'LineWidth', 2.5);
    handles.hFin1 = plot3(ax3, NaN, NaN, NaN, 'Color', [0.92 0.18 0.18], 'LineWidth', 2.2);
    handles.hFin2 = plot3(ax3, NaN, NaN, NaN, 'Color', [0.92 0.18 0.18], 'LineWidth', 2.2);
    handles.hCenter = plot3(ax3, x(1), y(1), z(1), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
    handles.hThrust = quiver3(ax3, x(1), y(1), z(1), 0, 0, 0, 0, ...
        'Color', [0 0.5 0], 'LineWidth', 2, 'MaxHeadSize', 0.8);
    handles.hImpact = plot3(ax3, x(end), y(end), z(end), 'x', 'Color', [0.8 0.2 0.1], ...
        'LineWidth', 2, 'MarkerSize', 10, 'Visible', 'off');
    handles.hSatTrace = plot3(ax3, NaN, NaN, NaN, '--', 'Color', sat_color, 'LineWidth', 2.2);
    handles.hSatellite = plot3(ax3, NaN, NaN, NaN, 'p', 'Color', sat_color, ...
        'MarkerFaceColor', sat_color, 'MarkerSize', 10);
    handles.hSatLabel = text(ax3, NaN, NaN, NaN, 'Satellite deploye', ...
        'Color', sat_color, 'FontWeight', 'bold', 'Visible', 'off');
    handles.txt = text(ax3, xmax - 0.5, ymax - 0.5, zmax - 0.3, '', ...
        'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', 'FontWeight', 'bold');

    axAlt = nexttile(tl, 3);
    hold(axAlt, 'on'); grid(axAlt, 'on'); box(axAlt, 'on');
    handles.hAlt = plot(axAlt, NaN, NaN, 'b', 'LineWidth', 1.8);
    handles.hSpeed = plot(axAlt, NaN, NaN, 'r', 'LineWidth', 1.4);
    xlabel(axAlt, 'Temps (s)');
    ylabel(axAlt, 'Altitude / Vitesse');
    title(axAlt, 'Evolution verticale');
    legend(axAlt, 'Altitude z', 'Vitesse |v|', 'Location', 'best');

    axAtt = nexttile(tl, 5);
    hold(axAtt, 'on'); grid(axAtt, 'on'); box(axAtt, 'on');
    handles.hPhi = plot(axAtt, NaN, NaN, 'm', 'LineWidth', 1.6);
    handles.hTheta = plot(axAtt, NaN, NaN, 'c', 'LineWidth', 1.6);
    handles.hPsi = plot(axAtt, NaN, NaN, 'k', 'LineWidth', 1.6);
    xlabel(axAtt, 'Temps (s)');
    ylabel(axAtt, 'Angle (deg)');
    title(axAtt, 'Attitude pendant le vol');
    legend(axAtt, 'phi', 'theta', 'psi', 'Location', 'best');

    axMass = nexttile(tl, 6);
    hold(axMass, 'on'); grid(axMass, 'on'); box(axMass, 'on');
    yyaxis(axMass, 'left');
    handles.hMass = plot(axMass, NaN, NaN, 'Color', [0.45 0.1 0.65], 'LineWidth', 1.7);
    ylabel(axMass, 'Masse (kg)');
    yyaxis(axMass, 'right');
    handles.hThrustHist = plot(axMass, NaN, NaN, 'Color', [0.0 0.55 0.2], 'LineWidth', 1.5);
    ylabel(axMass, 'Poussee (N)');
    xlabel(axMass, 'Temps (s)');
    title(axMass, 'Masse et poussee');
    legend(axMass, 'Masse', 'Poussee', 'Location', 'best');

    setappdata(fig, 'scene_handles', handles);
end

function add_scene_decor(ax3, xmin, xmax, ymin, ymax, zmax, x, y, cfg)
    [Xs, Ys] = meshgrid(linspace(xmin, xmax, cfg.sky_resolution), ...
                        linspace(ymin, ymax, cfg.sky_resolution));
    Zs1 = (zmax + 1.0) * ones(size(Xs));
    Zs2 = (zmax + 2.2) * ones(size(Xs));
    surf(ax3, Xs, Ys, Zs1, 'FaceColor', [0.82 0.91 1.00], ...
        'EdgeColor', 'none', 'FaceAlpha', 0.18);
    surf(ax3, Xs, Ys, Zs2, 'FaceColor', [0.93 0.97 1.00], ...
        'EdgeColor', 'none', 'FaceAlpha', 0.10);

    cloud_centers = [xmin + 0.20 * (xmax - xmin), ymin + 0.20 * (ymax - ymin), zmax - 1.2; ...
                     xmin + 0.62 * (xmax - xmin), ymin + 0.68 * (ymax - ymin), zmax - 0.8; ...
                     xmin + 0.78 * (xmax - xmin), ymin + 0.30 * (ymax - ymin), zmax - 1.6];
    for ic = 1:size(cloud_centers, 1)
        draw_cloud(ax3, cloud_centers(ic, :), 1.8 + 0.2 * ic);
    end

    tree_positions = [xmin + 2.5, ymin + 1.5; ...
                      xmin + 6.0, ymax - 2.0; ...
                      xmax - 3.5, ymin + 2.2; ...
                      xmax - 6.5, ymax - 1.8; ...
                      mean(x), ymin + 1.0];
    for itree = 1:size(tree_positions, 1)
        draw_tree(ax3, tree_positions(itree, 1), tree_positions(itree, 2), 0);
    end
end

function animate_scene(fig, data, metrics, cfg, paths)
    handles = getappdata(fig, 'scene_handles');
    assert_valid_scene_handles(handles);
    t = data.t;
    x = data.x; y = data.y; z = data.z;
    vx = data.vx; vy = data.vy; vz = data.vz;
    phi = data.phi; theta = data.theta; psi = data.psi;
    Tcmd = data.Tcmd;
    tburn = data.tburn;
    mass = data.mass;
    stage_name = data.stage_name;
    sat_x = data.sat_x; sat_y = data.sat_y; sat_z = data.sat_z;
    N = numel(t);

    vw = [];
    temp_video_path = fullfile(paths.videos_dir, 'modele_vega_tmp.mp4');
    final_video_path = fullfile(paths.videos_dir, 'modele_vega.mp4');
    if cfg.export_video
        if exist(temp_video_path, 'file')
            delete(temp_video_path);
        end
        vw = VideoWriter(temp_video_path, 'MPEG-4');
        vw.FrameRate = cfg.frame_rate;
        vw.Quality = cfg.video_quality;
        open(vw);
    end

    rocket = vega_c_parameters();
    L = max(rocket.height * 0.36, 9.0);
    W = max(rocket.body_diameter * 0.60, 1.8);
    thrust_scale = 8e-5;

    xTrace = zeros(1, N);
    yTrace = zeros(1, N);
    zTrace = zeros(1, N);

    step = 2;
    if cfg.is_batch
        step = max(cfg.min_step, ceil(N / cfg.capture_divisor));
    end

    frame_indices = unique([1:step:N, N]);
    if isfinite(cfg.max_frames)
        frame_indices = frame_indices(1:min(numel(frame_indices), cfg.max_frames));
    end

    for idx = 1:numel(frame_indices)
        assert_valid_scene_handles(handles);
        k = frame_indices(idx);
        R = eulerZYX(psi(k), theta(k), phi(k));

        c = [x(k); y(k); z(k)];
        body_pts = rocket_wireframe_points(c, R, L, W);

        set(handles.hBody, 'XData', body_pts.body(1, :), 'YData', body_pts.body(2, :), 'ZData', body_pts.body(3, :));
        set(handles.hNose1, 'XData', body_pts.nose1(1, :), 'YData', body_pts.nose1(2, :), 'ZData', body_pts.nose1(3, :));
        set(handles.hNose2, 'XData', body_pts.nose2(1, :), 'YData', body_pts.nose2(2, :), 'ZData', body_pts.nose2(3, :));
        set(handles.hFin1, 'XData', body_pts.fin1(1, :), 'YData', body_pts.fin1(2, :), 'ZData', body_pts.fin1(3, :));
        set(handles.hFin2, 'XData', body_pts.fin2(1, :), 'YData', body_pts.fin2(2, :), 'ZData', body_pts.fin2(3, :));
        set(handles.hCenter, 'XData', x(k), 'YData', y(k), 'ZData', z(k));

        xTrace(k) = x(k);
        yTrace(k) = y(k);
        zTrace(k) = z(k);
        valid = xTrace ~= 0 | yTrace ~= 0 | zTrace ~= 0;
        set(handles.hTrace, 'XData', xTrace(valid), 'YData', yTrace(valid), 'ZData', zTrace(valid));

        thrust_vec = R * [0; 0; thrust_scale * Tcmd(k)];
        set(handles.hThrust, 'XData', x(k), 'YData', y(k), 'ZData', z(k), ...
            'UData', thrust_vec(1), 'VData', thrust_vec(2), 'WData', thrust_vec(3));
        if data.satellite_released
            valid_sat = isfinite(sat_x(1:k)) & isfinite(sat_y(1:k)) & isfinite(sat_z(1:k));
            set(handles.hSatTrace, 'XData', sat_x(valid_sat), 'YData', sat_y(valid_sat), ...
                'ZData', sat_z(valid_sat));
            last_sat = find(valid_sat, 1, 'last');
            if ~isempty(last_sat)
                set(handles.hSatellite, 'XData', sat_x(last_sat), 'YData', sat_y(last_sat), ...
                    'ZData', sat_z(last_sat));
            end
            if k >= data.satellite_release_idx && t(k) <= data.satellite_release_time + cfg.satellite_label_duration
                set(handles.hSatLabel, 'Position', [sat_x(data.satellite_release_idx), sat_y(data.satellite_release_idx), ...
                    sat_z(data.satellite_release_idx)], 'Visible', 'on');
            else
                set(handles.hSatLabel, 'Visible', 'off');
            end
        end

        speed = sqrt(vx(k)^2 + vy(k)^2 + vz(k)^2);
        phase = flight_phase_label(t(k), tburn, k == N);
        active_stage = char(stage_name(k));
        if k == N
            set(handles.hImpact, 'Visible', 'on');
        end

        set(handles.txt, 'String', sprintf(['Vehicule : %s\nEtage actif : %s\nt = %.2f s\nx = %.2f  y = %.2f  z = %.2f m\n' ...
            'phi = %.1f deg  theta = %.1f deg  psi = %.1f deg\n' ...
            '|v| = %.2f m/s | %s\nPortee = %.2f m\nSatellite : %s'], ...
            rocket.name, active_stage, ...
            t(k), x(k), y(k), z(k), rad2deg(phi(k)), rad2deg(theta(k)), ...
            rad2deg(psi(k)), speed, phase, metrics.range_xy, logical_to_text(data.satellite_released && k >= data.satellite_release_idx)));

        set(handles.hAlt, 'XData', t(1:k), 'YData', z(1:k));
        set(handles.hSpeed, 'XData', t(1:k), 'YData', sqrt(vx(1:k).^2 + vy(1:k).^2 + vz(1:k).^2));
        yyaxis(get(handles.hMass, 'Parent'), 'left');
        set(handles.hMass, 'XData', t(1:k), 'YData', mass(1:k));
        yyaxis(get(handles.hMass, 'Parent'), 'right');
        set(handles.hThrustHist, 'XData', t(1:k), 'YData', Tcmd(1:k));
        set(handles.hPhi, 'XData', t(1:k), 'YData', rad2deg(phi(1:k)));
        set(handles.hTheta, 'XData', t(1:k), 'YData', rad2deg(theta(1:k)));
        set(handles.hPsi, 'XData', t(1:k), 'YData', rad2deg(psi(1:k)));

        drawnow limitrate nocallbacks;
        if cfg.export_video
            frame = getframe(fig);
            writeVideo(vw, frame);
        elseif ~cfg.is_batch && cfg.pause_dt > 0
            pause(cfg.pause_dt);
        end
    end

    saveas(fig, fullfile(paths.images_dir, 'modele_vega.png'));

    if cfg.export_video
        close(vw);
        if exist(final_video_path, 'file')
            delete(final_video_path);
        end
        if exist(temp_video_path, 'file')
            movefile(temp_video_path, final_video_path, 'f');
        end
    end

    close(fig);
end

function assert_valid_scene_handles(handles)
    required_fields = {'ax3', 'hTrace', 'hBody', 'hNose1', 'hNose2', 'hFin1', 'hFin2', 'hCenter', ...
        'hThrust', 'hImpact', 'hSatTrace', 'hSatellite', 'hSatLabel', 'txt', 'hAlt', 'hSpeed', 'hPhi', ...
        'hTheta', 'hPsi', 'hMass', 'hThrustHist'};

    for i = 1:numel(required_fields)
        field_name = required_fields{i};
        if ~isfield(handles, field_name) || ~isgraphics(handles.(field_name))
            error(['Le handle graphique "%s" est invalide ou a ete supprime. ' ...
                'Verifiez la figure, le layout et les objets de rendu.'], field_name);
        end
    end
end

function label = flight_phase_label(tk, tburn, is_final)
    label = 'Phase propulsee';
    if tk > tburn
        label = 'Phase balistique';
    end
    if is_final
        label = 'Impact au sol';
    end
end

function write_model_presentation(output_path)
    rocket = vega_c_parameters();
    fid = fopen(output_path, 'w');
    if fid < 0
        warning('Impossible de creer le texte de presentation : %s', output_path);
        return;
    end

    fprintf(fid, ['Le modele Vega developpe dans ce travail est calibre sur un profil simplifie ' ...
        'inspire du lanceur europeen Vega-C. Cette adaptation permet de passer d''un vehicule generique ' ...
        'a une representation numerique plus credibile d''une fusee reelle, en integrant des ordres de grandeur ' ...
        'coherents pour la masse au decollage, la geometrie, les etages propulsifs et les niveaux de poussee.\n\n']);

    fprintf(fid, ['Le vehicule considere est un lanceur d''environ %.1f metres de hauteur, de %.1f metres de diametre, ' ...
        'avec une masse au decollage de l''ordre de %.0f kg. Dans le modele, la sequence propulsive reprend la logique ' ...
        'generale de Vega-C a travers quatre segments fonctionnels : un premier etage de type P120C, un deuxieme etage ' ...
        'de type Zefiro-40, un troisieme etage de type Zefiro-9, puis un etage superieur de type AVUM+. Chaque etage est ' ...
        'decrit par une masse propulsive, une masse seche estimee et une poussee moyenne appliquee sur une duree de combustion simplifiee.\n\n'], ...
        rocket.height, rocket.body_diameter, rocket.m0);

    fprintf(fid, ['Cette representation reste volontairement simplifiee : elle ne constitue pas un simulateur orbital complet, ' ...
        'mais un modele 3D d''ascension inspire du reel. Son objectif principal est pedagogique et scientifique : ' ...
        'montrer comment des donnees techniques issues d''un lanceur reel peuvent etre injectees dans un modele dynamique tridimensionnel, ' ...
        'afin d''observer l''influence conjointe de la poussee, de la masse variable, de l''attitude et de la gravite sur la trajectoire.\n\n']);

    fprintf(fid, ['Dans la version actuelle du modele, une phase de separation simplifiee du satellite est introduite lorsque le lanceur atteint ' ...
        'l''altitude cible. A cet instant, la charge utile est consideree comme deployee dans l''espace sur une trajectoire propre, tandis que ' ...
        'le corps principal du lanceur poursuit une phase de retour balistique conduisant a sa retombee au sol. Cette extension permet de ' ...
        'representer plus clairement la logique d''une mission spatiale elementaire, en distinguant le mobile deploye et le lanceur apres separation.\n\n']);

    fprintf(fid, ['Dans cette perspective, le modele Vega enrichit sensiblement la qualite de l''analyse. Il permet de discuter les performances ' ...
        'de vol sur une base plus concrete, tout en conservant une structure de simulation suffisamment simple pour etre interpretable. ' ...
        'Il constitue ainsi une transition pertinente entre un modele academique de projectile propulse et une representation plus proche ' ...
        'des systemes de lancement reels, incluant une mise en espace simplifiee de la charge utile.\n']);

    fclose(fid);
end

function R = eulerZYX(psi, theta, phi)
    Rz = [cos(psi) -sin(psi) 0;
          sin(psi)  cos(psi) 0;
          0         0        1];

    Ry = [ cos(theta) 0 sin(theta);
           0          1 0;
          -sin(theta) 0 cos(theta)];

    Rx = [1 0         0;
          0 cos(phi) -sin(phi);
          0 sin(phi)  cos(phi)];

    R = Rz * Ry * Rx;
end

function eul_dot = euler_rates(phi, theta, omega)
    ctheta = max(cos(theta), 1e-3);
    T = [1, sin(phi) * tan(theta), cos(phi) * tan(theta);
         0, cos(phi),             -sin(phi);
         0, sin(phi) / ctheta,     cos(phi) / ctheta];
    eul_dot = T * omega;
end

function y = clamp(x, xmin, xmax)
    y = min(max(x, xmin), xmax);
end

function angle = wrap_to_pi_local(angle)
    angle = mod(angle + pi, 2 * pi) - pi;
end

function metrics = compute_flight_metrics(data)
    finite_mask = isfinite(data.x) & isfinite(data.y) & isfinite(data.z) & ...
        isfinite(data.vx) & isfinite(data.vy) & isfinite(data.vz) & isfinite(data.mass);
    valid_idx = find(finite_mask);
    if isempty(valid_idx)
        error('Aucun etat numeriquement valide n''a ete produit par la simulation.');
    end

    last_valid = valid_idx(end);
    speed = sqrt(data.vx(valid_idx).^2 + data.vy(valid_idx).^2 + data.vz(valid_idx).^2);

    if data.launcher_impacted && isfinite(data.launcher_impact_idx)
        metrics.flight_time = data.launcher_impact_time;
        metrics.range_xy = hypot(data.x(data.launcher_impact_idx), data.y(data.launcher_impact_idx));
        metrics.impact_x = data.x(data.launcher_impact_idx);
        metrics.impact_y = data.y(data.launcher_impact_idx);
        metrics.final_speed = norm([data.vx(data.launcher_impact_idx), data.vy(data.launcher_impact_idx), data.vz(data.launcher_impact_idx)]);
    else
        metrics.flight_time = data.t(last_valid);
        metrics.range_xy = hypot(data.x(last_valid), data.y(last_valid));
        metrics.impact_x = data.x(last_valid);
        metrics.impact_y = data.y(last_valid);
        metrics.final_speed = speed(end);
    end

    metrics.max_altitude = max(data.z(valid_idx));
    metrics.max_speed = max(speed);
    metrics.initial_mass = data.mass(1);
    metrics.final_mass = data.mass(last_valid);
    metrics.vehicle_name = data.vehicle_name;
    metrics.termination_reason = data.termination_reason;
    metrics.invalid_index = data.invalid_index;
    metrics.last_valid_index = last_valid;
    metrics.launcher_impacted = data.launcher_impacted;
    metrics.launcher_impact_time = data.launcher_impact_time;
    metrics.satellite_released = data.satellite_released;
    metrics.satellite_release_time = data.satellite_release_time;
    if data.satellite_released
        metrics.satellite_release_altitude = data.z(data.satellite_release_idx);
        metrics.satellite_release_range = hypot(data.x(data.satellite_release_idx), data.y(data.satellite_release_idx));
    else
        metrics.satellite_release_altitude = NaN;
        metrics.satellite_release_range = NaN;
    end
end

function write_metrics_log(log_path, metrics, cfg)
    fid = fopen(log_path, 'w');
    if fid < 0
        warning('Impossible de creer le fichier log : %s', log_path);
        return;
    end

    fprintf(fid, '===== MODELE VEGA =====\n');
    fprintf(fid, 'Vehicule                : %s\n', metrics.vehicle_name);
    fprintf(fid, 'Mode                    : %s\n', cfg.mode_name);
    fprintf(fid, 'Temps de vol            : %.4f s\n', metrics.flight_time);
    fprintf(fid, 'Portee horizontale      : %.4f m\n', metrics.range_xy);
    fprintf(fid, 'Impact X                : %.4f m\n', metrics.impact_x);
    fprintf(fid, 'Impact Y                : %.4f m\n', metrics.impact_y);
    fprintf(fid, 'Altitude maximale       : %.4f m\n', metrics.max_altitude);
    fprintf(fid, 'Vitesse maximale        : %.4f m/s\n', metrics.max_speed);
    fprintf(fid, 'Vitesse finale          : %.4f m/s\n', metrics.final_speed);
    fprintf(fid, 'Masse initiale          : %.4f kg\n', metrics.initial_mass);
    fprintf(fid, 'Masse finale            : %.4f kg\n', metrics.final_mass);
    fprintf(fid, 'Fin de simulation       : %s\n', metrics.termination_reason);
    fprintf(fid, 'Satellite deploye       : %s\n', logical_to_text(metrics.satellite_released));
    if metrics.satellite_released
        fprintf(fid, 'Temps separation sat.   : %.4f s\n', metrics.satellite_release_time);
        fprintf(fid, 'Altitude separation     : %.4f m\n', metrics.satellite_release_altitude);
        fprintf(fid, 'Portee separation       : %.4f m\n', metrics.satellite_release_range);
    end
    if isfinite(metrics.invalid_index)
        fprintf(fid, 'Premier index invalide  : %d\n', metrics.invalid_index);
    end
    fprintf(fid, 'Export video            : %s\n', logical_to_text(cfg.export_video));
    fprintf(fid, 'Decor complet           : %s\n', logical_to_text(cfg.show_decor));
    fprintf(fid, '============================\n');
    fclose(fid);
end

function rocket = vega_c_parameters()
    % Vega-C inspired dataset using official ESA / Arianespace values where
    % available. This 3D model remains simplified: it uses average stage
    % thrusts and a condensed atmospheric ascent representation.
    %
    % Official sources used:
    % ESA Vega-C page:
    % https://www.esa.int/Our_Activities/Space_Transportation/Launch_vehicles/Vega-C
    % Arianespace Vega-C User's Manual (Issue 0 Rev 0, May 2018):
    % https://www.arianespace.com/app/uploads/sites/4/2024/10/Vega-C-users-manual-Issue-0-Revision-0.pdf
    rocket.name = 'Vega-C (profil simplifie inspire du reel)';
    rocket.height = 35.0;               % m
    rocket.body_length = 35.0;          % m
    rocket.body_diameter = 3.0;         % m
    rocket.m0 = 210000.0;               % kg, ESA official liftoff mass
    rocket.reference_orbit = 'SSO / LEO light lift';

    rocket.payload_mass = 2300.0;       % kg, reference payload capability to SSO
    rocket.fairing_mass = 860.0;        % kg, simplified retained value
    rocket.interstage_mass = 2700.0;    % kg, residual structural mass estimate

    rocket.stage1.prop = 143600.0;      % kg, ESA official
    rocket.stage1.dry = 8000.0;         % kg, ESA official carbon structure
    rocket.stage1.thrust = 4.50e6;      % N, ESA official average thrust
    rocket.stage1.t_start = 0.0;
    rocket.stage1.t_end = 142.0;        % s, Arianespace ascent profile

    rocket.stage2.prop = 36200.0;       % kg, ESA official
    rocket.stage2.dry = 3500.0;         % kg, simplified estimate
    rocket.stage2.thrust = 1.304e6;     % N, ESA official average thrust
    rocket.stage2.t_start = 142.0;      % s
    rocket.stage2.t_end = 245.0;        % s, Arianespace ascent profile

    rocket.stage3.prop = 10000.0;       % kg, ESA official
    rocket.stage3.dry = 1200.0;         % kg, simplified estimate
    rocket.stage3.thrust = 3.13e5;      % N, ESA Vega Zefiro-9 reference thrust
    rocket.stage3.t_start = 249.0;      % s
    rocket.stage3.t_end = 369.0;        % s, simplified 120 s burn

    rocket.stage4.prop = 740.0;         % kg, ESA official AVUM+
    rocket.stage4.dry = 900.0;          % kg, simplified estimate
    rocket.stage4.thrust = 2.45e3;      % N, ESA official average thrust
    rocket.stage4.t_start = 448.0;      % s, Arianespace ascent profile
    rocket.stage4.t_end = 520.0;        % s, shortened first burn for this model

    rocket.t_stage1_end = rocket.stage1.t_end;
    rocket.t_stage2_burn = rocket.stage2.t_end - rocket.stage2.t_start;
    rocket.t_stage3_burn = rocket.stage3.t_end - rocket.stage3.t_start;
    rocket.t_stage3_end = rocket.stage3.t_end;
end

function display_vega_c_reference_table()
    rocket = vega_c_parameters();
    fprintf('\n===== TABLEAU DE REFERENCE VEGA-C =====\n');
    fprintf('Nom du vehicule        : %s\n', rocket.name);
    fprintf('Hauteur                : %.2f m\n', rocket.height);
    fprintf('Diametre corps         : %.2f m\n', rocket.body_diameter);
    fprintf('Masse au decollage     : %.0f kg\n', rocket.m0);
    fprintf('Charge utile ref.      : %.0f kg\n', rocket.payload_mass);
    fprintf('Reference mission      : %s\n', rocket.reference_orbit);
    fprintf('P120C                  : prop = %.0f kg | dry = %.0f kg | thrust = %.0f kN | burn = %.0f s\n', ...
        rocket.stage1.prop, rocket.stage1.dry, rocket.stage1.thrust / 1e3, ...
        rocket.stage1.t_end - rocket.stage1.t_start);
    fprintf('Zefiro-40              : prop = %.0f kg | dry = %.0f kg | thrust = %.0f kN | burn = %.0f s\n', ...
        rocket.stage2.prop, rocket.stage2.dry, rocket.stage2.thrust / 1e3, ...
        rocket.stage2.t_end - rocket.stage2.t_start);
    fprintf('Zefiro-9               : prop = %.0f kg | dry = %.0f kg | thrust = %.0f kN | burn = %.0f s\n', ...
        rocket.stage3.prop, rocket.stage3.dry, rocket.stage3.thrust / 1e3, ...
        rocket.stage3.t_end - rocket.stage3.t_start);
    fprintf('AVUM+                  : prop = %.0f kg | dry = %.0f kg | thrust = %.2f kN | burn = %.0f s\n', ...
        rocket.stage4.prop, rocket.stage4.dry, rocket.stage4.thrust / 1e3, ...
        rocket.stage4.t_end - rocket.stage4.t_start);
    fprintf('=======================================\n');
end

function [thrust, mass, stage_info] = vega_c_thrust_mass(tk, rocket)
    thrust = 0;
    stage_info = 'coasting';

    m_fixed = rocket.payload_mass + rocket.interstage_mass;

    if tk <= rocket.stage1.t_end
        thrust = rocket.stage1.thrust;
        m1_prop = remaining_propellant(tk, rocket.stage1.t_start, rocket.stage1.t_end, rocket.stage1.prop);
        mass = rocket.stage1.dry + m1_prop + ...
               rocket.stage2.dry + rocket.stage2.prop + ...
               rocket.stage3.dry + rocket.stage3.prop + ...
               rocket.stage4.dry + rocket.stage4.prop + ...
               rocket.fairing_mass + m_fixed;
        stage_info = 'P120C';
        return;
    end

    fairing = rocket.fairing_mass;
    if tk >= 254.0
        fairing = 0;
    end

    if tk >= rocket.stage2.t_start && tk <= rocket.stage2.t_end
        thrust = rocket.stage2.thrust;
        m2_prop = remaining_propellant(tk, rocket.stage2.t_start, rocket.stage2.t_end, rocket.stage2.prop);
        mass = rocket.stage2.dry + m2_prop + ...
               rocket.stage3.dry + rocket.stage3.prop + ...
               rocket.stage4.dry + rocket.stage4.prop + ...
               fairing + m_fixed;
        stage_info = 'Zefiro-40';
        return;
    end

    if tk >= rocket.stage3.t_start && tk <= rocket.stage3.t_end
        thrust = rocket.stage3.thrust;
        m3_prop = remaining_propellant(tk, rocket.stage3.t_start, rocket.stage3.t_end, rocket.stage3.prop);
        mass = rocket.stage3.dry + m3_prop + ...
               rocket.stage4.dry + rocket.stage4.prop + ...
               fairing + m_fixed;
        stage_info = 'Zefiro-9';
        return;
    end

    if tk >= rocket.stage4.t_start && tk <= rocket.stage4.t_end
        thrust = rocket.stage4.thrust;
        m4_prop = remaining_propellant(tk, rocket.stage4.t_start, rocket.stage4.t_end, rocket.stage4.prop);
        mass = rocket.stage4.dry + m4_prop + m_fixed;
        stage_info = 'AVUM+';
        return;
    end

    if tk < rocket.stage2.t_start
        mass = rocket.stage2.dry + rocket.stage2.prop + ...
               rocket.stage3.dry + rocket.stage3.prop + ...
               rocket.stage4.dry + rocket.stage4.prop + ...
               fairing + m_fixed;
    elseif tk < rocket.stage3.t_start
        mass = rocket.stage3.dry + rocket.stage3.prop + ...
               rocket.stage4.dry + rocket.stage4.prop + ...
               fairing + m_fixed;
    elseif tk < rocket.stage4.t_start
        mass = rocket.stage4.dry + rocket.stage4.prop + m_fixed;
    else
        mass = rocket.stage4.dry + m_fixed;
    end
end

function prop_left = remaining_propellant(tk, t0, tf, prop0)
    if tk <= t0
        prop_left = prop0;
        return;
    end
    if tk >= tf
        prop_left = 0;
        return;
    end
    burn_fraction = (tk - t0) / max(tf - t0, eps);
    prop_left = prop0 * max(0, 1 - burn_fraction);
end

function I = cylinder_inertia(mass, height, diameter)
    radius = diameter / 2;
    Ixx = (1 / 12) * mass * (3 * radius^2 + height^2);
    Iyy = Ixx;
    Izz = 0.5 * mass * radius^2;
    I = diag([Ixx, Iyy, Izz]);
end

function txt = logical_to_text(tf)
    if tf
        txt = 'oui';
    else
        txt = 'non';
    end
end

function mode = figure_visibility(is_batch)
    if is_batch
        mode = 'off';
    else
        mode = 'on';
    end
end

function draw_cloud(ax, center, scale)
    [sx, sy, sz] = sphere(8);
    offsets = [0 0 0; 0.9 0.2 0.1; -0.8 0.1 -0.05; 0.25 0.55 0.12];
    radii = [1.0, 0.75, 0.70, 0.62] * scale;
    for i = 1:size(offsets, 1)
        surf(ax, center(1) + offsets(i, 1) * scale + sx * radii(i), ...
            center(2) + offsets(i, 2) * scale + sy * 0.55 * radii(i), ...
            center(3) + offsets(i, 3) * scale + sz * 0.35 * radii(i), ...
            'FaceColor', [1 1 1], 'EdgeColor', 'none', 'FaceAlpha', 0.42);
    end
end

function draw_tree(ax, xt, yt, zt)
    [ct_x, ct_y, ct_z] = cylinder(0.16, 7);
    trunk_h = 1.3;
    surf(ax, xt + ct_x, yt + ct_y, zt + ct_z * trunk_h, ...
        'FaceColor', [0.45 0.28 0.12], 'EdgeColor', 'none');

    [sx, sy, sz] = sphere(7);
    surf(ax, xt + sx * 0.85, yt + sy * 0.85, zt + trunk_h + 0.9 + sz * 1.0, ...
        'FaceColor', [0.16 0.48 0.16], 'EdgeColor', 'none', 'FaceAlpha', 0.95);
    surf(ax, xt + 0.35 + sx * 0.55, yt - 0.15 + sy * 0.55, ...
        zt + trunk_h + 1.2 + sz * 0.65, ...
        'FaceColor', [0.20 0.56 0.20], 'EdgeColor', 'none', 'FaceAlpha', 0.92);
end

function pts = rocket_wireframe_points(c, R, L, W)
    body_tail = c + R * [-L; 0; 0];
    body_front = c + R * [0.50 * L; 0; 0];
    nose_tip = c + R * [L; 0; 0];

    nose_up = c + R * [0.50 * L; 0; 0.55 * W];
    nose_down = c + R * [0.50 * L; 0; -0.55 * W];

    fin_left_tip = c + R * [-0.82 * L; -0.95 * W; -0.20 * W];
    fin_right_tip = c + R * [-0.82 * L; 0.95 * W; -0.20 * W];
    fin_root = c + R * [-0.45 * L; 0; -0.10 * W];

    pts.body = [body_tail, body_front];
    pts.nose1 = [nose_up, nose_tip];
    pts.nose2 = [nose_down, nose_tip];
    pts.fin1 = [fin_left_tip, fin_root];
    pts.fin2 = [fin_right_tip, fin_root];
end
