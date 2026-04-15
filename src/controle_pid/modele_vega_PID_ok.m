function [data, metrics] = modele_vega_PID_ok(mode_name, pid_override)
% Modele Vega 3D with PID attitude control.
% This version extends the Vega-inspired vehicle with PID-based attitude
% regulation and can render either a lightweight preview or a video.
%
% Usage examples:
%   modele_vega_PID_ok
%   modele_vega_PID_ok('preview')
%   modele_vega_PID_ok('video')
%   modele_vega_PID_ok('ultra_light')
%   modele_vega_PID_ok('debug')
%   modele_vega_PID_ok('no_render')
%
% Outputs:
%   - out/images/modele_vega_PID_ok.png
%   - out/images/modele_vega_PID_ok_pid.png
%   - out/logs/modele_vega_PID_ok.txt
%   - out/logs/modele_vega_PID_ok_pid.txt
%   - out/videos/modele_vega_PID_ok.mp4 (only when video export is enabled)
%   - out/blender/modele_vega_PID_ok_blender.csv
%   - out/blender/import_modele_vega_PID_ok_blender.py
%   - out/blender/import_modele_vega_PID_ok_blender_cinematic.py

    if nargin < 1 || isempty(mode_name)
        mode_name = 'preview';
    end
    if nargin < 2
        pid_override = struct();
    end

    clc; close all;
    if exist('OCTAVE_VERSION', 'builtin') ~= 0
        try
            graphics_toolkit('gnuplot');
        catch
        end
        set(0, 'defaultfigurevisible', 'off');
    end

    cfg = build_config(mode_name, pid_override);
    paths = build_paths();
    ensure_output_dirs(paths);
    display_vega_c_reference_table();

    data = simulate_open_loop_3d(cfg);
    metrics = compute_flight_metrics(data);
    pid_metrics = compute_pid_metrics(data, cfg);

    if cfg.render_enabled
        render_outputs(data, metrics, cfg, paths);
    end
    write_metrics_log(fullfile(paths.logs_dir, 'modele_vega_PID_ok.txt'), metrics, cfg);
    write_pid_diagnostics_log(fullfile(paths.logs_dir, 'modele_vega_PID_ok_pid.txt'), data, metrics, pid_metrics, cfg);
    export_pid_diagnostics_plot(fullfile(paths.images_dir, 'modele_vega_PID_ok_pid.png'), data, cfg);
    write_model_presentation(fullfile(paths.logs_dir, 'modele_vega_PID_ok_presentation.txt'));
    if exist('OCTAVE_VERSION', 'builtin') == 0
        export_blender_assets(paths, data, metrics, cfg);
    end

    fprintf('\nMode : %s\n', cfg.mode_name);
    if cfg.render_enabled
        fprintf('Image enregistree : %s\n', fullfile(paths.images_dir, 'modele_vega_PID_ok.png'));
    else
        fprintf('Image : rendu desactive pour ce mode.\n');
    end
    fprintf('Diagnostic PID : %s\n', fullfile(paths.images_dir, 'modele_vega_PID_ok_pid.png'));
    if cfg.export_video
        fprintf('Video enregistree : %s\n', fullfile(paths.videos_dir, 'modele_vega_PID_ok.mp4'));
    else
        fprintf('Video : export desactive pour ce mode.\n');
    end
    fprintf('Temps de vol : %.2f s | Portee horizontale : %.2f m | Altitude max : %.2f m\n', ...
        metrics.flight_time, metrics.range_xy, metrics.max_altitude);
    fprintf('Fin de simulation : %s\n', metrics.termination_reason);
    if pid_metrics.has_active_control
        fprintf('PID actif : RMS phi = %.3f deg | theta = %.3f deg | psi = %.3f deg\n', ...
            pid_metrics.rms_phi_deg, pid_metrics.rms_theta_deg, pid_metrics.rms_psi_deg);
        fprintf('PID actif : Mmax phi = %.3e | theta = %.3e | psi = %.3e N.m\n', ...
            pid_metrics.max_Mphi, pid_metrics.max_Mtheta, pid_metrics.max_Mpsi);
    else
        fprintf('PID actif : aucune phase de commande detectee.\n');
    end
    if metrics.satellite_released
        fprintf('Satellite deploye a t = %.2f s | altitude = %.2f m\n', ...
            metrics.satellite_release_time, metrics.satellite_release_altitude);
    end
    if isfinite(metrics.invalid_index)
        fprintf('Premier index invalide detecte : %d\n', metrics.invalid_index);
    end
end

function cfg = build_config(mode_name, pid_override)
    cfg.mode_name = lower(char(mode_name));
    cfg.is_batch = ~usejava('desktop');
    if exist('OCTAVE_VERSION', 'builtin') == 0
        cfg.is_octave_headless = false;
    else
        display_env = getenv('DISPLAY');
        cfg.is_octave_headless = isempty(display_env) || ~usejava('desktop');
    end

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
    cfg.pid_phi = struct('kp', 6.4e5, 'ki', 2.4e4, 'kd', 2.2e5);
    cfg.pid_theta = struct('kp', 2.8e6, 'ki', 1.2e5, 'kd', 8.0e4);
    cfg.pid_psi = struct('kp', 5.2e5, 'ki', 1.8e4, 'kd', 2.0e5);
    cfg.pid_integral_limit = deg2rad(12);
    cfg.pid_ref_filter_tau = 0.8;
    cfg.pid_derivative_filter_tau = 0.25;
    cfg.pid_enabled = true;

    switch cfg.mode_name
        case 'debug'
            cfg.export_video = false;
            cfg.show_decor = false;
            cfg.capture_divisor = 200;
            cfg.min_step = 12;
            cfg.sim_t_end = 80;
            cfg.figure_position = [100 80 960 620];
            cfg.terrain_resolution = 20;
            cfg.max_frames = 10;
            cfg.sim_dt = 0.5;
        case 'no_render'
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
        case 'preview'
            cfg.export_video = false;
            cfg.show_decor = false;
            cfg.capture_divisor = 120;
            cfg.sim_t_end = 520;
            cfg.sim_dt = 0.25;
        case 'video'
            cfg.export_video = true;
            cfg.show_decor = false;
            cfg.capture_divisor = 120;
            cfg.frame_rate = 15;
            cfg.video_quality = 50;
            cfg.sim_t_end = 520;
            cfg.sim_dt = 0.25;
        case 'ultra_light'
            cfg.export_video = false;
            cfg.show_decor = false;
            cfg.capture_divisor = 160;
            cfg.min_step = 6;
            cfg.sim_t_end = 320;
            cfg.figure_position = [100 80 960 620];
            cfg.terrain_resolution = 24;
            cfg.sim_dt = 0.25;
        case 'full'
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

    if cfg.is_octave_headless
        cfg.export_video = false;
        cfg.show_decor = false;
        cfg.render_enabled = false;
        cfg.mode_name = 'no_render';
    end

    if ~isempty(pid_override)
        if isfield(pid_override, 'pid_phi')
            cfg.pid_phi = merge_pid_axis(cfg.pid_phi, pid_override.pid_phi);
        end
        if isfield(pid_override, 'pid_theta')
            cfg.pid_theta = merge_pid_axis(cfg.pid_theta, pid_override.pid_theta);
        end
        if isfield(pid_override, 'pid_psi')
            cfg.pid_psi = merge_pid_axis(cfg.pid_psi, pid_override.pid_psi);
        end
        if isfield(pid_override, 'pid_integral_limit')
            cfg.pid_integral_limit = pid_override.pid_integral_limit;
        end
        if isfield(pid_override, 'pid_enabled')
            cfg.pid_enabled = pid_override.pid_enabled;
        end
    end
end

function paths = build_paths()
    base_dir = fileparts(mfilename('fullpath'));
    project_dir = fileparts(fileparts(base_dir));
    paths.videos_dir = fullfile(project_dir, 'out', 'videos');
    paths.images_dir = fullfile(project_dir, 'out', 'images');
    paths.logs_dir = fullfile(project_dir, 'out', 'logs');
    paths.blender_dir = fullfile(project_dir, 'out', 'blender');
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
    if ~exist(paths.blender_dir, 'dir')
        mkdir(paths.blender_dir);
    end
end

function export_blender_assets(paths, data, metrics, cfg)
    write_blender_csv(fullfile(paths.blender_dir, 'modele_vega_PID_ok_blender.csv'), data, cfg);
    write_blender_python_template(fullfile(paths.blender_dir, 'import_modele_vega_PID_ok_blender.py'), metrics, cfg);
    write_blender_cinematic_template(fullfile(paths.blender_dir, 'import_modele_vega_PID_ok_blender_cinematic.py'), metrics, cfg);
end

function write_blender_csv(csv_path, data, cfg)
    N = numel(data.t);
    speed = sqrt(data.vx(:).^2 + data.vy(:).^2 + data.vz(:).^2);
    launcher_visible = ones(N, 1);
    if data.launcher_impacted && isfinite(data.launcher_impact_idx)
        launcher_visible((data.launcher_impact_idx + 1):end) = 0;
    end

    satellite_visible = zeros(N, 1);
    if data.satellite_released && isfinite(data.satellite_release_idx)
        satellite_visible(data.satellite_release_idx:end) = 1;
    end

    launcher_impacted = zeros(N, 1);
    if data.launcher_impacted && isfinite(data.launcher_impact_idx)
        launcher_impacted(data.launcher_impact_idx:end) = 1;
    end

    satellite_released = zeros(N, 1);
    if data.satellite_released && isfinite(data.satellite_release_idx)
        satellite_released(data.satellite_release_idx:end) = 1;
    end

    T = table( ...
        (0:N-1)', data.t(:), ...
        data.x(:), data.y(:), data.z(:), ...
        data.vx(:), data.vy(:), data.vz(:), speed, ...
        rad2deg(data.phi(:)), rad2deg(data.theta(:)), rad2deg(data.psi(:)), ...
        data.mass(:), data.Tcmd(:), data.stage_name(:), ...
        data.sat_x(:), data.sat_y(:), data.sat_z(:), ...
        launcher_visible, satellite_visible, launcher_impacted, satellite_released, ...
        repmat(cfg.frame_rate, N, 1), ...
        'VariableNames', { ...
        'frame_idx', 'time_s', ...
        'launcher_x_m', 'launcher_y_m', 'launcher_z_m', ...
        'launcher_vx_mps', 'launcher_vy_mps', 'launcher_vz_mps', 'launcher_speed_mps', ...
        'launcher_phi_deg', 'launcher_theta_deg', 'launcher_psi_deg', ...
        'launcher_mass_kg', 'thrust_n', 'active_stage', ...
        'satellite_x_m', 'satellite_y_m', 'satellite_z_m', ...
        'launcher_visible', 'satellite_visible', 'launcher_impacted', 'satellite_released', ...
        'recommended_fps'});

    writetable(T, csv_path);
end

function write_blender_python_template(output_path, metrics, cfg)
    fid = fopen(output_path, 'w');
    if fid < 0
        warning('Impossible de creer le script Blender : %s', output_path);
        return;
    end

    fprintf(fid, '# Blender Python template generated by modele_vega_PID_ok.m\n');
    fprintf(fid, 'import bpy\n');
    fprintf(fid, 'import csv\n');
    fprintf(fid, 'from math import radians\n');
    fprintf(fid, 'from mathutils import Vector\n\n');
    fprintf(fid, '# Place this script in Blender''s text editor and adjust csv_path if needed.\n');
    fprintf(fid, 'csv_path = bpy.path.abspath("//modele_vega_PID_ok_blender.csv")\n');
    fprintf(fid, 'launcher_name = "VegaLauncher"\n');
    fprintf(fid, 'satellite_name = "DeployedSatellite"\n');
    fprintf(fid, 'camera_name = "VegaCamera"\n');
    fprintf(fid, 'flame_name = "VegaFlame"\n');
    fprintf(fid, 'trajectory_collection_name = "VegaTrajectories"\n');
    fprintf(fid, 'scene_scale = 0.001  # meters to Blender units (1 BU = 1 km)\n');
    fprintf(fid, 'camera_offset = Vector((-18.0, -12.0, 8.0))\n\n');
    fprintf(fid, 'scene = bpy.context.scene\n');
    fprintf(fid, 'scene.render.fps = %d\n\n', cfg.frame_rate);
    fprintf(fid, 'launcher = bpy.data.objects.get(launcher_name)\n');
    fprintf(fid, 'satellite = bpy.data.objects.get(satellite_name)\n');
    fprintf(fid, 'camera = bpy.data.objects.get(camera_name)\n');
    fprintf(fid, 'flame = bpy.data.objects.get(flame_name)\n');
    fprintf(fid, 'if launcher is None or satellite is None:\n');
    fprintf(fid, '    raise RuntimeError("Create Blender objects named VegaLauncher and DeployedSatellite before running this script.")\n\n');
    fprintf(fid, 'launcher.rotation_mode = "XYZ"\n');
    fprintf(fid, 'satellite.rotation_mode = "XYZ"\n');
    fprintf(fid, 'if camera is not None:\n');
    fprintf(fid, '    camera.rotation_mode = "XYZ"\n');
    fprintf(fid, 'if flame is not None:\n');
    fprintf(fid, '    flame.rotation_mode = "XYZ"\n\n');
    fprintf(fid, 'rows = []\n');
    fprintf(fid, 'launcher_points = []\n');
    fprintf(fid, 'satellite_points = []\n');
    fprintf(fid, 'with open(csv_path, newline="", encoding="utf-8") as f:\n');
    fprintf(fid, '    reader = csv.DictReader(f)\n');
    fprintf(fid, '    for row in reader:\n');
    fprintf(fid, '        rows.append(row)\n');
    fprintf(fid, '        launcher_points.append((\n');
    fprintf(fid, '            float(row["launcher_x_m"]) * scene_scale,\n');
    fprintf(fid, '            float(row["launcher_y_m"]) * scene_scale,\n');
    fprintf(fid, '            float(row["launcher_z_m"]) * scene_scale))\n');
    fprintf(fid, '        if row["satellite_visible"] == "1":\n');
    fprintf(fid, '            satellite_points.append((\n');
    fprintf(fid, '                float(row["satellite_x_m"]) * scene_scale,\n');
    fprintf(fid, '                float(row["satellite_y_m"]) * scene_scale,\n');
    fprintf(fid, '                float(row["satellite_z_m"]) * scene_scale))\n\n');
    fprintf(fid, 'def ensure_collection(name):\n');
    fprintf(fid, '    collection = bpy.data.collections.get(name)\n');
    fprintf(fid, '    if collection is None:\n');
    fprintf(fid, '        collection = bpy.data.collections.new(name)\n');
    fprintf(fid, '        bpy.context.scene.collection.children.link(collection)\n');
    fprintf(fid, '    return collection\n\n');
    fprintf(fid, 'def clear_object(name):\n');
    fprintf(fid, '    obj = bpy.data.objects.get(name)\n');
    fprintf(fid, '    if obj is not None:\n');
    fprintf(fid, '        bpy.data.objects.remove(obj, do_unlink=True)\n\n');
    fprintf(fid, 'def create_curve_object(name, points, collection):\n');
    fprintf(fid, '    if not points:\n');
    fprintf(fid, '        return None\n');
    fprintf(fid, '    clear_object(name)\n');
    fprintf(fid, '    curve = bpy.data.curves.new(name=name, type="CURVE")\n');
    fprintf(fid, '    curve.dimensions = "3D"\n');
    fprintf(fid, '    curve.resolution_u = 2\n');
    fprintf(fid, '    curve.bevel_depth = 0.02\n');
    fprintf(fid, '    spline = curve.splines.new("POLY")\n');
    fprintf(fid, '    spline.points.add(len(points) - 1)\n');
    fprintf(fid, '    for i, pt in enumerate(points):\n');
    fprintf(fid, '        spline.points[i].co = (pt[0], pt[1], pt[2], 1.0)\n');
    fprintf(fid, '    obj = bpy.data.objects.new(name, curve)\n');
    fprintf(fid, '    collection.objects.link(obj)\n');
    fprintf(fid, '    return obj\n\n');
    fprintf(fid, 'traj_collection = ensure_collection(trajectory_collection_name)\n');
    fprintf(fid, 'create_curve_object("LauncherTrajectory", launcher_points, traj_collection)\n');
    fprintf(fid, 'create_curve_object("SatelliteTrajectory", satellite_points, traj_collection)\n\n');
    fprintf(fid, 'for row in rows:\n');
    fprintf(fid, '    frame = int(row["frame_idx"]) + 1\n');
    fprintf(fid, '    scene.frame_set(frame)\n\n');
    fprintf(fid, '    launcher_loc = Vector((\n');
    fprintf(fid, '        float(row["launcher_x_m"]) * scene_scale,\n');
    fprintf(fid, '        float(row["launcher_y_m"]) * scene_scale,\n');
    fprintf(fid, '        float(row["launcher_z_m"]) * scene_scale))\n');
    fprintf(fid, '    launcher.location = launcher_loc\n');
    fprintf(fid, '    launcher.rotation_euler = (\n');
    fprintf(fid, '        radians(float(row["launcher_phi_deg"])),\n');
    fprintf(fid, '        radians(float(row["launcher_theta_deg"])),\n');
    fprintf(fid, '        radians(float(row["launcher_psi_deg"])))\n');
    fprintf(fid, '    launcher.hide_viewport = row["launcher_visible"] == "0"\n');
    fprintf(fid, '    launcher.hide_render = row["launcher_visible"] == "0"\n');
    fprintf(fid, '    launcher.keyframe_insert(data_path="location", frame=frame)\n');
    fprintf(fid, '    launcher.keyframe_insert(data_path="rotation_euler", frame=frame)\n');
    fprintf(fid, '    launcher.keyframe_insert(data_path="hide_viewport", frame=frame)\n');
    fprintf(fid, '    launcher.keyframe_insert(data_path="hide_render", frame=frame)\n\n');
    fprintf(fid, '    if row["satellite_visible"] == "1":\n');
    fprintf(fid, '        satellite.location = (\n');
    fprintf(fid, '            float(row["satellite_x_m"]) * scene_scale,\n');
    fprintf(fid, '            float(row["satellite_y_m"]) * scene_scale,\n');
    fprintf(fid, '            float(row["satellite_z_m"]) * scene_scale)\n');
    fprintf(fid, '    satellite.hide_viewport = row["satellite_visible"] == "0"\n');
    fprintf(fid, '    satellite.hide_render = row["satellite_visible"] == "0"\n');
    fprintf(fid, '    satellite.keyframe_insert(data_path="location", frame=frame)\n');
    fprintf(fid, '    satellite.keyframe_insert(data_path="hide_viewport", frame=frame)\n');
    fprintf(fid, '    satellite.keyframe_insert(data_path="hide_render", frame=frame)\n\n');
    fprintf(fid, '    if flame is not None:\n');
    fprintf(fid, '        flame.location = launcher_loc\n');
    fprintf(fid, '        flame.rotation_euler = launcher.rotation_euler\n');
    fprintf(fid, '        thrust_on = float(row["thrust_n"]) > 0.0 and row["launcher_visible"] == "1"\n');
    fprintf(fid, '        flame.hide_viewport = not thrust_on\n');
    fprintf(fid, '        flame.hide_render = not thrust_on\n');
    fprintf(fid, '        flame.scale = (1.0, 1.0, 0.6 + 0.8 * min(float(row["thrust_n"]) / 4500000.0, 1.0))\n');
    fprintf(fid, '        flame.keyframe_insert(data_path="location", frame=frame)\n');
    fprintf(fid, '        flame.keyframe_insert(data_path="rotation_euler", frame=frame)\n');
    fprintf(fid, '        flame.keyframe_insert(data_path="scale", frame=frame)\n');
    fprintf(fid, '        flame.keyframe_insert(data_path="hide_viewport", frame=frame)\n');
    fprintf(fid, '        flame.keyframe_insert(data_path="hide_render", frame=frame)\n\n');
    fprintf(fid, '    if camera is not None:\n');
    fprintf(fid, '        target = satellite.location.copy() if row["satellite_visible"] == "1" else launcher_loc\n');
    fprintf(fid, '        camera.location = target + camera_offset\n');
    fprintf(fid, '        direction = target - camera.location\n');
    fprintf(fid, '        camera.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()\n');
    fprintf(fid, '        camera.keyframe_insert(data_path="location", frame=frame)\n');
    fprintf(fid, '        camera.keyframe_insert(data_path="rotation_euler", frame=frame)\n\n');
    fprintf(fid, 'scene.frame_start = 1\n');
    fprintf(fid, 'scene.frame_end = %d\n', max(1, round(metrics.last_valid_index)));
    fprintf(fid, 'print("Import Vega/Blender termine avec trajectoires, camera et flamme optionnelle.")\n');
    fclose(fid);
end

function write_blender_cinematic_template(output_path, metrics, cfg)
    fid = fopen(output_path, 'w');
    if fid < 0
        warning('Impossible de creer le script Blender cinematique : %s', output_path);
        return;
    end

    fprintf(fid, '# Blender cinematic import template generated by modele_vega_PID_ok.m\n');
    fprintf(fid, 'import bpy\n');
    fprintf(fid, 'import csv\n');
    fprintf(fid, 'from math import radians\n');
    fprintf(fid, 'from mathutils import Vector\n\n');
    fprintf(fid, 'csv_path = bpy.path.abspath("//modele_vega_PID_ok_blender.csv")\n');
    fprintf(fid, 'launcher_name = "VegaLauncher"\n');
    fprintf(fid, 'satellite_name = "DeployedSatellite"\n');
    fprintf(fid, 'camera_name = "VegaCamera"\n');
    fprintf(fid, 'flame_name = "VegaFlame"\n');
    fprintf(fid, 'scene_scale = 0.001\n');
    fprintf(fid, 'earth_radius = 6.0\n');
    fprintf(fid, 'earth_location = Vector((0.0, 0.0, -earth_radius - 0.3))\n');
    fprintf(fid, 'camera_offset = Vector((-22.0, -14.0, 10.0))\n');
    fprintf(fid, 'trajectory_collection_name = "VegaTrajectories"\n\n');
    fprintf(fid, 'scene = bpy.context.scene\n');
    fprintf(fid, 'scene.render.fps = %d\n', cfg.frame_rate);
    fprintf(fid, 'scene.render.engine = "BLENDER_EEVEE_NEXT"\n\n');
    fprintf(fid, 'launcher = bpy.data.objects.get(launcher_name)\n');
    fprintf(fid, 'satellite = bpy.data.objects.get(satellite_name)\n');
    fprintf(fid, 'camera = bpy.data.objects.get(camera_name)\n');
    fprintf(fid, 'flame = bpy.data.objects.get(flame_name)\n');
    fprintf(fid, 'if launcher is None or satellite is None:\n');
    fprintf(fid, '    raise RuntimeError("Create Blender objects named VegaLauncher and DeployedSatellite before running this script.")\n\n');
    fprintf(fid, 'launcher.rotation_mode = "XYZ"\n');
    fprintf(fid, 'satellite.rotation_mode = "XYZ"\n');
    fprintf(fid, 'if camera is not None:\n');
    fprintf(fid, '    camera.rotation_mode = "XYZ"\n');
    fprintf(fid, 'if flame is not None:\n');
    fprintf(fid, '    flame.rotation_mode = "XYZ"\n\n');
    fprintf(fid, 'def ensure_collection(name):\n');
    fprintf(fid, '    collection = bpy.data.collections.get(name)\n');
    fprintf(fid, '    if collection is None:\n');
    fprintf(fid, '        collection = bpy.data.collections.new(name)\n');
    fprintf(fid, '        bpy.context.scene.collection.children.link(collection)\n');
    fprintf(fid, '    return collection\n\n');
    fprintf(fid, 'def clear_object(name):\n');
    fprintf(fid, '    obj = bpy.data.objects.get(name)\n');
    fprintf(fid, '    if obj is not None:\n');
    fprintf(fid, '        bpy.data.objects.remove(obj, do_unlink=True)\n\n');
    fprintf(fid, 'def get_or_create_material(name, base_color, emission_strength=0.0):\n');
    fprintf(fid, '    mat = bpy.data.materials.get(name)\n');
    fprintf(fid, '    if mat is None:\n');
    fprintf(fid, '        mat = bpy.data.materials.new(name=name)\n');
    fprintf(fid, '    mat.use_nodes = True\n');
    fprintf(fid, '    nodes = mat.node_tree.nodes\n');
    fprintf(fid, '    links = mat.node_tree.links\n');
    fprintf(fid, '    nodes.clear()\n');
    fprintf(fid, '    out = nodes.new("ShaderNodeOutputMaterial")\n');
    fprintf(fid, '    bsdf = nodes.new("ShaderNodeBsdfPrincipled")\n');
    fprintf(fid, '    bsdf.inputs["Base Color"].default_value = base_color\n');
    fprintf(fid, '    bsdf.inputs["Roughness"].default_value = 0.35\n');
    fprintf(fid, '    if emission_strength > 0.0:\n');
    fprintf(fid, '        bsdf.inputs["Emission Color"].default_value = base_color\n');
    fprintf(fid, '        bsdf.inputs["Emission Strength"].default_value = emission_strength\n');
    fprintf(fid, '    links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])\n');
    fprintf(fid, '    return mat\n\n');
    fprintf(fid, 'def ensure_world_background():\n');
    fprintf(fid, '    world = scene.world\n');
    fprintf(fid, '    if world is None:\n');
    fprintf(fid, '        world = bpy.data.worlds.new("World")\n');
    fprintf(fid, '        scene.world = world\n');
    fprintf(fid, '    world.use_nodes = True\n');
    fprintf(fid, '    nodes = world.node_tree.nodes\n');
    fprintf(fid, '    links = world.node_tree.links\n');
    fprintf(fid, '    nodes.clear()\n');
    fprintf(fid, '    out = nodes.new("ShaderNodeOutputWorld")\n');
    fprintf(fid, '    bg = nodes.new("ShaderNodeBackground")\n');
    fprintf(fid, '    bg.inputs["Color"].default_value = (0.01, 0.015, 0.04, 1.0)\n');
    fprintf(fid, '    bg.inputs["Strength"].default_value = 1.1\n');
    fprintf(fid, '    links.new(bg.outputs["Background"], out.inputs["Surface"])\n\n');
    fprintf(fid, 'def ensure_earth():\n');
    fprintf(fid, '    clear_object("StylizedEarth")\n');
    fprintf(fid, '    bpy.ops.mesh.primitive_uv_sphere_add(radius=earth_radius, location=earth_location)\n');
    fprintf(fid, '    earth = bpy.context.active_object\n');
    fprintf(fid, '    earth.name = "StylizedEarth"\n');
    fprintf(fid, '    earth_mat = get_or_create_material("EarthMaterial", (0.08, 0.22, 0.52, 1.0), emission_strength=0.15)\n');
    fprintf(fid, '    earth.data.materials.clear()\n');
    fprintf(fid, '    earth.data.materials.append(earth_mat)\n');
    fprintf(fid, '    return earth\n\n');
    fprintf(fid, 'def ensure_starfield():\n');
    fprintf(fid, '    collection = ensure_collection("VegaEnvironment")\n');
    fprintf(fid, '    for i in range(24):\n');
    fprintf(fid, '        name = f"Star_{i:02d}"\n');
    fprintf(fid, '        clear_object(name)\n');
    fprintf(fid, '        bpy.ops.mesh.primitive_uv_sphere_add(radius=0.03 + 0.02 * (i %% 3))\n');
    fprintf(fid, '        star = bpy.context.active_object\n');
    fprintf(fid, '        star.name = name\n');
    fprintf(fid, '        star.location = Vector(((-18 + i * 1.5), (8 - (i %% 6) * 3.0), 7 + (i %% 5) * 2.0))\n');
    fprintf(fid, '        star.data.materials.clear()\n');
    fprintf(fid, '        star.data.materials.append(get_or_create_material("StarMaterial", (1.0, 0.97, 0.92, 1.0), emission_strength=3.0))\n');
    fprintf(fid, '        collection.objects.link(star)\n');
    fprintf(fid, '        if star.name in bpy.context.scene.collection.objects:\n');
    fprintf(fid, '            bpy.context.scene.collection.objects.unlink(star)\n\n');
    fprintf(fid, 'def create_curve_object(name, points, bevel_depth, material_name, base_color, emission_strength):\n');
    fprintf(fid, '    if not points:\n');
    fprintf(fid, '        return None\n');
    fprintf(fid, '    clear_object(name)\n');
    fprintf(fid, '    curve = bpy.data.curves.new(name=name, type="CURVE")\n');
    fprintf(fid, '    curve.dimensions = "3D"\n');
    fprintf(fid, '    curve.resolution_u = 2\n');
    fprintf(fid, '    curve.bevel_depth = bevel_depth\n');
    fprintf(fid, '    spline = curve.splines.new("POLY")\n');
    fprintf(fid, '    spline.points.add(len(points) - 1)\n');
    fprintf(fid, '    for i, pt in enumerate(points):\n');
    fprintf(fid, '        spline.points[i].co = (pt[0], pt[1], pt[2], 1.0)\n');
    fprintf(fid, '    obj = bpy.data.objects.new(name, curve)\n');
    fprintf(fid, '    obj.data.materials.append(get_or_create_material(material_name, base_color, emission_strength))\n');
    fprintf(fid, '    ensure_collection(trajectory_collection_name).objects.link(obj)\n');
    fprintf(fid, '    return obj\n\n');
    fprintf(fid, 'ensure_world_background()\n');
    fprintf(fid, 'ensure_earth()\n');
    fprintf(fid, 'ensure_starfield()\n');
    fprintf(fid, 'launcher.data.materials.clear()\n');
    fprintf(fid, 'launcher.data.materials.append(get_or_create_material("LauncherMaterial", (0.82, 0.86, 0.92, 1.0), emission_strength=0.0))\n');
    fprintf(fid, 'satellite.data.materials.clear()\n');
    fprintf(fid, 'satellite.data.materials.append(get_or_create_material("SatelliteMaterial", (0.15, 0.85, 1.0, 1.0), emission_strength=2.5))\n');
    fprintf(fid, 'if flame is not None:\n');
    fprintf(fid, '    flame.data.materials.clear()\n');
    fprintf(fid, '    flame.data.materials.append(get_or_create_material("FlameMaterial", (1.0, 0.45, 0.08, 1.0), emission_strength=4.0))\n\n');
    fprintf(fid, 'rows = []\n');
    fprintf(fid, 'launcher_points = []\n');
    fprintf(fid, 'satellite_points = []\n');
    fprintf(fid, 'with open(csv_path, newline="", encoding="utf-8") as f:\n');
    fprintf(fid, '    reader = csv.DictReader(f)\n');
    fprintf(fid, '    for row in reader:\n');
    fprintf(fid, '        rows.append(row)\n');
    fprintf(fid, '        launcher_points.append((\n');
    fprintf(fid, '            float(row["launcher_x_m"]) * scene_scale,\n');
    fprintf(fid, '            float(row["launcher_y_m"]) * scene_scale,\n');
    fprintf(fid, '            float(row["launcher_z_m"]) * scene_scale))\n');
    fprintf(fid, '        if row["satellite_visible"] == "1":\n');
    fprintf(fid, '            satellite_points.append((\n');
    fprintf(fid, '                float(row["satellite_x_m"]) * scene_scale,\n');
    fprintf(fid, '                float(row["satellite_y_m"]) * scene_scale,\n');
    fprintf(fid, '                float(row["satellite_z_m"]) * scene_scale))\n\n');
    fprintf(fid, 'create_curve_object("LauncherTrajectory", launcher_points, 0.015, "LauncherTrailMaterial", (1.0, 0.35, 0.20, 1.0), 1.2)\n');
    fprintf(fid, 'create_curve_object("SatelliteTrajectory", satellite_points, 0.012, "SatelliteTrailMaterial", (0.20, 0.85, 1.0, 1.0), 2.0)\n\n');
    fprintf(fid, 'for row in rows:\n');
    fprintf(fid, '    frame = int(row["frame_idx"]) + 1\n');
    fprintf(fid, '    scene.frame_set(frame)\n');
    fprintf(fid, '    launcher_loc = Vector((\n');
    fprintf(fid, '        float(row["launcher_x_m"]) * scene_scale,\n');
    fprintf(fid, '        float(row["launcher_y_m"]) * scene_scale,\n');
    fprintf(fid, '        float(row["launcher_z_m"]) * scene_scale))\n');
    fprintf(fid, '    launcher.location = launcher_loc\n');
    fprintf(fid, '    launcher.rotation_euler = (\n');
    fprintf(fid, '        radians(float(row["launcher_phi_deg"])),\n');
    fprintf(fid, '        radians(float(row["launcher_theta_deg"])),\n');
    fprintf(fid, '        radians(float(row["launcher_psi_deg"])))\n');
    fprintf(fid, '    launcher.hide_viewport = row["launcher_visible"] == "0"\n');
    fprintf(fid, '    launcher.hide_render = row["launcher_visible"] == "0"\n');
    fprintf(fid, '    launcher.keyframe_insert(data_path="location", frame=frame)\n');
    fprintf(fid, '    launcher.keyframe_insert(data_path="rotation_euler", frame=frame)\n');
    fprintf(fid, '    launcher.keyframe_insert(data_path="hide_viewport", frame=frame)\n');
    fprintf(fid, '    launcher.keyframe_insert(data_path="hide_render", frame=frame)\n\n');
    fprintf(fid, '    if row["satellite_visible"] == "1":\n');
    fprintf(fid, '        satellite.location = (\n');
    fprintf(fid, '            float(row["satellite_x_m"]) * scene_scale,\n');
    fprintf(fid, '            float(row["satellite_y_m"]) * scene_scale,\n');
    fprintf(fid, '            float(row["satellite_z_m"]) * scene_scale)\n');
    fprintf(fid, '    satellite.hide_viewport = row["satellite_visible"] == "0"\n');
    fprintf(fid, '    satellite.hide_render = row["satellite_visible"] == "0"\n');
    fprintf(fid, '    satellite.keyframe_insert(data_path="location", frame=frame)\n');
    fprintf(fid, '    satellite.keyframe_insert(data_path="hide_viewport", frame=frame)\n');
    fprintf(fid, '    satellite.keyframe_insert(data_path="hide_render", frame=frame)\n\n');
    fprintf(fid, '    if flame is not None:\n');
    fprintf(fid, '        flame.location = launcher_loc\n');
    fprintf(fid, '        flame.rotation_euler = launcher.rotation_euler\n');
    fprintf(fid, '        thrust_on = float(row["thrust_n"]) > 0.0 and row["launcher_visible"] == "1"\n');
    fprintf(fid, '        flame.hide_viewport = not thrust_on\n');
    fprintf(fid, '        flame.hide_render = not thrust_on\n');
    fprintf(fid, '        flame.scale = (0.55, 0.55, 0.8 + 1.3 * min(float(row["thrust_n"]) / 4500000.0, 1.0))\n');
    fprintf(fid, '        flame.keyframe_insert(data_path="location", frame=frame)\n');
    fprintf(fid, '        flame.keyframe_insert(data_path="rotation_euler", frame=frame)\n');
    fprintf(fid, '        flame.keyframe_insert(data_path="scale", frame=frame)\n');
    fprintf(fid, '        flame.keyframe_insert(data_path="hide_viewport", frame=frame)\n');
    fprintf(fid, '        flame.keyframe_insert(data_path="hide_render", frame=frame)\n\n');
    fprintf(fid, '    if camera is not None:\n');
    fprintf(fid, '        target = Vector(satellite.location) if row["satellite_visible"] == "1" else launcher_loc\n');
    fprintf(fid, '        camera.location = target + camera_offset\n');
    fprintf(fid, '        direction = target - camera.location\n');
    fprintf(fid, '        camera.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()\n');
    fprintf(fid, '        camera.keyframe_insert(data_path="location", frame=frame)\n');
    fprintf(fid, '        camera.keyframe_insert(data_path="rotation_euler", frame=frame)\n\n');
    fprintf(fid, 'scene.frame_start = 1\n');
    fprintf(fid, 'scene.frame_end = %d\n', max(1, round(metrics.last_valid_index)));
    fprintf(fid, 'print("Import Vega/Blender cinematique termine.")\n');
    fclose(fid);
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
    stage_name = cell(1, N);
    sat_x = NaN(1, N); sat_y = NaN(1, N); sat_z = NaN(1, N);
    phi_ref_hist = zeros(1, N); theta_ref_hist = zeros(1, N); psi_ref_hist = zeros(1, N);
    pid_p_phi = zeros(1, N); pid_i_phi = zeros(1, N); pid_d_phi = zeros(1, N); pid_m_phi = zeros(1, N);
    pid_p_theta = zeros(1, N); pid_i_theta = zeros(1, N); pid_d_theta = zeros(1, N); pid_m_theta = zeros(1, N);
    pid_p_psi = zeros(1, N); pid_i_psi = zeros(1, N); pid_d_psi = zeros(1, N); pid_m_psi = zeros(1, N);
    control_active_hist = false(1, N);
    guidance_active_hist = false(1, N);

    theta_ref = interp1( ...
        [0, 15, 70, 140, 220, 300, 360], ...
        deg2rad([0, 5, 18, 35, 52, 63, 70]), ...
        t, 'pchip', 'extrap');
    psi_ref = interp1( ...
        [0, 60, 180, 260, 360], ...
        deg2rad([0, 1, 2, 3, 3]), ...
        t, 'pchip', 'extrap');

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
    int_phi = 0;
    int_theta = 0;
    int_psi = 0;
    phi_ref_state = 0;
    theta_ref_state = 0;
    psi_ref_state = 0;
    prev_phi_meas = phi(1);
    prev_theta_meas = theta(1);
    prev_psi_meas = psi(1);
    filt_dphi = 0;
    filt_dtheta = 0;
    filt_dpsi = 0;

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
            stage_name{k} = 'Lanceur au sol';

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

        guidance_active = (tk <= tburn) && (Tcmd(k - 1) > 0) && ~satellite_released;
        control_active = guidance_active && cfg.pid_enabled;
        if guidance_active
            phi_ref_raw = deg2rad(3) * sin(2 * pi * 0.35 * tk);
            theta_ref_raw = clamp(theta_ref(k), -tilt_max, tilt_max);
            psi_ref_raw = psi_ref(k);
        else
            phi_ref_raw = phi(k - 1);
            theta_ref_raw = theta(k - 1);
            psi_ref_raw = psi(k - 1);
        end

        phi_ref_state = lowpass_scalar(phi_ref_state, phi_ref_raw, cfg.pid_ref_filter_tau, dt);
        theta_ref_state = lowpass_scalar(theta_ref_state, theta_ref_raw, cfg.pid_ref_filter_tau, dt);
        psi_ref_state = lowpass_angle(psi_ref_state, psi_ref_raw, cfg.pid_ref_filter_tau, dt);
        phi_ref_hist(k) = phi_ref_state;
        theta_ref_hist(k) = theta_ref_state;
        psi_ref_hist(k) = psi_ref_state;
        guidance_active_hist(k) = guidance_active;
        control_active_hist(k) = control_active;

        [Tcmd(k), current_mass, stage_info] = vega_c_thrust_mass(tk, rocket);
        if satellite_released
            Tcmd(k) = 0;
            current_mass = launcher_mass_after_release;
            stage_info = 'Retour balistique';
        end
        m_hist(k) = current_mass;
        stage_name{k} = char(stage_info);
        if Tcmd(k) <= 0 && tk > rocket.t_stage3_end && z(k - 1) <= 0
            impact_idx = k - 1;
            break;
        end

        if control_active
            ephi = phi_ref_state - phi(k - 1);
            etheta = theta_ref_state - theta(k - 1);
            epsi = wrap_to_pi_local(psi_ref_state - psi(k - 1));

            dphi_meas = (phi(k - 1) - prev_phi_meas) / max(dt, eps);
            dtheta_meas = (theta(k - 1) - prev_theta_meas) / max(dt, eps);
            dpsi_meas = wrap_to_pi_local(psi(k - 1) - prev_psi_meas) / max(dt, eps);

            [Mx_ref, int_phi, filt_dphi, terms_phi] = pid_axis_moment( ...
                cfg.pid_phi, ephi, int_phi, dphi_meas, filt_dphi, dt, ...
                cfg.pid_integral_limit, cfg.max_control_moment, cfg.pid_derivative_filter_tau);
            [My_ref, int_theta, filt_dtheta, terms_theta] = pid_axis_moment( ...
                cfg.pid_theta, etheta, int_theta, dtheta_meas, filt_dtheta, dt, ...
                cfg.pid_integral_limit, cfg.max_control_moment, cfg.pid_derivative_filter_tau);
            [Mz_ref, int_psi, filt_dpsi, terms_psi] = pid_axis_moment( ...
                cfg.pid_psi, epsi, int_psi, dpsi_meas, filt_dpsi, dt, ...
                cfg.pid_integral_limit, cfg.max_control_moment, cfg.pid_derivative_filter_tau);
            pid_p_phi(k) = terms_phi.p; pid_i_phi(k) = terms_phi.i; pid_d_phi(k) = terms_phi.d; pid_m_phi(k) = Mx_ref;
            pid_p_theta(k) = terms_theta.p; pid_i_theta(k) = terms_theta.i; pid_d_theta(k) = terms_theta.d; pid_m_theta(k) = My_ref;
            pid_p_psi(k) = terms_psi.p; pid_i_psi(k) = terms_psi.i; pid_d_psi(k) = terms_psi.d; pid_m_psi(k) = Mz_ref;
        else
            Mx_ref = 0;
            My_ref = 0;
            Mz_ref = 0;
        end
        prev_phi_meas = phi(k - 1);
        prev_theta_meas = theta(k - 1);
        prev_psi_meas = psi(k - 1);
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
            stage_name{k} = 'Separation satellite';
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
    data.phi_ref = phi_ref_hist(1:impact_idx);
    data.theta_ref = theta_ref_hist(1:impact_idx);
    data.psi_ref = psi_ref_hist(1:impact_idx);
    data.pid_p_phi = pid_p_phi(1:impact_idx);
    data.pid_i_phi = pid_i_phi(1:impact_idx);
    data.pid_d_phi = pid_d_phi(1:impact_idx);
    data.pid_m_phi = pid_m_phi(1:impact_idx);
    data.pid_p_theta = pid_p_theta(1:impact_idx);
    data.pid_i_theta = pid_i_theta(1:impact_idx);
    data.pid_d_theta = pid_d_theta(1:impact_idx);
    data.pid_m_theta = pid_m_theta(1:impact_idx);
    data.pid_p_psi = pid_p_psi(1:impact_idx);
    data.pid_i_psi = pid_i_psi(1:impact_idx);
    data.pid_d_psi = pid_d_psi(1:impact_idx);
    data.pid_m_psi = pid_m_psi(1:impact_idx);
    data.guidance_active = guidance_active_hist(1:impact_idx);
    data.control_active = control_active_hist(1:impact_idx);
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

    saveas(fig, fullfile(paths.images_dir, 'modele_vega_PID_ok.png'));
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

    temp_video_path = fullfile(paths.videos_dir, 'modele_vega_PID_ok_tmp.mp4');
    final_video_path = fullfile(paths.videos_dir, 'modele_vega_PID_ok.mp4');
    if exist(temp_video_path, 'file')
        delete(temp_video_path);
    end

    video_backend = video_writer_compat_start(temp_video_path, cfg.frame_rate, cfg.video_quality);

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
        video_backend = video_writer_compat_write(video_backend, fig);
    end

    saveas(fig, fullfile(paths.images_dir, 'modele_vega_PID_ok.png'));
    video_writer_compat_close(video_backend);
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

    video_backend = [];
    temp_video_path = fullfile(paths.videos_dir, 'modele_vega_PID_ok_tmp.mp4');
    final_video_path = fullfile(paths.videos_dir, 'modele_vega_PID_ok.mp4');
    if cfg.export_video
        if exist(temp_video_path, 'file')
            delete(temp_video_path);
        end
        video_backend = video_writer_compat_start(temp_video_path, cfg.frame_rate, cfg.video_quality);
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
            video_backend = video_writer_compat_write(video_backend, fig);
        elseif ~cfg.is_batch && cfg.pause_dt > 0
            pause(cfg.pause_dt);
        end
    end

    saveas(fig, fullfile(paths.images_dir, 'modele_vega_PID_ok.png'));

    if cfg.export_video
        video_writer_compat_close(video_backend);
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

    fprintf(fid, ['Le modele Vega PID developpe dans ce travail est calibre sur un profil simplifie ' ...
        'inspire du lanceur europeen Vega-C. Cette adaptation permet de passer d''un vehicule generique ' ...
        'a une representation numerique plus credibile d''une fusee reelle, en integrant des ordres de grandeur ' ...
        'coherents pour la masse au decollage, la geometrie, les etages propulsifs et les niveaux de poussee, ainsi qu''un pilotage PID sur l''attitude.\n\n']);

    fprintf(fid, ['Le vehicule considere est un lanceur d''environ %.1f metres de hauteur, de %.1f metres de diametre, ' ...
        'avec une masse au decollage de l''ordre de %.0f kg. Dans le modele, la sequence propulsive reprend la logique ' ...
        'generale de Vega-C a travers quatre segments fonctionnels : un premier etage de type P120C, un deuxieme etage ' ...
        'de type Zefiro-40, un troisieme etage de type Zefiro-9, puis un etage superieur de type AVUM+. Chaque etage est ' ...
        'decrit par une masse propulsive, une masse seche estimee et une poussee moyenne appliquee sur une duree de combustion simplifiee.\n\n'], ...
        rocket.height, rocket.body_diameter, rocket.m0);

    fprintf(fid, ['Cette representation reste volontairement simplifiee : elle ne constitue pas un simulateur orbital complet, ' ...
        'mais un modele 3D d''ascension inspire du reel. Son objectif principal est pedagogique et scientifique : ' ...
        'montrer comment des donnees techniques issues d''un lanceur reel peuvent etre injectees dans un modele dynamique tridimensionnel, ' ...
        'afin d''observer l''influence conjointe de la poussee, de la masse variable, de l''attitude et de la gravite sur la trajectoire. ' ...
        'L''ajout du regulateur PID permet en outre d''etudier l''effet d''une commande en boucle fermee sur la stabilisation du vol.\n\n']);

    fprintf(fid, ['Dans la version actuelle du modele, une phase de separation simplifiee du satellite est introduite lorsque le lanceur atteint ' ...
        'l''altitude cible. A cet instant, la charge utile est consideree comme deployee dans l''espace sur une trajectoire propre, tandis que ' ...
        'le corps principal du lanceur poursuit une phase de retour balistique conduisant a sa retombee au sol. Cette extension permet de ' ...
        'representer plus clairement la logique d''une mission spatiale elementaire, en distinguant le mobile deploye et le lanceur apres separation.\n\n']);

    fprintf(fid, ['Dans cette perspective, le modele Vega PID enrichit sensiblement la qualite de l''analyse. Il permet de discuter les performances ' ...
        'de vol sur une base plus concrete, tout en conservant une structure de simulation suffisamment simple pour etre interpretable. ' ...
        'Il constitue ainsi une transition pertinente entre un modele academique de projectile propulse et une representation plus proche ' ...
        'des systemes de lancement reels, incluant une mise en espace simplifiee de la charge utile et un pilotage stabilise par PID.\n']);

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

function state = lowpass_scalar(state, target, tau, dt)
    if tau <= 0
        state = target;
        return;
    end
    alpha = dt / (tau + dt);
    state = state + alpha * (target - state);
end

function state = lowpass_angle(state, target, tau, dt)
    if tau <= 0
        state = wrap_to_pi_local(target);
        return;
    end
    alpha = dt / (tau + dt);
    state = wrap_to_pi_local(state + alpha * wrap_to_pi_local(target - state));
end

function [moment_cmd, int_state, filt_rate, terms] = pid_axis_moment(axis_cfg, err, int_state, meas_rate, filt_rate, dt, integral_limit, moment_limit, deriv_tau)
    filt_rate = lowpass_scalar(filt_rate, meas_rate, deriv_tau, dt);
    p_term = axis_cfg.kp * err;
    d_term = -axis_cfg.kd * filt_rate;

    int_candidate = clamp(int_state + err * dt, -integral_limit, integral_limit);
    unsat_with_candidate = p_term + axis_cfg.ki * int_candidate + d_term;
    moment_cmd = clamp(unsat_with_candidate, -moment_limit, moment_limit);

    % Conditional integration avoids winding up when the actuator is already saturated.
    if abs(unsat_with_candidate - moment_cmd) < 1e-9 || sign(err) ~= sign(unsat_with_candidate - moment_cmd)
        int_state = int_candidate;
    end

    moment_cmd = clamp(p_term + axis_cfg.ki * int_state + d_term, -moment_limit, moment_limit);
    terms.p = p_term;
    terms.i = axis_cfg.ki * int_state;
    terms.d = d_term;
    terms.meas_rate = filt_rate;
end

function export_pid_diagnostics_plot(output_path, data, cfg)
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [120 90 1180 820]);
    tl = tiledlayout(fig, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile(tl, 1);
    hold(ax1, 'on'); grid(ax1, 'on'); box(ax1, 'on');
    plot(ax1, data.t, rad2deg(data.phi), 'm', 'LineWidth', 1.5);
    plot(ax1, data.t, rad2deg(data.phi_ref), '--', 'Color', [0.35 0.0 0.55], 'LineWidth', 1.2);
    title(ax1, 'Roulis');
    ylabel(ax1, 'Angle (deg)');
    legend(ax1, 'phi', 'phi ref', 'Location', 'best');

    ax2 = nexttile(tl, 2);
    hold(ax2, 'on'); grid(ax2, 'on'); box(ax2, 'on');
    plot(ax2, data.t, data.pid_m_phi, 'Color', [0.65 0.0 0.65], 'LineWidth', 1.4);
    plot(ax2, data.t, data.pid_p_phi, ':', 'Color', [0.15 0.35 0.85], 'LineWidth', 1.0);
    plot(ax2, data.t, data.pid_i_phi, '--', 'Color', [0.10 0.55 0.10], 'LineWidth', 1.0);
    plot(ax2, data.t, data.pid_d_phi, '-.', 'Color', [0.85 0.30 0.10], 'LineWidth', 1.0);
    title(ax2, 'Moments PID roulis');
    ylabel(ax2, 'Moment (N.m)');
    legend(ax2, 'Mphi', 'P', 'I', 'D', 'Location', 'best');

    ax3 = nexttile(tl, 3);
    hold(ax3, 'on'); grid(ax3, 'on'); box(ax3, 'on');
    plot(ax3, data.t, rad2deg(data.theta), 'c', 'LineWidth', 1.5);
    plot(ax3, data.t, rad2deg(data.theta_ref), '--', 'Color', [0.0 0.45 0.55], 'LineWidth', 1.2);
    title(ax3, 'Tangage');
    ylabel(ax3, 'Angle (deg)');
    legend(ax3, 'theta', 'theta ref', 'Location', 'best');

    ax4 = nexttile(tl, 4);
    hold(ax4, 'on'); grid(ax4, 'on'); box(ax4, 'on');
    plot(ax4, data.t, data.pid_m_theta, 'Color', [0.00 0.45 0.60], 'LineWidth', 1.4);
    plot(ax4, data.t, data.pid_p_theta, ':', 'Color', [0.15 0.35 0.85], 'LineWidth', 1.0);
    plot(ax4, data.t, data.pid_i_theta, '--', 'Color', [0.10 0.55 0.10], 'LineWidth', 1.0);
    plot(ax4, data.t, data.pid_d_theta, '-.', 'Color', [0.85 0.30 0.10], 'LineWidth', 1.0);
    title(ax4, 'Moments PID tangage');
    ylabel(ax4, 'Moment (N.m)');
    legend(ax4, 'Mtheta', 'P', 'I', 'D', 'Location', 'best');

    ax5 = nexttile(tl, 5);
    hold(ax5, 'on'); grid(ax5, 'on'); box(ax5, 'on');
    plot(ax5, data.t, rad2deg(data.psi), 'k', 'LineWidth', 1.5);
    plot(ax5, data.t, rad2deg(data.psi_ref), '--', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.2);
    title(ax5, 'Lacet');
    xlabel(ax5, 'Temps (s)');
    ylabel(ax5, 'Angle (deg)');
    legend(ax5, 'psi', 'psi ref', 'Location', 'best');

    ax6 = nexttile(tl, 6);
    hold(ax6, 'on'); grid(ax6, 'on'); box(ax6, 'on');
    plot(ax6, data.t, data.pid_m_psi, 'Color', [0.15 0.15 0.15], 'LineWidth', 1.4);
    plot(ax6, data.t, data.pid_p_psi, ':', 'Color', [0.15 0.35 0.85], 'LineWidth', 1.0);
    plot(ax6, data.t, data.pid_i_psi, '--', 'Color', [0.10 0.55 0.10], 'LineWidth', 1.0);
    plot(ax6, data.t, data.pid_d_psi, '-.', 'Color', [0.85 0.30 0.10], 'LineWidth', 1.0);
    title(ax6, 'Moments PID lacet');
    xlabel(ax6, 'Temps (s)');
    ylabel(ax6, 'Moment (N.m)');
    legend(ax6, 'Mpsi', 'P', 'I', 'D', 'Location', 'best');

    sgtitle(tl, sprintf('Diagnostic PID - mode %s | filtre ref = %.2f s | filtre D = %.2f s', ...
        cfg.mode_name, cfg.pid_ref_filter_tau, cfg.pid_derivative_filter_tau));
    saveas(fig, output_path);
    close(fig);
end

function write_pid_diagnostics_log(log_path, data, metrics, pid_metrics, cfg)
    fid = fopen(log_path, 'w');
    if fid < 0
        warning('Impossible de creer le log PID : %s', log_path);
        return;
    end

    fprintf(fid, '===== DIAGNOSTIC PID MODELE VEGA PID OK =====\n');
    fprintf(fid, 'Mode                    : %s\n', cfg.mode_name);
    fprintf(fid, 'Temps de vol            : %.4f s\n', metrics.flight_time);
    fprintf(fid, 'Filtre reference tau    : %.4f s\n', cfg.pid_ref_filter_tau);
    fprintf(fid, 'Filtre derivee tau      : %.4f s\n', cfg.pid_derivative_filter_tau);
    fprintf(fid, 'Limite integrale        : %.4f deg\n', rad2deg(cfg.pid_integral_limit));
    fprintf(fid, 'Moment max commande     : %.4f N.m\n', cfg.max_control_moment);
    fprintf(fid, 'Commande active         : %s\n', logical_to_text(pid_metrics.has_active_control));
    if pid_metrics.has_active_control
        fprintf(fid, 'Echantillons actifs     : %d\n', pid_metrics.num_active_samples);
        fprintf(fid, 'RMS phi actif           : %.4f deg\n', pid_metrics.rms_phi_deg);
        fprintf(fid, 'RMS theta actif         : %.4f deg\n', pid_metrics.rms_theta_deg);
        fprintf(fid, 'RMS psi actif           : %.4f deg\n', pid_metrics.rms_psi_deg);
        fprintf(fid, 'Mmax phi actif          : %.4f N.m\n', pid_metrics.max_Mphi);
        fprintf(fid, 'Mmax theta actif        : %.4f N.m\n', pid_metrics.max_Mtheta);
        fprintf(fid, 'Mmax psi actif          : %.4f N.m\n', pid_metrics.max_Mpsi);
    end
    fprintf(fid, '\n');
    write_pid_axis_summary(fid, 'phi', cfg.pid_phi, pid_metrics.ephi_deg, pid_metrics.pid_m_phi, pid_metrics.pid_p_phi, pid_metrics.pid_i_phi, pid_metrics.pid_d_phi, cfg.max_control_moment);
    write_pid_axis_summary(fid, 'theta', cfg.pid_theta, pid_metrics.etheta_deg, pid_metrics.pid_m_theta, pid_metrics.pid_p_theta, pid_metrics.pid_i_theta, pid_metrics.pid_d_theta, cfg.max_control_moment);
    write_pid_axis_summary(fid, 'psi', cfg.pid_psi, pid_metrics.epsi_deg, pid_metrics.pid_m_psi, pid_metrics.pid_p_psi, pid_metrics.pid_i_psi, pid_metrics.pid_d_psi, cfg.max_control_moment);
    fprintf(fid, '================================================\n');
    fclose(fid);
end

function write_pid_axis_summary(fid, axis_name, axis_cfg, err_deg, moment_cmd, p_term, i_term, d_term, moment_limit)
    saturation_ratio = mean(abs(moment_cmd) >= 0.999 * moment_limit);
    fprintf(fid, '--- Axe %s ---\n', axis_name);
    fprintf(fid, 'Gains                   : kp = %.4e | ki = %.4e | kd = %.4e\n', axis_cfg.kp, axis_cfg.ki, axis_cfg.kd);
    fprintf(fid, 'Erreur RMS              : %.4f deg\n', sqrt(mean(err_deg .^ 2)));
    fprintf(fid, 'Erreur max abs          : %.4f deg\n', max(abs(err_deg)));
    fprintf(fid, 'Moment max abs          : %.4f N.m\n', max(abs(moment_cmd)));
    fprintf(fid, 'Part de saturation      : %.4f\n', saturation_ratio);
    fprintf(fid, 'P max abs               : %.4f N.m\n', max(abs(p_term)));
    fprintf(fid, 'I max abs               : %.4f N.m\n', max(abs(i_term)));
    fprintf(fid, 'D max abs               : %.4f N.m\n', max(abs(d_term)));
    fprintf(fid, '\n');
end

function pid_metrics = compute_pid_metrics(data, cfg)
    pid_metrics.has_active_control = false;
    pid_metrics.num_active_samples = 0;
    pid_metrics.rms_phi_deg = NaN;
    pid_metrics.rms_theta_deg = NaN;
    pid_metrics.rms_psi_deg = NaN;
    pid_metrics.max_Mphi = NaN;
    pid_metrics.max_Mtheta = NaN;
    pid_metrics.max_Mpsi = NaN;
    pid_metrics.ephi_deg = [];
    pid_metrics.etheta_deg = [];
    pid_metrics.epsi_deg = [];
    pid_metrics.pid_m_phi = [];
    pid_metrics.pid_m_theta = [];
    pid_metrics.pid_m_psi = [];
    pid_metrics.pid_p_phi = [];
    pid_metrics.pid_p_theta = [];
    pid_metrics.pid_p_psi = [];
    pid_metrics.pid_i_phi = [];
    pid_metrics.pid_i_theta = [];
    pid_metrics.pid_i_psi = [];
    pid_metrics.pid_d_phi = [];
    pid_metrics.pid_d_theta = [];
    pid_metrics.pid_d_psi = [];

    if ~isfield(data, 'control_active')
        return;
    end

    idx = find(data.control_active);
    if isempty(idx)
        return;
    end

    pid_metrics.has_active_control = true;
    pid_metrics.num_active_samples = numel(idx);
    pid_metrics.ephi_deg = rad2deg(data.phi_ref(idx) - data.phi(idx));
    pid_metrics.etheta_deg = rad2deg(data.theta_ref(idx) - data.theta(idx));
    pid_metrics.epsi_deg = rad2deg(arrayfun(@wrap_to_pi_local, data.psi_ref(idx) - data.psi(idx)));

    pid_metrics.pid_m_phi = data.pid_m_phi(idx);
    pid_metrics.pid_m_theta = data.pid_m_theta(idx);
    pid_metrics.pid_m_psi = data.pid_m_psi(idx);
    pid_metrics.pid_p_phi = data.pid_p_phi(idx);
    pid_metrics.pid_p_theta = data.pid_p_theta(idx);
    pid_metrics.pid_p_psi = data.pid_p_psi(idx);
    pid_metrics.pid_i_phi = data.pid_i_phi(idx);
    pid_metrics.pid_i_theta = data.pid_i_theta(idx);
    pid_metrics.pid_i_psi = data.pid_i_psi(idx);
    pid_metrics.pid_d_phi = data.pid_d_phi(idx);
    pid_metrics.pid_d_theta = data.pid_d_theta(idx);
    pid_metrics.pid_d_psi = data.pid_d_psi(idx);

    pid_metrics.rms_phi_deg = sqrt(mean(pid_metrics.ephi_deg .^ 2));
    pid_metrics.rms_theta_deg = sqrt(mean(pid_metrics.etheta_deg .^ 2));
    pid_metrics.rms_psi_deg = sqrt(mean(pid_metrics.epsi_deg .^ 2));
    pid_metrics.max_Mphi = max(abs(pid_metrics.pid_m_phi));
    pid_metrics.max_Mtheta = max(abs(pid_metrics.pid_m_theta));
    pid_metrics.max_Mpsi = max(abs(pid_metrics.pid_m_psi));
end

function octave_configure_headless_graphics()
    if exist('OCTAVE_VERSION', 'builtin') ~= 0
        try
            graphics_toolkit('gnuplot');
        catch
        end
        set(0, 'defaultfigurevisible', 'off');
    end
end

function tf = octave_headless_mode()
    if exist('OCTAVE_VERSION', 'builtin') == 0
        tf = false;
        return;
    end

    display_env = getenv('DISPLAY');
    tf = isempty(display_env) || ~usejava('desktop');
end

function backend = video_writer_compat_start(video_path, frame_rate, quality)
    backend = struct('type', 'none', 'writer', []);

    if exist('OCTAVE_VERSION', 'builtin') ~= 0
        return;
    end

    writer = VideoWriter(video_path, 'MPEG-4');
    writer.FrameRate = frame_rate;
    if isprop(writer, 'Quality')
        writer.Quality = quality;
    end
    open(writer);

    backend.type = 'matlab';
    backend.writer = writer;
end

function backend = video_writer_compat_write(backend, fig)
    if strcmp(backend.type, 'matlab')
        frame = getframe(fig);
        writeVideo(backend.writer, frame);
    end
end

function video_writer_compat_close(backend)
    if strcmp(backend.type, 'matlab')
        close(backend.writer);
    end
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
    metrics.final_phi_deg = rad2deg(data.phi(last_valid));
    metrics.final_theta_deg = rad2deg(data.theta(last_valid));
    metrics.final_psi_deg = rad2deg(data.psi(last_valid));
    metrics.max_abs_phi_deg = max(abs(rad2deg(data.phi(valid_idx))));
    metrics.max_abs_theta_deg = max(abs(rad2deg(data.theta(valid_idx))));
    metrics.max_abs_psi_deg = max(abs(rad2deg(data.psi(valid_idx))));
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

function axis_cfg = merge_pid_axis(axis_cfg, axis_override)
    fields = {'kp', 'ki', 'kd'};
    for i = 1:numel(fields)
        field_name = fields{i};
        if isfield(axis_override, field_name)
            axis_cfg.(field_name) = axis_override.(field_name);
        end
    end
end

function write_metrics_log(log_path, metrics, cfg)
    fid = fopen(log_path, 'w');
    if fid < 0
        warning('Impossible de creer le fichier log : %s', log_path);
        return;
    end

    fprintf(fid, '===== MODELE VEGA PID =====\n');
    fprintf(fid, 'Vehicule                : %s\n', metrics.vehicle_name);
    fprintf(fid, 'Mode                    : %s\n', cfg.mode_name);
    fprintf(fid, 'Commande PID            : %s\n', logical_to_text(cfg.pid_enabled));
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
