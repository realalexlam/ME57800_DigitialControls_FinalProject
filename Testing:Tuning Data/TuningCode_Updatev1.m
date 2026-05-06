%% tune_straight_drive_controller_from_marker_rpm_logs.m
% Tunes a practical straight-driving controller from logs containing:
%   - camera marker position: X_meters, Y_meters, Z_meters
%   - wheel encoder RPM: Left_RPM, Right_RPM
%
% Camera convention:
%   x = left/right
%   y = up/down
%   z = depth
%
% Controller structure tuned here:
%
%   steering_corr =
%       k_heading * e_psi
%     + k_lateral * e_y
%     + k_rpm_balance * e_rpm
%
% where:
%   e_y   = lateral offset (x)
%   e_psi = atan2(x, z)    % heading/bearing surrogate
%   e_rpm = normalized wheel-speed mismatch
%
% Notes:
% 1) This script tunes a straight-driving / heading-stabilizing controller.
% 2) It does NOT identify the full inner wheel PI gains, because the logs do
%    not include commanded wheel references or motor command inputs.
% 3) The output is useful for:
%       - outer-loop camera tuning
%       - optional RPM-balance correction term
%
% Suggested future use:
%   - If you keep your current ROS code unchanged, use:
%       k_heading, k_lateral, heading_deadband, lateral_deadband,
%       max_steering_correction
%   - If you later add RPM balance correction in ROS, also use:
%       k_rpm_balance, rpm_balance_deadband

clear; clc; close all;

%% User settings
logPattern = 'marker_rpm_test_*.csv';

% Camera timeout model used in simulation [s]
camera_hold_sec = 0.30;

% Smoothing window sizes (odd numbers preferred)
psiSmoothWindow = 11;
vSmoothWindow   = 11;
rpmSmoothWindow = 9;

% Cost weights
W.rms_x       = 5.0;   % lateral error
W.final_x     = 8.0;   % final lateral error
W.rms_psi     = 3.0;   % heading surrogate
W.rms_u       = 0.08;  % steering effort
W.rms_du      = 0.03;  % steering chatter
W.rms_rpmdiff = 0.75;  % wheel mismatch penalty

% Parameters to tune:
% p = [k_heading, k_lateral, k_rpm_balance, ...
%      heading_db, lateral_db, rpm_db, omega_max]
p0 = [0.90, 1.00, 0.35, 0.015, 0.002, 0.020, 1.80];

lb = [0.00, 0.00, -5.00, 0.001, 0.0005, 0.000, 0.10];
ub = [20.0, 20.0,  5.00, 0.300, 0.3000, 0.500, 5.00];

%% Load logs
files = dir(logPattern);
if isempty(files)
    error('No files matched pattern: %s', logPattern);
end

datasets = cell(numel(files),1);
for i = 1:numel(files)
    datasets{i} = loadMarkerRpmCsv(files(i).name, camera_hold_sec, ...
        psiSmoothWindow, vSmoothWindow, rpmSmoothWindow);
end

fprintf('Loaded %d CSV files.\n', numel(datasets));
for i = 1:numel(datasets)
    D = datasets{i};
    fprintf('  %s : N=%d, median Ts=%.4f s, duration=%.2f s, validCam=%d\n', ...
        D.name, numel(D.t), median(D.dt), D.t(end)-D.t(1), sum(D.valid_cam));
end

%% Tune controller
opts = optimset('Display','iter', ...
                'MaxFunEvals', 6000, ...
                'MaxIter', 2500, ...
                'TolX', 1e-5, ...
                'TolFun', 1e-6);

obj = @(p) totalCostBounded(p, datasets, W, lb, ub);
[pStar, Jstar] = fminsearch(obj, p0, opts);
pStar = clampVec(pStar, lb, ub);

k_heading   = pStar(1);
k_lateral   = pStar(2);
k_rpm_bal   = pStar(3);
heading_db  = pStar(4);
lateral_db  = pStar(5);
rpm_db      = pStar(6);
omega_max   = pStar(7);

fprintf('\n=== Tuned straight-driving controller parameters ===\n');
fprintf('k_heading                = %.6f\n', k_heading);
fprintf('k_lateral                = %.6f\n', k_lateral);
fprintf('k_rpm_balance            = %.6f\n', k_rpm_bal);
fprintf('heading_deadband [rad]   = %.6f\n', heading_db);
fprintf('lateral_deadband [m]     = %.6f\n', lateral_db);
fprintf('rpm_balance_deadband     = %.6f\n', rpm_db);
fprintf('omega_max [rad/s]        = %.6f\n', omega_max);
fprintf('Objective J              = %.6f\n', Jstar);

%% Simulate baseline and tuned controller
results = cell(numel(datasets),1);
summary = cell(numel(datasets),1);

for i = 1:numel(datasets)
    D = datasets{i};

    baseParams  = [0, 0, 0, heading_db, lateral_db, rpm_db, omega_max];
    tunedParams = pStar;

    B = simulateStraightController(D, baseParams);
    T = simulateStraightController(D, tunedParams);

    results{i}.D = D;
    results{i}.base = B;
    results{i}.tuned = T;

    summary{i,1} = D.name;
    summary{i,2} = rms(abs(D.x_raw_valid));
    summary{i,3} = rms(abs(B.x_sim));
    summary{i,4} = rms(abs(T.x_sim));
    summary{i,5} = abs(B.x_sim(end));
    summary{i,6} = abs(T.x_sim(end));
    summary{i,7} = rms(abs(D.psi_raw_valid));
    summary{i,8} = rms(abs(T.psi_sim));
    summary{i,9} = rms(T.u);
    summary{i,10} = 100 * (rms(abs(D.x_raw_valid)) - rms(abs(T.x_sim))) / max(rms(abs(D.x_raw_valid)), eps);
    summary{i,11} = 100 * (rms(abs(D.psi_raw_valid)) - rms(abs(T.psi_sim))) / max(rms(abs(D.psi_raw_valid)), eps);
end

summaryTable = cell2table(summary, 'VariableNames', ...
    {'File','RMS_MeasuredX','RMS_BaselineX','RMS_TunedX', ...
     'FinalAbsBaselineX','FinalAbsTunedX', ...
     'RMS_MeasuredPsi','RMS_TunedPsi','RMS_SteerCmd', ...
     'PctImproveX','PctImprovePsi'});

disp(' ');
disp('=== Summary ===');
disp(summaryTable);

%% Per-file plots
for i = 1:numel(results)
    R = results{i};
    D = R.D;
    B = R.base;
    T = R.tuned;

    % Forward-distance proxy
    s = zeros(size(D.t));
    for k = 1:numel(D.t)-1
        s(k+1) = s(k) + D.v(k) * D.dt(k);
    end

    figure('Name', D.name, 'Color', 'w');
    tiledlayout(3,2,'Padding','compact','TileSpacing','compact');

    % 1) Lateral offset vs time
    nexttile;
    plot(D.t, D.x_fill, 'k--', 'LineWidth', 1.2); hold on;
    plot(D.t, B.x_sim, '--', 'LineWidth', 1.1);
    plot(D.t, T.x_sim, '-', 'LineWidth', 1.5);
    yline(0,'k:');
    grid on;
    xlabel('Time [s]'); ylabel('X lateral [m]');
    title('Lateral error vs time');
    legend('Measured','Baseline sim','Tuned sim','Location','best');

    % 2) Heading surrogate vs time
    nexttile;
    plot(D.t, D.psi_fill, 'k--', 'LineWidth', 1.2); hold on;
    plot(D.t, B.psi_sim, '--', 'LineWidth', 1.1);
    plot(D.t, T.psi_sim, '-', 'LineWidth', 1.5);
    yline(0,'k:');
    grid on;
    xlabel('Time [s]'); ylabel('\psi [rad]');
    title('\psi \approx atan2(x,z)');
    legend('Measured','Baseline sim','Tuned sim','Location','best');

    % 3) Path proxy: lateral vs forward distance
    nexttile;
    plot(s, D.x_fill, 'k--', 'LineWidth', 1.2); hold on;
    plot(s, T.x_sim, '-', 'LineWidth', 1.5);
    yline(0,'k:');
    grid on;
    xlabel('Forward distance proxy [m]');
    ylabel('Lateral offset [m]');
    title('Before/after path proxy');
    legend('Measured','Tuned sim','Location','best');

    % 4) X-Z camera-frame proxy
    nexttile;
    plot(D.z_fill, D.x_fill, 'k--', 'LineWidth', 1.2); hold on;
    plot(D.z_fill, T.x_sim, '-', 'LineWidth', 1.5);
    grid on;
    xlabel('Z depth [m]');
    ylabel('X lateral [m]');
    title('Camera-frame X-Z path proxy');
    legend('Measured','Tuned sim','Location','best');

    % 5) Steering and wheel mismatch
    nexttile;
    plot(D.t, T.u, 'LineWidth', 1.4); hold on;
    yline(0,'k:');
    yline(omega_max,'r--');
    yline(-omega_max,'r--');
    grid on;
    xlabel('Time [s]');
    ylabel('\omega_{corr} [rad/s]');
    title('Steering correction');
    legend('\omega_{corr}','limits','Location','best');

    % 6) Normalized RPM mismatch
    nexttile;
    plot(D.t, D.rpm_diff_norm, 'LineWidth', 1.2); hold on;
    yline(0,'k:');
    yline(rpm_db,'r--');
    yline(-rpm_db,'r--');
    grid on;
    xlabel('Time [s]');
    ylabel('RPM mismatch');
    title('Normalized wheel-speed mismatch');
    legend('e_{rpm}','deadband','Location','best');

    sgtitle(sprintf('%s', D.name), 'Interpreter', 'none');
end

%% Aggregate improvement bars
names = strings(numel(results),1);
improveX = zeros(numel(results),1);
improvePsi = zeros(numel(results),1);

for i = 1:numel(results)
    names(i) = erase(results{i}.D.name, '.csv');
    improveX(i) = summary{i,10};
    improvePsi(i) = summary{i,11};
end

figure('Name','Improvement summary','Color','w');
tiledlayout(2,1,'Padding','compact','TileSpacing','compact');

nexttile;
bar(categorical(names), improveX);
grid on;
ylabel('Improvement [%]');
title('Percent improvement in lateral RMS');

nexttile;
bar(categorical(names), improvePsi);
grid on;
ylabel('Improvement [%]');
title('Percent improvement in heading surrogate RMS');

%% Overlay summary plots
figure('Name','All logs overlay','Color','w');
tiledlayout(2,1,'Padding','compact','TileSpacing','compact');

nexttile;
hold on; grid on;
for i = 1:numel(results)
    D = results{i}.D;
    T = results{i}.tuned;
    plot(D.t - D.t(1), D.x_fill, '--');
    plot(D.t - D.t(1), T.x_sim, 'LineWidth', 1.2);
end
xlabel('Time from start [s]');
ylabel('X lateral [m]');
title('Measured vs tuned lateral error (all logs)');

nexttile;
hold on; grid on;
for i = 1:numel(results)
    D = results{i}.D;
    T = results{i}.tuned;
    plot(D.t - D.t(1), abs(D.psi_fill), '--');
    plot(D.t - D.t(1), abs(T.psi_sim), 'LineWidth', 1.2);
end
xlabel('Time from start [s]');
ylabel('|\psi| [rad]');
title('Measured vs tuned heading surrogate magnitude (all logs)');

%% Suggested controller constants
fprintf('\n=== Suggested straight-driving controller constants ===\n');
fprintf('k_heading              = %.6f\n', k_heading);
fprintf('k_lateral              = %.6f\n', k_lateral);
fprintf('k_rpm_balance          = %.6f\n', k_rpm_bal);
fprintf('heading_deadband       = %.6f\n', heading_db);
fprintf('lateral_deadband       = %.6f\n', lateral_db);
fprintf('rpm_balance_deadband   = %.6f\n', rpm_db);
fprintf('max_steering_correction= %.6f\n', omega_max);

fprintf('\nROS compatibility note:\n');
fprintf(['- Your current ROS code can directly use:\n' ...
         '    k_heading, k_lateral, heading_deadband, lateral_deadband,\n' ...
         '    max_steering_correction\n' ...
         '- To use k_rpm_balance and rpm_balance_deadband, you would need to\n' ...
         '  add an RPM-balance term in the steering correction law.\n']);

%% ========================= Local functions =========================
function D = loadMarkerRpmCsv(filename, camera_hold_sec, psiSmoothWindow, vSmoothWindow, rpmSmoothWindow)
    T = readtable(filename, 'TextType', 'string');

    requiredCols = ["Timestamp","Marker_ID","X_meters","Y_meters","Z_meters","Left_RPM","Right_RPM"];
    for c = requiredCols
        if ~ismember(c, string(T.Properties.VariableNames))
            error('File %s is missing required column: %s', filename, c);
        end
    end

    tsec = parseTimestampToSeconds(T.Timestamp);
    tsec = tsec(:);

    finiteMask = isfinite(tsec);
    T = T(finiteMask,:);
    tsec = tsec(finiteMask);

    keep = [true; diff(tsec) > 0];
    T = T(keep,:);
    tsec = tsec(keep);

    if numel(tsec) < 2
        error('File %s has too few valid timestamp samples.', filename);
    end

    markerId = string(T.Marker_ID);
    x_raw = T.X_meters(:);
    z_raw = T.Z_meters(:);
    left_rpm = double(T.Left_RPM(:));
    right_rpm = double(T.Right_RPM(:));

    valid_cam = (markerId ~= "Lost") & isfinite(x_raw) & isfinite(z_raw) & (z_raw > 0);

    % Keep raw valid-only vectors for metrics
    x_raw_valid = x_raw(valid_cam);
    z_raw_valid = z_raw(valid_cam);
    if isempty(x_raw_valid)
        error('File %s has no valid camera measurements.', filename);
    end
    psi_raw_valid = atan2(x_raw_valid, max(z_raw_valid, 1e-6));

    % Filled camera signals for model building / plotting
    x_cam = x_raw;
    z_cam = z_raw;
    x_cam(~valid_cam) = NaN;
    z_cam(~valid_cam) = NaN;

    x_fill = fillmissing(x_cam, 'previous');
    z_fill = fillmissing(z_cam, 'previous');

    % If file starts with invalid data
    if isnan(x_fill(1))
        firstValid = find(valid_cam, 1, 'first');
        x_fill(1:firstValid-1) = x_raw(firstValid);
        z_fill(1:firstValid-1) = z_raw(firstValid);
    end

    x_fill = fillmissing(x_fill, 'nearest');
    z_fill = fillmissing(z_fill, 'nearest');

    z_safe = max(z_fill, 1e-6);
    psi_fill = atan2(x_fill, z_safe);

    % Smooth psi for derivative estimate
    psi_smooth = smoothdata(psi_fill, 'movmedian', psiSmoothWindow);

    dt = diff(tsec);
    dt = [dt; dt(end)];

    % Forward-speed proxy from depth profile
    z_smooth = smoothdata(z_fill, 'movmedian', vSmoothWindow);
    dz = gradient(z_smooth, tsec);
    v = max(0, -dz);   % moving toward marker => z decreases

    % Open-loop heading-rate disturbance estimate
    psi_dot_open = gradient(psi_smooth, tsec);

    % RPM mismatch
    rpm_avg = max((abs(left_rpm) + abs(right_rpm)) / 2, 1.0);
    rpm_diff_norm = smoothdata((right_rpm - left_rpm) ./ rpm_avg, 'movmean', rpmSmoothWindow);

    D.name = filename;
    D.t = tsec;
    D.dt = dt;

    D.valid_cam = valid_cam;
    D.camera_hold_sec = camera_hold_sec;

    D.x_raw_valid = x_raw_valid;
    D.psi_raw_valid = psi_raw_valid;

    D.x_fill = x_fill;
    D.z_fill = z_fill;
    D.psi_fill = psi_fill;
    D.psi_dot_open = psi_dot_open;
    D.v = v;

    D.left_rpm = left_rpm;
    D.right_rpm = right_rpm;
    D.rpm_diff_norm = rpm_diff_norm;
end

function tsec = parseTimestampToSeconds(ts)
    if isdatetime(ts)
        tsec = seconds(ts - ts(1));
        return;
    end

    if isduration(ts)
        tsec = seconds(ts - ts(1));
        return;
    end

    if isnumeric(ts)
        tsec = ts - ts(1);
        return;
    end

    if iscell(ts)
        ts = string(ts);
    elseif ischar(ts)
        ts = string(cellstr(ts));
    elseif ~isstring(ts)
        try
            ts = string(ts);
        catch
            error('Unsupported Timestamp type: %s', class(ts));
        end
    end

    ts = strtrim(ts(:));
    bad = ismissing(ts) | (strlength(ts) == 0);
    ts = ts(~bad);

    if isempty(ts)
        error('Timestamp column is empty.');
    end

    % Try datetime formats first
    fmts_dt = [ ...
        "HH:mm:ss.SSS", ...
        "HH:mm:ss.SS", ...
        "HH:mm:ss.S", ...
        "HH:mm:ss", ...
        "yyyy-MM-dd HH:mm:ss.SSS", ...
        "yyyy-MM-dd HH:mm:ss", ...
        "MM/dd/yyyy HH:mm:ss.SSS", ...
        "MM/dd/yyyy HH:mm:ss" ...
    ];

    for k = 1:numel(fmts_dt)
        try
            dtm = datetime(ts, 'InputFormat', fmts_dt(k));
            tsec = seconds(dtm - dtm(1));
            return;
        catch
        end
    end

    % Try duration formats
    fmts_dur = [ ...
        "hh:mm:ss.SSS", ...
        "hh:mm:ss.SS", ...
        "hh:mm:ss.S", ...
        "hh:mm:ss", ...
        "mm:ss.SSS", ...
        "mm:ss.SS", ...
        "mm:ss.S", ...
        "mm:ss" ...
    ];

    for k = 1:numel(fmts_dur)
        try
            dur = duration(ts, 'InputFormat', fmts_dur(k));
            tsec = seconds(dur - dur(1));
            return;
        catch
        end
    end

    vals = str2double(ts);
    if all(isfinite(vals))
        tsec = vals - vals(1);
        return;
    end

    error('Could not parse Timestamp column. First value was: %s', ts(1));
end

function J = totalCostBounded(p, datasets, W, lb, ub)
    p = clampVec(p, lb, ub);
    Jsum = 0;

    for i = 1:numel(datasets)
        R = simulateStraightController(datasets{i}, p);

        xNorm = max(0.05, abs(datasets{i}.x_fill(1)));
        psiNorm = max(0.02, abs(datasets{i}.psi_fill(1)));

        J_i = W.rms_x       * rms(R.x_sim / xNorm) + ...
              W.final_x     * abs(R.x_sim(end) / xNorm) + ...
              W.rms_psi     * rms(R.psi_sim / psiNorm) + ...
              W.rms_u       * rms(R.u) + ...
              W.rms_du      * rms(R.du) + ...
              W.rms_rpmdiff * rms(R.rpm_diff_used);

        Jsum = Jsum + J_i;
    end

    J = Jsum / numel(datasets);
end

function R = simulateStraightController(D, p)
    % p = [k_heading, k_lateral, k_rpm_balance, heading_db, lateral_db, rpm_db, omega_max]
    k_heading = p(1);
    k_lateral = p(2);
    k_rpm     = p(3);
    hdb       = p(4);
    ldb       = p(5);
    rpmdb     = p(6);
    omegaMax  = p(7);

    N = numel(D.t);
    x = zeros(N,1);
    psi = zeros(N,1);
    u = zeros(N,1);
    e_rpm_used = zeros(N,1);

    x(1) = D.x_fill(1);
    psi(1) = D.psi_fill(1);

    lastValidCamTime = -inf;

    for k = 1:N-1
        Ts = D.dt(k);

        if D.valid_cam(k)
            lastValidCamTime = D.t(k);
        end

        camFresh = (D.t(k) - lastValidCamTime) <= D.camera_hold_sec;

        if camFresh
            e_y = applyDeadband(x(k), ldb);
            e_psi = applyDeadband(psi(k), hdb);
        else
            e_y = 0;
            e_psi = 0;
        end

        e_rpm = applyDeadband(D.rpm_diff_norm(k), rpmdb);
        e_rpm_used(k) = e_rpm;

        omega_corr = k_heading * e_psi + k_lateral * e_y + k_rpm * e_rpm;
        omega_corr = clamp(omega_corr, -omegaMax, omegaMax);
        u(k) = omega_corr;

        % Disturbance-following surrogate model:
        %   measured psi_dot_open is treated as the open-loop drift tendency
        %   controller subtracts from it
        psi(k+1) = psi(k) + Ts * (D.psi_dot_open(k) - omega_corr);

        % Kinematic lateral update
        x(k+1) = x(k) + Ts * D.v(k) * sin(psi(k));
    end

    u(end) = u(max(end-1,1));
    e_rpm_used(end) = e_rpm_used(max(end-1,1));

    du = [0; diff(u) ./ max(D.dt(1:end-1), eps)];

    R.x_sim = x;
    R.psi_sim = psi;
    R.u = u;
    R.du = du;
    R.rpm_diff_used = e_rpm_used;
end

function y = applyDeadband(x, db)
    if abs(x) < db
        y = 0;
    else
        y = x;
    end
end

function x = clamp(x, lo, hi)
    x = max(lo, min(hi, x));
end

function v = clampVec(v, lo, hi)
    v = max(lo, min(hi, v));
end