%% tune_outer_camera_controller_from_logs.m
% Tunes the OUTER camera/tag straightening controller from ArUco CSV logs.
%
% IMPORTANT:
%   These CSVs only contain camera position data (X,Y,Z). They do NOT contain
%   wheel encoder speeds or motor commands. Because of that, this script tunes
%   ONLY the outer-loop parameters used in the ROS2 controller:
%
%       steering_corr = k_heading * e_psi + k_lateral * e_y
%
%   where:
%       e_y   = X_meters   (left/right lateral offset)
%       e_psi = atan2(X_meters, Z_meters)  (bearing-angle surrogate)
%
%   The inner wheel PI gains (kp_left, ki_left, kp_right, ki_right) CANNOT
%   be identified from these files alone. Tune those later with encoder data
%   or in the digital twin.
%
% Assumptions:
%   x = left/right, y = up/down, z = depth
%   positive steering correction reduces positive heading/bearing error
%   forward speed is estimated from the measured depth profile z(t)
%
% Outputs:
%   - Tuned outer-loop gains / deadbands
%   - Plots comparing measured drift vs simulated corrected response
%   - Additional path/heading validation plots
%   - Suggested ROS2 parameter values

clear; clc; close all;

%% User settings
logPattern = 'aruco_tracking_data_*.csv';

% Cost weights for tuning
W.rms_x      = 4.0;   % penalize lateral offset strongly
W.final_x    = 8.0;   % penalize final lateral offset
W.rms_psi    = 2.0;   % penalize heading/bearing error
W.rms_u      = 0.10;  % penalize large steering commands
W.rms_du     = 0.02;  % penalize chattering / aggressive corrections

% Initial guesses for [k_heading, k_lateral, heading_db, lateral_db, omega_max]
p0 = [1.5, 1.0, 0.03, 0.01, 1.50];

% Bounds
lb = [0.01, 0.01, 0.001, 0.001, 0.10];
ub = [20.0, 20.0, 0.300, 0.300, 5.00];

%% Load all CSV logs
files = dir(logPattern);
if isempty(files)
    error('No files matched pattern: %s', logPattern);
end

datasets = cell(numel(files),1);
for i = 1:numel(files)
    datasets{i} = loadArucoCsv(files(i).name);
end

fprintf('Loaded %d CSV files.\n', numel(datasets));
for i = 1:numel(datasets)
    D = datasets{i};
    fprintf('  %s : N=%d, median Ts=%.4f s, duration=%.2f s\n', ...
        D.name, numel(D.t), median(D.dt), D.t(end)-D.t(1));
end

%% Tune outer-loop parameters
q0 = log(p0);
opts = optimset('Display','iter', 'MaxFunEvals', 5000, 'MaxIter', 2000, ...
                'TolX',1e-5, 'TolFun',1e-6);

obj = @(q) totalCost(exp(q), datasets, W, lb, ub);
[qStar, Jstar] = fminsearch(obj, q0, opts);
pStar = exp(qStar);
pStar = clampVec(pStar, lb, ub);

k_heading   = pStar(1);
k_lateral   = pStar(2);
heading_db  = pStar(3);
lateral_db  = pStar(4);
omega_max   = pStar(5);

fprintf('\n=== Tuned OUTER-LOOP parameters ===\n');
fprintf('k_heading              = %.6f\n', k_heading);
fprintf('k_lateral              = %.6f\n', k_lateral);
fprintf('heading_deadband [rad] = %.6f\n', heading_db);
fprintf('lateral_deadband [m]   = %.6f\n', lateral_db);
fprintf('omega_max [rad/s]      = %.6f\n', omega_max);
fprintf('Objective J            = %.6f\n', Jstar);

%% Simulate with tuned controller and baseline
results = cell(numel(datasets),1);
summary = cell(numel(datasets),1);
for i = 1:numel(datasets)
    D = datasets{i};
    tuned = simulateOuterLoop(D, pStar);
    base  = simulateOuterLoop(D, [0, 0, heading_db, lateral_db, omega_max]);

    results{i}.D = D;
    results{i}.tuned = tuned;
    results{i}.base = base;

    summary{i,1} = D.name;
    summary{i,2} = rms(D.x);
    summary{i,3} = rms(base.x_sim);
    summary{i,4} = rms(tuned.x_sim);
    summary{i,5} = abs(base.x_sim(end));
    summary{i,6} = abs(tuned.x_sim(end));
    summary{i,7} = rms(tuned.u);
    summary{i,8} = rms(abs(D.psi_meas));
    summary{i,9} = rms(abs(tuned.psi_sim));
end

summaryTable = cell2table(summary, 'VariableNames', ...
    {'File','RMS_MeasuredX','RMS_BaselineSimX','RMS_TunedSimX', ...
     'FinalAbsBaselineX','FinalAbsTunedX','RMS_SteeringCmd', ...
     'RMS_MeasuredPsi','RMS_TunedPsi'});

disp(' ');
disp('=== Summary ===');
disp(summaryTable);

%% Original per-file plots
for i = 1:numel(results)
    R = results{i};
    D = R.D;
    B = R.base;
    T = R.tuned;

    figure('Name', D.name, 'Color','w');
    tiledlayout(3,1,'Padding','compact','TileSpacing','compact');

    nexttile;
    plot(D.t, D.x, 'k-', 'LineWidth',1.3); hold on;
    plot(D.t, B.x_sim, '--', 'LineWidth',1.2);
    plot(D.t, T.x_sim, '-', 'LineWidth',1.5);
    yline(0,'k:');
    grid on;
    xlabel('Time [s]'); ylabel('X lateral [m]');
    title(sprintf('%s  |  lateral error', D.name), 'Interpreter','none');
    legend('Measured X (drifting)','Sim baseline (no correction)','Sim tuned control', 'Location','best');

    nexttile;
    plot(D.t, D.psi_meas, 'k-', 'LineWidth',1.3); hold on;
    plot(D.t, B.psi_sim, '--', 'LineWidth',1.2);
    plot(D.t, T.psi_sim, '-', 'LineWidth',1.5);
    yline(0,'k:');
    grid on;
    xlabel('Time [s]'); ylabel('\psi [rad]');
    title('Bearing / heading surrogate');
    legend('\psi measured = atan2(x,z)','Sim baseline','Sim tuned control', 'Location','best');

    nexttile;
    plot(D.t, T.u, 'LineWidth',1.4); hold on;
    yline(0,'k:');
    yline(omega_max,'r--'); 
    yline(-omega_max,'r--');
    grid on;
    xlabel('Time [s]'); ylabel('\omega_{corr} [rad/s]');
    title('Tuned steering correction');
    legend('\omega_{corr}','saturation limits', 'Location','best');
end

%% Overlay plot across all files
figure('Name','All files overlay', 'Color','w');
tiledlayout(2,1,'Padding','compact','TileSpacing','compact');

nexttile;
hold on; grid on;
for i = 1:numel(results)
    D = results{i}.D;
    T = results{i}.tuned;
    plot(D.t - D.t(1), D.x, '--');
    plot(D.t - D.t(1), T.x_sim, 'LineWidth',1.2);
end
xlabel('Time from start [s]'); ylabel('X lateral [m]');
title('Measured drift vs tuned controlled response (all files)');
legendEntries = {};
for i = 1:numel(results)
    legendEntries{end+1} = sprintf('%s measured', erase(results{i}.D.name,'.csv')); %#ok<SAGROW>
    legendEntries{end+1} = sprintf('%s tuned', erase(results{i}.D.name,'.csv')); %#ok<SAGROW>
end
legend(legendEntries, 'Interpreter','none', 'Location','eastoutside');

nexttile;
hold on; grid on;
for i = 1:numel(results)
    D = results{i}.D;
    T = results{i}.tuned;
    plot(D.t - D.t(1), T.u, 'LineWidth',1.2);
end
xlabel('Time from start [s]'); ylabel('\omega_{corr} [rad/s]');
title('Tuned steering commands across all files');
legend(arrayfun(@(i) erase(results{i}.D.name,'.csv'), 1:numel(results), 'UniformOutput', false), ...
    'Interpreter','none', 'Location','eastoutside');

%% Additional validation plots: path proxy and heading improvement
for i = 1:numel(results)
    R = results{i};
    D = R.D;
    T = R.tuned;
    B = R.base;

    % Forward-distance axis from measured forward speed estimate
    s = zeros(size(D.t));
    for k = 1:numel(D.t)-1
        s(k+1) = s(k) + D.v(k) * D.dt(k);
    end

    figure('Name', [D.name ' - path validation'], 'Color', 'w');
    tiledlayout(2,2,'Padding','compact','TileSpacing','compact');

    % 1) Path proxy: lateral offset vs forward distance
    nexttile;
    plot(s, D.x, 'k--', 'LineWidth', 1.3); hold on;
    plot(s, B.x_sim, '--', 'LineWidth', 1.2);
    plot(s, T.x_sim, '-', 'LineWidth', 1.6);
    yline(0,'k:');
    grid on;
    xlabel('Forward distance traveled [m]');
    ylabel('Lateral offset X [m]');
    title('Path proxy: before vs after');
    legend('Measured drift','Sim baseline','Sim tuned','Location','best');

    % 2) X-Z plane proxy
    nexttile;
    plot(D.z, D.x, 'k--', 'LineWidth', 1.3); hold on;
    plot(D.z, T.x_sim, '-', 'LineWidth', 1.6);
    grid on;
    xlabel('Depth Z [m]');
    ylabel('Lateral offset X [m]');
    title('Camera-frame X-Z path proxy');
    legend('Measured drift','Tuned controlled','Location','best');

    % 3) Heading surrogate improvement
    nexttile;
    plot(D.t, abs(D.psi_meas), 'k--', 'LineWidth', 1.3); hold on;
    plot(D.t, abs(T.psi_sim), '-', 'LineWidth', 1.6);
    grid on;
    xlabel('Time [s]');
    ylabel('|heading surrogate| [rad]');
    title('Heading error magnitude improvement');
    legend('|Measured \psi|','|Tuned \psi|','Location','best');

    % 4) Lateral error improvement magnitude
    nexttile;
    plot(D.t, abs(D.x), 'k--', 'LineWidth', 1.3); hold on;
    plot(D.t, abs(T.x_sim), '-', 'LineWidth', 1.6);
    grid on;
    xlabel('Time [s]');
    ylabel('|X lateral error| [m]');
    title('Lateral error magnitude improvement');
    legend('|Measured X|','|Tuned X|','Location','best');
end

%% Aggregate improvement plot
improveX = zeros(numel(results),1);
improvePsi = zeros(numel(results),1);
names = strings(numel(results),1);

for i = 1:numel(results)
    D = results{i}.D;
    T = results{i}.tuned;
    names(i) = erase(D.name,'.csv');

    improveX(i) = 100 * (rms(abs(D.x)) - rms(abs(T.x_sim))) / max(rms(abs(D.x)), eps);
    improvePsi(i) = 100 * (rms(abs(D.psi_meas)) - rms(abs(T.psi_sim))) / max(rms(abs(D.psi_meas)), eps);
end

figure('Name','Improvement summary', 'Color','w');
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

%% Suggested ROS2 parameters
fprintf('\n=== Suggested ROS2 OUTER-LOOP parameters ===\n');
fprintf('self.declare_parameter("k_heading", %.6f)\n', k_heading);
fprintf('self.declare_parameter("k_lateral", %.6f)\n', k_lateral);
fprintf('self.declare_parameter("heading_deadband", %.6f)\n', heading_db);
fprintf('self.declare_parameter("lateral_deadband", %.6f)\n', lateral_db);
fprintf('self.declare_parameter("max_steering_correction", %.6f)\n', omega_max);

fprintf('\nNOTE:\n');
fprintf(['These logs are enough to tune the CAMERA-based outer loop only.\n' ...
         'To tune kp_left/ki_left and kp_right/ki_right, you need wheel encoder logs\n' ...
         'or a Unity/digital-twin model that outputs actual wheel speeds in response\n' ...
         'to command inputs.\n']);

%% ========================= Local functions =========================
function D = loadArucoCsv(filename)
    T = readtable(filename, 'TextType', 'string');

    if ~ismember("Timestamp", string(T.Properties.VariableNames))
        error('File %s does not contain a Timestamp column.', filename);
    end
    tsec = parseTimestampToSeconds(T.Timestamp);
    tsec = tsec(:);

    requiredCols = ["X_meters","Y_meters","Z_meters"];
    for c = requiredCols
        if ~ismember(c, string(T.Properties.VariableNames))
            error('File %s is missing required column: %s', filename, c);
        end
    end

    finiteMask = isfinite(tsec);
    T = T(finiteMask,:);
    tsec = tsec(finiteMask);

    keep = [true; diff(tsec) > 0];
    T = T(keep,:);
    tsec = tsec(keep);

    if numel(tsec) < 2
        error('File %s has too few valid timestamp samples after cleaning.', filename);
    end

    x = T.X_meters(:);
    z = T.Z_meters(:);

    dt = diff(tsec);
    dt = [dt; dt(end)];

    dz = gradient(z, tsec);
    v = max(0, -dz);   % positive forward speed estimate

    psi_meas = atan2(x, z);

    D.name = filename;
    D.t = tsec;
    D.dt = dt;
    D.x = x;
    D.z = z;
    D.v = v;
    D.psi_meas = psi_meas;
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
            error('Unsupported Timestamp column type: %s', class(ts));
        end
    end

    ts = strtrim(ts);
    ts = ts(:);

    bad = ismissing(ts) | (strlength(ts) == 0);
    if any(bad)
        ts = ts(~bad);
    end

    if isempty(ts)
        error('Timestamp column is empty after cleaning.');
    end

    fmts = [ ...
        "HH:mm:ss.SSS", ...
        "HH:mm:ss", ...
        "yyyy-MM-dd HH:mm:ss.SSS", ...
        "yyyy-MM-dd HH:mm:ss", ...
        "MM/dd/yyyy HH:mm:ss.SSS", ...
        "MM/dd/yyyy HH:mm:ss" ...
    ];

    for k = 1:numel(fmts)
        try
            dtm = datetime(ts, 'InputFormat', fmts(k));
            tsec = seconds(dtm - dtm(1));
            return;
        catch
        end
    end

    durFmts = [ ...
        "hh:mm:ss.SSS", ...
        "hh:mm:ss" ...
    ];

    for k = 1:numel(durFmts)
        try
            dur = duration(ts, 'InputFormat', durFmts(k));
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

function J = totalCost(p, datasets, W, lb, ub)
    p = clampVec(p, lb, ub);

    Jsum = 0;
    for i = 1:numel(datasets)
        R = simulateOuterLoop(datasets{i}, p);

        xNorm = max(0.05, abs(datasets{i}.x(1)));
        psiNorm = max(0.02, abs(datasets{i}.psi_meas(1)));

        J_i = W.rms_x   * rms(R.x_sim / xNorm) + ...
              W.final_x * abs(R.x_sim(end) / xNorm) + ...
              W.rms_psi * rms(R.psi_sim / psiNorm) + ...
              W.rms_u   * rms(R.u) + ...
              W.rms_du  * rms(R.du);

        Jsum = Jsum + J_i;
    end
    J = Jsum / numel(datasets);
end

function R = simulateOuterLoop(D, p)
    % p = [k_heading, k_lateral, heading_db, lateral_db, omega_max]
    k_heading  = p(1);
    k_lateral  = p(2);
    heading_db = p(3);
    lateral_db = p(4);
    omega_max  = p(5);

    N = numel(D.t);
    x = zeros(N,1);
    psi = zeros(N,1);
    u = zeros(N,1);

    x(1) = D.x(1);
    psi(1) = D.psi_meas(1);

    for k = 1:N-1
        Ts = D.dt(k);
        v  = D.v(k);

        ey   = applyDeadband(x(k), lateral_db);
        epsi = applyDeadband(psi(k), heading_db);

        omega_corr = k_heading * epsi + k_lateral * ey;
        omega_corr = clamp(omega_corr, -omega_max, omega_max);
        u(k) = omega_corr;

        % Small-angle approximation:
        % x_dot   = v * psi
        % psi_dot = -omega_corr
        x(k+1)   = x(k) + Ts * v * psi(k);
        psi(k+1) = psi(k) - Ts * omega_corr;
    end

    u(end) = u(max(end-1,1));

    dt_rate = max(D.dt(1:end-1), eps);
    du = [0; diff(u) ./ dt_rate];

    R.x_sim   = x;
    R.psi_sim = psi;
    R.u       = u;
    R.du      = du;
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