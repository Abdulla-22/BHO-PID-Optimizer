clc;
clear;
close all;

%% =========================
% User selections
% system_id:
% 1 = Ball & Beam
% 2 = Cruise Control
% 3 = Motor Speed
% 4 = Custom Transfer Function
%
% controller_type:
% 'PI'  = Proportional Integral
% 'PD'  = Proportional Derivative
% 'PID' = Proportional Integral Derivative
%
% optimization_mode:
% 1 = BEST
% 2 = CONSTRAINED
%
% comparison:
% true  = Show BHO vs ZN
% false = Show BHO only
%% =========================
system_id = 3;
controller_type = 'PI';
optimization_mode = 1;
comparison = false;

%% =========================
% Custom transfer function
% Used only when system_id = 4
%% =========================
custom_tf.num = [1];
custom_tf.den = [1 5];
custom_tf.step_amp = 1;

%% =========================
% User performance specifications
% Used only in CONSTRAINED mode
%% =========================
wantedovershoot = 1.0;     % In percentage
wantedrisetime  = 0.01;    % In seconds
wantedess       = 1.0;     % In percentage

%% =========================
% Optimization settings
%% =========================
nPop     = 10;
MaxIt    = 10;
sim_time = 5;

%% =========================
% Controller-system compatibility check
%% =========================
if system_id == 1 && strcmpi(controller_type, 'PI')
    error(['PI controller is not suitable for the Ball and Beam system. ', ...
           'Please choose PD or PID controller.']);
end

if system_id == 2 && strcmpi(controller_type, 'PD')
    warning(['PD controller may not be 100%% efficient for the Cruise Control system. ', ...
             'It may cause steady-state error and may not track the desired reference accurately.']);
end

%% =========================
% Paths
%% =========================
mainFolder = fileparts(mfilename('fullpath'));

initFolder = fullfile(mainFolder, 'Initialization');
if ~isfolder(initFolder)
    error('Initialization folder not found: %s', initFolder);
end
addpath(genpath(initFolder));

costFolder = fullfile(mainFolder, 'Costs Functions');
if ~isfolder(costFolder)
    error('Costs Functions folder not found: %s', costFolder);
end
addpath(genpath(costFolder));

tuningFolder = fullfile(mainFolder, 'Tuning Methods');
if ~isfolder(tuningFolder)
    error('Tuning Methods folder not found: %s', tuningFolder);
end
addpath(genpath(tuningFolder));

%% =========================
% Select controller search space
%% =========================
switch upper(controller_type)
    case 'PI'
        VarMin = [0 0];
        VarMax = [1000 1000];

    case 'PD'
        VarMin = [0 0];
        VarMax = [1000 100];

    case 'PID'
        VarMin = [0 0 0];
        VarMax = [1000 500 200];

    otherwise
        error('Invalid controller_type. Use PI, PD, or PID.');
end

%% =========================
% Select system, model name, initialization, and cost function
%% =========================
isCustomSystem = false;

switch upper(controller_type)

    case 'PI'
        switch system_id
            case 1
                model      = "BallandBeamPI";
                initFcn    = @init_ballandbeam;
                costFcnRaw = @(K) cost_ballandbeam(K, model);

            case 2
                model      = "CruiseControlPI";
                initFcn    = @init_cruise;
                costFcnRaw = @(K) cost_cruise(K, model);

            case 3
                model      = "MotorSpeedPI";
                initFcn    = @init_motor;
                costFcnRaw = @(K) cost_motor(K, model);

            case 4
                model      = "CustomSystemPI";
                initFcn    = @() assignCustomTFToBase(custom_tf);
                costFcnRaw = @(K) cost_custom(K, model);
                isCustomSystem = true;

            otherwise
                error('Invalid system_id. Use 1, 2, 3, or 4.');
        end

    case 'PD'
        switch system_id
            case 1
                model      = "BallandBeamPD";
                initFcn    = @init_ballandbeam;
                costFcnRaw = @(K) cost_ballandbeam(K, model);

            case 2
                model      = "CruiseControlPD";
                initFcn    = @init_cruise;
                costFcnRaw = @(K) cost_cruise(K, model);

            case 3
                model      = "MotorSpeedPD";
                initFcn    = @init_motor;
                costFcnRaw = @(K) cost_motor(K, model);

            case 4
                model      = "CustomSystemPD";
                initFcn    = @() assignCustomTFToBase(custom_tf);
                costFcnRaw = @(K) cost_custom(K, model);
                isCustomSystem = true;

            otherwise
                error('Invalid system_id. Use 1, 2, 3, or 4.');
        end

    case 'PID'
        switch system_id
            case 1
                model      = "BallandBeamPID";
                initFcn    = @init_ballandbeam;
                costFcnRaw = @(K) cost_ballandbeam(K, model);

            case 2
                model      = "CruiseControlPID";
                initFcn    = @init_cruise;
                costFcnRaw = @(K) cost_cruise(K, model);

            case 3
                model      = "MotorSpeedPID";
                initFcn    = @init_motor;
                costFcnRaw = @(K) cost_motor(K, model);

            case 4
                model      = "CustomSystemPID";
                initFcn    = @() assignCustomTFToBase(custom_tf);
                costFcnRaw = @(K) cost_custom(K, model);
                isCustomSystem = true;

            otherwise
                error('Invalid system_id. Use 1, 2, 3, or 4.');
        end

    otherwise
        error('Invalid controller_type. Use PI, PD, or PID.');
end

%% =========================
% Build full model folder path
%% =========================
modelFolder = fullfile(mainFolder, 'Simulink Models', upper(controller_type));

if ~isfolder(modelFolder)
    error('Model folder not found: %s', modelFolder);
end

addpath(genpath(modelFolder));

%% =========================
% Run initialization once
%% =========================
initFcn();

%% =========================
% Assign settings to base workspace
%% =========================
assignin('base', 'sim_time', sim_time);
assignin('base', 'optimization_mode', optimization_mode);
assignin('base', 'wantedovershoot', wantedovershoot);
assignin('base', 'wantedrisetime', wantedrisetime);
assignin('base', 'wantedess', wantedess);

if ~evalin('base', 'exist(''ref'', ''var'')')
    assignin('base', 'ref', 1);
end

%% =========================
% Reset stop flag
%% =========================
setappdata(0, 'BHO_Stop', false);

%% =========================
% Load Simulink model
%% =========================
bdclose('all');

modelPath = fullfile(modelFolder, model + ".slx");

if ~isfile(modelPath)
    error('Model file not found: %s', modelPath);
end

load_system(modelPath);

%% =========================
% Speed-up settings
%% =========================
set_param(model, 'SimulationMode', 'accelerator');
set_param(model, 'FastRestart', 'on');

%% =========================
% Wrap cost function based on controller type
%% =========================
costFcn = @(Kopt) controllerCostWrapper(Kopt, controller_type, costFcnRaw);

%% =========================
% Initialize storage for comparison
%% =========================
if comparison
    methods = {'BHO', 'ZN'};
else
    methods = {'BHO'};
end

results = struct();
validMethods = {};

for i = 1:length(methods)

    method = methods{i};

    switch upper(method)

        case 'BHO'
            bestKopt = BlackHoleAlgorithm(costFcn, nPop, MaxIt, VarMin, VarMax, controller_type);
            bestK = expandControllerGains(bestKopt, controller_type);

        case 'ZN'
            if isCustomSystem
                [Ku, Tu] = estimateUltimateGainCustom(model, custom_tf, sim_time);
            else
                [Ku, Tu] = estimateUltimateGain(model);
            end

            if isnan(Ku) || isnan(Tu) || isinf(Ku) || isinf(Tu)
                warning('ZN tuning skipped because Ku/Tu could not be estimated.');
                continue;
            end

            bestK = znGainsByType(Ku, Tu, controller_type);

            if any(isnan(bestK)) || any(isinf(bestK))
                warning('ZN tuning skipped because the computed gains are invalid.');
                continue;
            end

        otherwise
            error('Unknown tuning method');
    end

    disp(['=== BEST ' upper(controller_type) ' GAINS (' method ') ===']);
    disp(bestK);

    assignin('base', 'kp', bestK(1));
    assignin('base', 'ki', bestK(2));
    assignin('base', 'kd', bestK(3));

    if isCustomSystem
        assignCustomTFToBase(custom_tf);
    end

    simOut = sim(model, 'StopTime', num2str(sim_time), 'CaptureErrors', 'on');

    if ~isempty(simOut.ErrorMessage)
        error('Simulation error in %s method: %s', method, simOut.ErrorMessage);
    end

    resp = simOut.OutputResponse;

    t = resp.Time(:);
    y = squeeze(resp.Data);

    if isempty(y)
        error('OutputResponse.Data is empty in %s method.', method);
    end

    if isrow(y)
        y = y.';
    end

    if size(y,2) > 1
        y = y(:,1);
    end

    ref = getReferenceFromBase();

    try
        info = stepinfo(y, t, ref, 'SettlingTimeThreshold', 0.02);
    catch
        info = stepinfo(y, t, ref);
    end

    results.(method).t        = t;
    results.(method).y        = y;
    results.(method).Gains    = bestK;
    results.(method).stepinfo = info;

    actual_ESS = abs(ref - y(end));
    if abs(ref) > 0
        results.(method).ess = (actual_ESS / abs(ref)) * 100;
    else
        results.(method).ess = actual_ESS;
    end

    validMethods{end + 1} = method; %#ok<AGROW>
end

%% =========================
% Turn Fast Restart off after finishing
%% =========================
set_param(model, 'FastRestart', 'off');

if isempty(validMethods)
    error('No valid tuning method could be completed.');
end

%% =========================
% Plot result(s)
%% =========================
figure;
hold on;
grid on;

colors = {'b', 'r', 'k', 'g'};

for i = 1:length(validMethods)
    method = validMethods{i};

    tplot = results.(method).t(:);
    yplot = results.(method).y(:);

    plot(tplot, yplot, ...
        'LineWidth', 2, ...
        'Color', colors{i}, ...
        'DisplayName', method);
end

xlabel('Time [s]');
ylabel('Output');

if length(validMethods) > 1
    title([upper(controller_type) ' Comparison']);
else
    title([upper(controller_type) ' Response']);
end

legend('show', 'Location', 'best');
hold off;

%% =========================
% Display results
%% =========================
disp(' ');
fprintf('=== OPTIMIZATION MODE = %d ===\n', optimization_mode);

if optimization_mode == 2
    fprintf('Max Overshoot     = %.6f %%\n', wantedovershoot);
    fprintf('Max Rise Time     = %.6f s\n', wantedrisetime);
    fprintf('Max ESS           = %.6f %%\n', wantedess);
end

for i = 1:length(validMethods)
    method = validMethods{i};
    info = results.(method).stepinfo;

    fprintf('\n=== Step info (%s) ===\n', method);
    fprintf('Rise Time      = %.6f s\n', info.RiseTime);
    fprintf('Settling Time  = %.6f s\n', info.SettlingTime);
    fprintf('Overshoot      = %.6f %%\n', info.Overshoot);
    fprintf('Peak Time      = %.6f s\n', info.PeakTime);
    fprintf('ESS            = %.6f %%\n', results.(method).ess);
end

%% =========================
% Local function: assign custom TF to base workspace
%% =========================
function assignCustomTFToBase(custom_tf)

    if ~isstruct(custom_tf)
        error('custom_tf must be a struct.');
    end

    if ~isfield(custom_tf, 'num') || ~isfield(custom_tf, 'den') || ~isfield(custom_tf, 'step_amp')
        error('custom_tf must contain num, den, and step_amp.');
    end

    if isempty(custom_tf.num) || isempty(custom_tf.den)
        error('custom_tf.num and custom_tf.den cannot be empty.');
    end

    assignin('base', 'num', custom_tf.num);
    assignin('base', 'den', custom_tf.den);
    assignin('base', 'step_amp', custom_tf.step_amp);
end

%% =========================
% Local function: controller cost wrapper
%% =========================
function cost = controllerCostWrapper(Kopt, controller_type, costFcnRaw)
    Kfull = expandControllerGains(Kopt, controller_type);
    cost = costFcnRaw(Kfull);
end

%% =========================
% Local function: expand gains to [Kp Ki Kd]
%% =========================
function Kfull = expandControllerGains(Kopt, controller_type)

    switch upper(controller_type)
        case 'PI'
            Kfull = [Kopt(1) Kopt(2) 0];

        case 'PD'
            Kfull = [Kopt(1) 0 Kopt(2)];

        case 'PID'
            Kfull = [Kopt(1) Kopt(2) Kopt(3)];

        otherwise
            error('Invalid controller_type.');
    end
end

%% =========================
% Local function: ZN gains by controller type
%% =========================
function Kfull = znGainsByType(Ku, Tu, controller_type)
    [Kp, Ki, Kd] = ZieglerNichols(Ku, Tu, controller_type);
    Kfull = [Kp Ki Kd];
end

%% =========================
% Local function: get reference from base workspace
%% =========================
function ref = getReferenceFromBase()

    if evalin('base', 'exist(''step_amp'', ''var'')')
        ref = evalin('base', 'step_amp');
    elseif evalin('base', 'exist(''ref'', ''var'')')
        ref = evalin('base', 'ref');
    else
        ref = 1;
    end
end

%% =========================
% Local function: estimate Ku and Tu for normal systems
%% =========================
function [Ku, Tu] = estimateUltimateGain(model)

    Kp = 0.5;
    Ki = 0;
    Kd = 0;

    assignin('base', 'kp', Kp);
    assignin('base', 'ki', Ki);
    assignin('base', 'kd', Kd);

    maxIter = 20;
    dKp = 0.5;

    Ku = NaN;
    Tu = NaN;

    for k = 1:maxIter

        assignin('base', 'kp', Kp);
        assignin('base', 'ki', 0);
        assignin('base', 'kd', 0);

        simOut = sim(model, 'StopTime', '5', 'CaptureErrors', 'on');

        if ~isempty(simOut.ErrorMessage)
            warning('Simulation error while estimating Ku/Tu at Kp = %.4f', Kp);
            break;
        end

        y = simOut.OutputResponse.Data;
        t = simOut.OutputResponse.Time;

        if isempty(y) || isempty(t) || any(isnan(y)) || any(isinf(y)) || max(abs(y)) > 100
            warning('Unstable or invalid response detected while estimating Ku/Tu at Kp = %.4f', Kp);
            break;
        end

        dy = diff(y);
        idx = find(dy(1:end-1) .* dy(2:end) < 0);

        if numel(idx) >= 6
            Ku = Kp;
            crossingTimes = t(idx + 1);
            Tu = mean(diff(crossingTimes)) * 2;
            return;
        end

        Kp = Kp + dKp;
    end

    warning('No sustained oscillation detected. Use Ku and Tu manually.');
end

%% =========================
% Local function: estimate Ku and Tu for custom system
%% =========================
function [Ku, Tu] = estimateUltimateGainCustom(model, custom_tf, sim_time)

    Kp = 0.5;
    Ki = 0;
    Kd = 0;

    maxIter = 20;
    dKp = 0.5;

    Ku = NaN;
    Tu = NaN;

    for k = 1:maxIter

        assignin('base', 'kp', Kp);
        assignin('base', 'ki', Ki);
        assignin('base', 'kd', Kd);

        assignin('base', 'num', custom_tf.num);
        assignin('base', 'den', custom_tf.den);
        assignin('base', 'step_amp', custom_tf.step_amp);

        simOut = sim(model, 'StopTime', num2str(sim_time), 'CaptureErrors', 'on');

        if ~isempty(simOut.ErrorMessage)
            warning('Simulation error while estimating Ku/Tu at Kp = %.4f', Kp);
            break;
        end

        y = simOut.OutputResponse.Data;
        t = simOut.OutputResponse.Time;

        if isempty(y) || isempty(t) || any(isnan(y)) || any(isinf(y)) || max(abs(y)) > 1e6
            warning('Unstable or invalid response detected while estimating Ku/Tu at Kp = %.4f', Kp);
            break;
        end

        dy = diff(y);
        idx = find(dy(1:end-1) .* dy(2:end) < 0);

        if numel(idx) >= 6
            Ku = Kp;
            crossingTimes = t(idx + 1);
            Tu = mean(diff(crossingTimes)) * 2;
            return;
        end

        Kp = Kp + dKp;
    end

    warning('No sustained oscillation detected for custom system. Use Ku and Tu manually.');
end