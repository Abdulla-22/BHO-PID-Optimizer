clc;
clear;
close all;

%% =========================
% User selections
% system_id:
% 1 = Ball & Beam
% 2 = Cruise Control
% 3 = Motor Speed
%
% controller_type:
% 'PI'  = Proportional Integral
% 'PD'  = Proportional Derivative
% 'PID' = Proportional Integral Derivative
%% =========================
system_id = 2;
controller_type = 'PI';   % 'PI' | 'PD' | 'PID'

%% =========================
% User performance specifications
%% =========================
wantedovershoot    = 10;      % Percent
wantedrisetime     = 3;    % Seconds
wantedsettlingtime = 7;      % Seconds
wantedess          = 0.1;   % Absolute steady-state error

%% =========================
% Optimization settings
%% =========================
nPop = 20;       % Number of stars
MaxIt = 20;     % Maximum iterations
sim_time = 60;

%% =========================
% Paths
%% =========================
mainFolder = fileparts(mfilename('fullpath'));   % Folder containing main.m

% Initialization folder
initFolder = fullfile(mainFolder, 'Initialization');
if ~isfolder(initFolder)
    error('Initialization folder not found: %s', initFolder);
end
addpath(genpath(initFolder));

% Cost functions folder
costFolder = fullfile(mainFolder, 'Costs Functions');
if ~isfolder(costFolder)
    error('Costs Functions folder not found: %s', costFolder);
end
addpath(genpath(costFolder));

% Tuning methods folder
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
        VarMin = [0 0];       % [Kp Ki]
        VarMax = [1000 1000];
        
    case 'PD'
        VarMin = [0 0];       % [Kp Kd]
        VarMax = [1000 1000];
        
    case 'PID'
        VarMin = [0 0 0];     % [Kp Ki Kd]
        VarMax = [1000 1000 1000];
        
    otherwise
        error('Invalid controller_type. Use PI, PD, or PID.');
end

%% =========================
% Select system, model name, initialization, and cost function
%% =========================
switch upper(controller_type)
    case 'PI'
        switch system_id
            case 1
                model = "BallandBeamPI";
                initFcn = @init_ballandbeam;
                costFcnRaw = @(K) cost_ballandbeam(K, model);
            case 2
                model = "CruiseControlPI";
                initFcn = @init_cruise;
                costFcnRaw = @(K) cost_cruise(K, model);
            case 3
                model = "MotorSpeedPI";
                initFcn = @init_motor;
                costFcnRaw = @(K) cost_motor(K, model);
            otherwise
                error('Invalid system_id. Use 1, 2, or 3.');
        end

    case 'PD'
        switch system_id
            case 1
                model = "BallandBeamPD";
                initFcn = @init_ballandbeam;
                costFcnRaw = @(K) cost_ballandbeam(K, model);
            case 2
                model = "CruiseControlPD";
                initFcn = @init_cruise;
                costFcnRaw = @(K) cost_cruise(K, model);
            case 3
                model = "MotorSpeedPD";
                initFcn = @init_motor;
                costFcnRaw = @(K) cost_motor(K, model);
            otherwise
                error('Invalid system_id. Use 1, 2, or 3.');
        end

    case 'PID'
        switch system_id
            case 1
                model = "BallandBeamPID";
                initFcn = @init_ballandbeam;
                costFcnRaw = @(K) cost_ballandbeam(K, model);
            case 2
                model = "CruiseControlPID";
                initFcn = @init_cruise;
                costFcnRaw = @(K) cost_cruise(K, model);
            case 3
                model = "MotorSpeedPID";
                initFcn = @init_motor;
                costFcnRaw = @(K) cost_motor(K, model);
            otherwise
                error('Invalid system_id. Use 1, 2, or 3.');
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
assignin('base', 'wantedovershoot', wantedovershoot);
assignin('base', 'wantedrisetime', wantedrisetime);
assignin('base', 'wantedsettlingtime', wantedsettlingtime);
assignin('base', 'wantedess', wantedess);

% Default reference if not already assigned
if ~evalin('base', 'exist(''ref'', ''var'')')
    assignin('base', 'ref', 1);
end

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
methods = {'BHO','ZN'};
results = struct();

for i = 1:length(methods)
    method = methods{i};
    
    switch upper(method)
        case 'BHO'
            bestKopt = BlackHoleAlgorithm(costFcn, nPop, MaxIt, VarMin, VarMax, controller_type);
            bestK = expandControllerGains(bestKopt, controller_type);

        case 'ZN'
            [Ku, Tu] = estimateUltimateGain(model);
            bestK = znGainsByType(Ku, Tu, controller_type);

        otherwise
            error('Unknown tuning method');
    end

    disp(['=== BEST ' upper(controller_type) ' GAINS (' method ') ===']);
    disp(bestK);

    % Assign gains to base workspace for Simulink
    assignin('base','kp',bestK(1));
    assignin('base','ki',bestK(2));
    assignin('base','kd',bestK(3));

    % Simulate model
    simOut = sim(model, 'StopTime', num2str(sim_time), 'CaptureErrors', 'on');

    if ~isempty(simOut.ErrorMessage)
        error('Simulation error in %s method: %s', method, simOut.ErrorMessage);
    end

    resp = simOut.OutputResponse;

    results.(method).t = resp.Time;
    results.(method).y = resp.Data;
    results.(method).Gains = bestK;
    results.(method).stepinfo = stepinfo(resp.Data, resp.Time);
end

%% =========================
% Turn Fast Restart off after finishing
%% =========================
set_param(model, 'FastRestart', 'off');

%% =========================
% Plot comparison
%% =========================
figure;
hold on;
grid on;

colors = {'b','r'};

for i = 1:length(methods)
    method = methods{i};
    plot(results.(method).t, results.(method).y, ...
        'LineWidth', 2, ...
        'Color', colors{i});
end

xlabel('Time [s]');
ylabel('Output');
title([upper(controller_type) ' Comparison: BHO vs Ziegler-Nichols']);
legend(methods, 'Location', 'best');

%% =========================
% Display step info comparison
%% =========================
disp(' ');
disp('=== USER SPECIFICATIONS ===');
fprintf('Wanted Overshoot     = %.4f %%\n', wantedovershoot);
fprintf('Wanted Rise Time     = %.4f s\n', wantedrisetime);
fprintf('Wanted Settling Time = %.4f s\n', wantedsettlingtime);
fprintf('Wanted ESS           = %.6f\n', wantedess);

for i = 1:length(methods)
    method = methods{i};
    info = results.(method).stepinfo;
    
    fprintf('\n=== Step info (%s) ===\n', method);
    fprintf('Rise Time      = %.6f s\n', info.RiseTime);
    fprintf('Settling Time  = %.6f s\n', info.SettlingTime);
    fprintf('Overshoot      = %.6f %%\n', info.Overshoot);
    fprintf('Peak Time      = %.6f s\n', info.PeakTime);
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
% Local function: Estimate Ku and Tu without toolbox
%% =========================
function [Ku, Tu] = estimateUltimateGain(model)

    Kp = 0.5;
    Ki = 0;
    Kd = 0;

    assignin('base','kp',Kp);
    assignin('base','ki',Ki);
    assignin('base','kd',Kd);

    maxIter = 20;
    dKp = 0.5;
    Ku = NaN;
    Tu = NaN;

    for k = 1:maxIter
        assignin('base','kp',Kp);
        assignin('base','ki',0);
        assignin('base','kd',0);

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
    Ku = Kp;
    Tu = 1;

end