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
system_id = 3;
controller_type = 'PI';   % 'PI' | 'PD' | 'PID'

%% =========================
% Optimization settings
%% =========================
nPop = 20;      % Number of stars
MaxIt = 20;     % Maximum iterations
sim_time = 20;

%% =========================
% Paths
%% =========================
mainFolder = fileparts(mfilename('fullpath'));   % Folder containing main.m

% Add initialization folder
initFolder = fullfile(mainFolder, 'initialization');

if ~isfolder(initFolder)
    error('Initialization folder not found: %s', initFolder);
end

addpath(initFolder);

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
% Select system and model path
%% =========================
controllerFolder = "optimized - " + upper(controller_type);

switch system_id
    case 1
        systemFolder = "Ball_and_beam";
        model = "CLBandBBD";
        initFcn = @init_ballandbeam;
        costFcnRaw = @(K) cost_ballandbeam(K, model);
        
    case 2
        systemFolder = "cruise_control";
        model = "CLCCBD";
        initFcn = @init_cruise;
        costFcnRaw = @(K) cost_cruise(K, model);
        
    case 3
        systemFolder = "Motor_speed";
        model = "CLMSBD";
        initFcn = @init_motor;
        costFcnRaw = @(K) cost_motor(K, model);
        
    otherwise
        error('Invalid system_id. Use 1, 2, or 3.');
end

%% =========================
% Build full model folder path
%% =========================
modelFolder = fullfile(mainFolder, controllerFolder, systemFolder);

if ~isfolder(modelFolder)
    error('Model folder not found: %s', modelFolder);
end

addpath(genpath(modelFolder));

%% =========================
% Run initialization file
%% =========================
initFcn();

%% =========================
% Assign simulation settings to base workspace
%% =========================
assignin('base', 'sim_time', sim_time);

% Default reference if not already assigned
if ~evalin('base', 'exist(''ref'', ''var'')')
    assignin('base', 'ref', 1);
end

%% =========================
% Load Simulink model (FULL PATH)
%% =========================
bdclose('all');   % Close any previously opened models

modelPath = fullfile(modelFolder, model + ".slx");

if ~isfile(modelPath)
    error('Model file not found: %s', modelPath);
end

load_system(modelPath);

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
            bestKopt = blackHole(costFcn, nPop, MaxIt, VarMin, VarMax, controller_type);
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
    simOut = sim(model);
    resp = simOut.OutputResponse;

    results.(method).t = resp.Time;
    results.(method).y = resp.Data;
    results.(method).Gains = bestK;
    results.(method).stepinfo = stepinfo(resp.Data, resp.Time);
end

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
for i = 1:length(methods)
    method = methods{i};
    disp(['Step info (' method '):']);
    disp(results.(method).stepinfo);
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
    [Kp, Ki, Kd] = zn_controller(Ku, Tu, controller_type);
    Kfull = [Kp Ki Kd];
end

%% =========================
% Local function: Estimate Ku and Tu without toolbox
%% =========================
function [Ku, Tu] = estimateUltimateGain(model)
    Kp = 1;
    Ki = 0;
    Kd = 0;

    assignin('base','kp',Kp);
    assignin('base','ki',Ki);
    assignin('base','kd',Kd);

    maxIter = 50;
    dKp = 1;
    Ku = NaN;
    Tu = NaN;

    for k = 1:maxIter
        assignin('base','kp',Kp);
        assignin('base','ki',0);
        assignin('base','kd',0);

        simOut = sim(model, 'StopTime', '10');
        y = simOut.OutputResponse.Data;
        t = simOut.OutputResponse.Time;

        dy = diff(y);
        idx = find(dy(1:end-1).*dy(2:end) < 0);

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