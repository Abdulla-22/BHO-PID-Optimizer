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
%
% collect_results:
% true  = Save command window log and BHO iteration history
% false = Do not save files
%% =========================
system_id = 4;
controller_type = 'PID';
optimization_mode = 1;
comparison = true;
collect_results = true;

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
nPop     = 20;
MaxIt    = 20;
sim_time = 5;

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
% Prepare results saving
%% =========================
diaryStarted = false;
logFilePath  = '';
csvFilePath  = '';
pngFilePath  = '';
bhoHistory   = table();

if collect_results
    systemName = getSystemName(system_id);
    [resultsFolder, baseFileName] = prepareResultsPaths(mainFolder, systemName, controller_type);

    logFilePath = fullfile(resultsFolder, [baseFileName '.txt']);
    csvFilePath = fullfile(resultsFolder, [baseFileName '.csv']);
    pngFilePath = fullfile(resultsFolder, [baseFileName '.png']);

    diary off;
    diary(logFilePath);
    diaryStarted = true;

    fprintf('=============================================\n');
    fprintf('Results collection is enabled.\n');
    fprintf('Text log file : %s\n', logFilePath);
    fprintf('CSV results   : %s\n', csvFilePath);
    fprintf('PNG plot file : %s\n', pngFilePath);
    fprintf('=============================================\n\n');
end

cleanupObj = onCleanup(@() safeDiaryOff(diaryStarted)); %#ok<NASGU>

try
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
    % Clean old generated files before loading model
    %% =========================
    bdclose('all');
    cleanSimulinkGeneratedFiles(mainFolder);

    %% =========================
    % Load Simulink model
    %% =========================
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
                [bestKopt, bhoHistory] = BlackHoleAlgorithm( ...
                    costFcn, nPop, MaxIt, VarMin, VarMax, controller_type, model);

                if isempty(bestKopt)
                    error('BHO did not return a valid solution.');
                end

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
        t = squeeze(resp.Time);
        y = squeeze(resp.Data);

        t = t(:);
        y = y(:);

        if isempty(t) || isempty(y)
            error('OutputResponse is empty in %s method.', method);
        end

        if numel(t) ~= numel(y)
            error('Time and output length mismatch in %s method.', method);
        end

        if numel(t) < 2
            error(['OutputResponse in ' method ' contains less than 2 samples. ', ...
                   'Check the output logging settings in the Simulink model.']);
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
    fig = figure;
    hold on;
    grid on;
    box on;

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
    % Save plot to PNG
    %% =========================
    if collect_results
        try
            exportgraphics(fig, pngFilePath, 'Resolution', 300);
            fprintf('\nPlot image saved to:\n%s\n', pngFilePath);
        catch
            try
                saveas(fig, pngFilePath);
                fprintf('\nPlot image saved to:\n%s\n', pngFilePath);
            catch ME_png
                warning('Could not save plot as PNG: %s', ME_png.message);
            end
        end
    end

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
    % Save BHO history to CSV
    %% =========================
    if collect_results
        if ~isempty(bhoHistory)
            writetable(bhoHistory, csvFilePath);
            fprintf('\nBHO iteration history saved to:\n%s\n', csvFilePath);
        else
            warning('collect_results is true, but BHO history table is empty.');
        end
    end

catch ME
    try
        if bdIsLoaded(model)
            set_param(model, 'FastRestart', 'off');
        end
    catch
    end
    rethrow(ME);
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
% Local function: get system name
%% =========================
function systemName = getSystemName(system_id)
    switch system_id
        case 1
            systemName = 'BallandBeam';
        case 2
            systemName = 'CruiseControl';
        case 3
            systemName = 'DCMotorSpeed';
        case 4
            systemName = 'CustomSystem';
        otherwise
            error('Invalid system_id.');
    end
end

%% =========================
% Local function: prepare result paths
%% =========================
function [resultsFolder, baseFileName] = prepareResultsPaths(mainFolder, systemName, controller_type)
    resultsRoot = fullfile(mainFolder, 'Results');

    if ~isfolder(resultsRoot)
        mkdir(resultsRoot);
    end

    resultsFolder = fullfile(resultsRoot, systemName, upper(controller_type));

    if ~isfolder(resultsFolder)
        mkdir(resultsFolder);
    end

    testNumber = getNextTestNumber(resultsFolder, systemName, controller_type);
    baseFileName = sprintf('%s_%s_test%d', systemName, upper(controller_type), testNumber);
end

%% =========================
% Local function: get next test number
%% =========================
function testNumber = getNextTestNumber(resultsFolder, systemName, controller_type)
    patternTxt = fullfile(resultsFolder, sprintf('%s_%s_test*.txt', systemName, upper(controller_type)));
    patternCsv = fullfile(resultsFolder, sprintf('%s_%s_test*.csv', systemName, upper(controller_type)));

    filesTxt = dir(patternTxt);
    filesCsv = dir(patternCsv);
    allFiles = [filesTxt; filesCsv];

    maxNum = 0;

    for k = 1:numel(allFiles)
        token = regexp(allFiles(k).name, '_test(\d+)\.', 'tokens', 'once');
        if ~isempty(token)
            fileNum = str2double(token{1});
            if ~isnan(fileNum)
                maxNum = max(maxNum, fileNum);
            end
        end
    end

    testNumber = maxNum + 1;
end

%% =========================
% Local function: safe diary off
%% =========================
function safeDiaryOff(diaryStarted)
    if diaryStarted
        try
            diary off;
        catch
        end
    end
end

%% =========================
% Local function: clean old Simulink generated files
%% =========================
function cleanSimulinkGeneratedFiles(mainFolder)
    foldersToDelete = { ...
        fullfile(mainFolder, 'slprj'), ...
        fullfile(mainFolder, 'simulinkcache')};

    for i = 1:numel(foldersToDelete)
        thisFolder = foldersToDelete{i};
        if exist(thisFolder, 'dir')
            try
                rmdir(thisFolder, 's');
                fprintf('Deleted old generated folder: %s\n', thisFolder);
            catch ME
                warning('Could not delete folder: %s\nReason: %s', thisFolder, ME.message);
            end
        end
    end

    try
        Simulink.fileGenControl('reset');
        fprintf('Simulink file generation cache reset successfully.\n');
    catch ME
        warning('Could not reset Simulink file generation settings: %s', ME.message);
    end
end

%% =========================
% Local function: estimate Ku and Tu for normal systems
%% =========================
function [Ku, Tu] = estimateUltimateGain(model)
    Kp_low = 0.1;
    Kp_high = 100;
    maxSearchIter = 30;
    sim_time_local = 15;
    Ku = NaN;
    Tu = NaN;

    [status, ~] = checkSustainedOscillation(model, Kp_high, sim_time_local);

    while status == -1 && Kp_high < 10000
        Kp_low = Kp_high;
        Kp_high = Kp_high * 2;
        [status, ~] = checkSustainedOscillation(model, Kp_high, sim_time_local);
    end

    if status == -1
        warning('System did not reach instability. It might be inherently stable.');
        return;
    end

    for k = 1:maxSearchIter
        Kp_test = (Kp_low + Kp_high) / 2;
        [status, Tu_candidate] = checkSustainedOscillation(model, Kp_test, sim_time_local);

        if status == 0
            Ku = Kp_test;
            Tu = Tu_candidate;
            return;
        elseif status == 1
            Kp_high = Kp_test;
            if ~isnan(Tu_candidate)
                Tu = Tu_candidate;
            end
        else
            Kp_low = Kp_test;
        end
    end

    Ku = Kp_high;
end

%% =========================
% Local function: estimate Ku and Tu for custom system
%% =========================
function [Ku, Tu] = estimateUltimateGainCustom(model, custom_tf, sim_time_local)
    Kp_low = 0.1;
    Kp_high = 100;
    maxSearchIter = 30;
    Ku = NaN;
    Tu = NaN;

    [status, ~] = checkSustainedOscillationCustom(model, custom_tf, Kp_high, sim_time_local);

    while status == -1 && Kp_high < 10000
        Kp_low = Kp_high;
        Kp_high = Kp_high * 2;
        [status, ~] = checkSustainedOscillationCustom(model, custom_tf, Kp_high, sim_time_local);
    end

    if status == -1
        warning('Custom system did not reach instability. It might be inherently stable.');
        return;
    end

    for k = 1:maxSearchIter
        Kp_test = (Kp_low + Kp_high) / 2;
        [status, Tu_candidate] = checkSustainedOscillationCustom(model, custom_tf, Kp_test, sim_time_local);

        if status == 0
            Ku = Kp_test;
            Tu = Tu_candidate;
            return;
        elseif status == 1
            Kp_high = Kp_test;
            if ~isnan(Tu_candidate)
                Tu = Tu_candidate;
            end
        else
            Kp_low = Kp_test;
        end
    end

    Ku = Kp_high;
end

%% =========================
% Local function: oscillation check for normal system
% Returns: 1 (Unstable/Growing), -1 (Stable/Decaying), 0 (Sustained)
%% =========================
function [status, Tu] = checkSustainedOscillation(model, Kp_test, sim_time_local)
    status = -1;
    Tu = NaN;

    assignin('base', 'kp', Kp_test);
    assignin('base', 'ki', 0);
    assignin('base', 'kd', 0);

    try
        simOut = sim(model, 'StopTime', num2str(sim_time_local), 'CaptureErrors', 'on');

        if ~isempty(simOut.ErrorMessage)
            status = 1;
            return;
        end

        y = squeeze(simOut.OutputResponse.Data);
        t = squeeze(simOut.OutputResponse.Time);

    catch
        status = 1;
        return;
    end

    if isempty(y) || numel(y) < 20 || any(isnan(y)) || any(isinf(y)) || max(abs(y)) > 1e6
        status = 1;
        return;
    end

    start_idx = round(0.3 * numel(y));
    y2 = y(start_idx:end);
    t2 = t(start_idx:end);

    peak_idx = [];
    for i = 2:length(y2)-1
        if y2(i) > y2(i-1) && y2(i) >= y2(i+1)
            peak_idx(end+1) = i; %#ok<AGROW>
        end
    end

    if numel(peak_idx) < 3
        status = -1;
        return;
    end

    peak_times = t2(peak_idx);
    peak_vals  = y2(peak_idx);

    nUse = min(5, numel(peak_times));
    p_vals = peak_vals(end-nUse+1:end);
    p_times = peak_times(end-nUse+1:end);

    periods = diff(p_times);
    if isempty(periods) || any(periods <= 0)
        return;
    end

    Tu = mean(periods);
    amp_ratio = abs(p_vals(end)) / (abs(p_vals(1)) + eps);

    if amp_ratio > 1.05
        status = 1;
    elseif amp_ratio < 0.95
        status = -1;
    else
        status = 0;
    end
end

%% =========================
% Local function: oscillation check for custom system
% Returns: 1 (Unstable/Growing), -1 (Stable/Decaying), 0 (Sustained)
%% =========================
function [status, Tu] = checkSustainedOscillationCustom(model, custom_tf, Kp_test, sim_time_local)
    status = -1;
    Tu = NaN;

    assignin('base', 'kp', Kp_test);
    assignin('base', 'ki', 0);
    assignin('base', 'kd', 0);
    assignin('base', 'num', custom_tf.num);
    assignin('base', 'den', custom_tf.den);
    assignin('base', 'step_amp', custom_tf.step_amp);

    try
        simOut = sim(model, 'StopTime', num2str(sim_time_local), 'CaptureErrors', 'on');

        if ~isempty(simOut.ErrorMessage)
            status = 1;
            return;
        end

        y = squeeze(simOut.OutputResponse.Data);
        t = squeeze(simOut.OutputResponse.Time);

    catch
        status = 1;
        return;
    end

    if isempty(y) || numel(y) < 20 || any(isnan(y)) || any(isinf(y)) || max(abs(y)) > 1e6
        status = 1;
        return;
    end

    start_idx = round(0.3 * numel(y));
    y2 = y(start_idx:end);
    t2 = t(start_idx:end);

    peak_idx = [];
    for i = 2:length(y2)-1
        if y2(i) > y2(i-1) && y2(i) >= y2(i+1)
            peak_idx(end+1) = i; %#ok<AGROW>
        end
    end

    if numel(peak_idx) < 3
        status = -1;
        return;
    end

    peak_times = t2(peak_idx);
    peak_vals  = y2(peak_idx);

    nUse = min(5, numel(peak_times));
    p_vals = peak_vals(end-nUse+1:end);
    p_times = peak_times(end-nUse+1:end);

    periods = diff(p_times);
    if isempty(periods) || any(periods <= 0)
        return;
    end

    Tu = mean(periods);
    amp_ratio = abs(p_vals(end)) / (abs(p_vals(1)) + eps);

    if amp_ratio > 1.05
        status = 1;
    elseif amp_ratio < 0.95
        status = -1;
    else
        status = 0;
    end
end