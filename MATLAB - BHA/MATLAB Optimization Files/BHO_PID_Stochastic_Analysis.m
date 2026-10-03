clc;
clear;
close all;

%% ========================================================================
% BHO PID Stochastic Analysis
%
% Purpose:
%   Run repeated independent BHO optimizations for PID controllers of:
%       1) Ball and Beam
%       2) Cruise Control
%       3) DC Motor Speed
%
% This script intentionally reuses the existing project files:
%   - Initialization/init_ballandbeam.m
%   - Initialization/init_cruise.m
%   - Initialization/init_motor.m
%   - Costs Functions/cost_ballandbeam.m
%   - Costs Functions/cost_cruise.m
%   - Costs Functions/cost_motor.m
%   - Tuning Methods/BlackHoleAlgorithm.m
%
% The cost function is NOT reimplemented here. Each BHO candidate is
% evaluated by the corresponding existing cost_*.m file.
%
% The script is designed for the PID stochastic analysis reported in the
% research-paper revision using Best Values Mode only.
%
% Outputs:
%   - BHO_PID_Stochastic_Results.xlsx
%       Sheet "All_Runs"      : every independent run
%       Sheet "Summary"       : detailed statistics
%       Sheet "Paper_Summary" : compact Best/Mean/Std/Worst table
%   - BHO_PID_Stochastic_AllRuns.csv
%   - BHO_PID_Stochastic_Summary.csv
%   - BHO_PID_Stochastic_PaperSummary.csv
%   - BHO_PID_Stochastic_Checkpoint.mat
%
% Place this file in the ROOT project folder:
%   MATLAB Optimization Files\BHO_PID_Stochastic_Analysis.m
%% ========================================================================

%% =========================
% Repeated-run settings
%% =========================
nRuns = 10;
baseSeed = 2026;

%% =========================
% BHO settings
%% =========================
nPop = 100;
MaxIt = 100;
controller_type = 'PID';
optimization_mode = 1;  % Best Values Mode

%% =========================
% PID search bounds used for this stochastic study
%% =========================
% These values are intentionally defined here so the stochastic analysis
% does not depend on the current values inside main.m.
VarMin = [0 0 0];
VarMax = [1000 1000 1000];

%% =========================
% Compatibility arguments
%% =========================
% The current cost-function signatures require these arguments.
% They are not used when optimization_mode == 1 because every cost
% function returns from the Best Values Mode section before target-based
% calculations are reached.
wantedovershoot = 0.1;
wantedrisetime = 0.1;
wantedess = 0.01;

%% =========================
% Systems to analyze
% 1 = Ball and Beam
% 2 = Cruise Control
% 3 = DC Motor Speed
%% =========================
systemsToRun = [1 2 3];

%% =========================
% Project folders
%% =========================
mainFolder = fileparts(mfilename('fullpath'));

initFolder = fullfile(mainFolder, 'Initialization');
costFolder = fullfile(mainFolder, 'Costs Functions');
tuningFolder = fullfile(mainFolder, 'Tuning Methods');

requiredFolders = {initFolder, costFolder, tuningFolder};

for i = 1:numel(requiredFolders)
    if ~isfolder(requiredFolders{i})
        error('Required folder not found: %s', requiredFolders{i});
    end
end

addpath(genpath(initFolder));
addpath(genpath(costFolder));
addpath(genpath(tuningFolder));

%% =========================
% Verify required project functions
%% =========================
requiredFunctions = { ...
    'init_ballandbeam', ...
    'init_cruise', ...
    'init_motor', ...
    'cost_ballandbeam', ...
    'cost_cruise', ...
    'cost_motor', ...
    'BlackHoleAlgorithm'};

for i = 1:numel(requiredFunctions)
    if exist(requiredFunctions{i}, 'file') ~= 2
        error('Required MATLAB function not found on path: %s', requiredFunctions{i});
    end
end

%% =========================
% Output files
%% =========================
outputXlsx = fullfile(mainFolder, 'BHO_PID_Stochastic_Results.xlsx');
outputAllRunsCsv = fullfile(mainFolder, 'BHO_PID_Stochastic_AllRuns.csv');
outputSummaryCsv = fullfile(mainFolder, 'BHO_PID_Stochastic_Summary.csv');
outputPaperSummaryCsv = fullfile(mainFolder, 'BHO_PID_Stochastic_PaperSummary.csv');
checkpointFile = fullfile(mainFolder, 'BHO_PID_Stochastic_Checkpoint.mat');

%% =========================
% Storage
%% =========================
allRuns = table();

fprintf('\n============================================================\n');
fprintf('BHO PID STOCHASTIC ANALYSIS\n');
fprintf('Independent runs per system : %d\n', nRuns);
fprintf('Population size             : %d stars\n', nPop);
fprintf('Maximum iterations          : %d\n', MaxIt);
fprintf('Optimization mode           : Best Values Mode\n');
fprintf('PID bounds                  : Kp [%.0f, %.0f], Ki [%.0f, %.0f], Kd [%.0f, %.0f]\n', ...
    VarMin(1), VarMax(1), VarMin(2), VarMax(2), VarMin(3), VarMax(3));
fprintf('============================================================\n');

totalTimer = tic;

%% =========================
% Run each system
%% =========================
for system_id = systemsToRun

    [systemName, initFcn, costFcnHandle, sim_time] = ...
        getSystemConfiguration(system_id);

    fprintf('\n############################################################\n');
    fprintf('System          : %s\n', systemName);
    fprintf('Simulation time : %.2f s\n', sim_time);
    fprintf('Runs            : %d\n', nRuns);
    fprintf('Population      : %d\n', nPop);
    fprintf('Iterations      : %d\n', MaxIt);
    fprintf('############################################################\n');

    %% Initialize the existing plant model
    plant = initFcn();

    validatePlantStructure(plant, systemName);

    %% Assign the same useful variables used by the project to base workspace
    assignin('base', 'sim_time', sim_time);
    assignin('base', 'optimization_mode', optimization_mode);
    assignin('base', 'wantedovershoot', wantedovershoot);
    assignin('base', 'wantedrisetime', wantedrisetime);
    assignin('base', 'wantedess', wantedess);

    assignin('base', 'num', plant.num);
    assignin('base', 'den', plant.den);
    assignin('base', 'step_amp', plant.step_amp);
    assignin('base', 'ref', plant.step_amp);

    %% Use the existing system cost function directly
    costFcn = @(K) costFcnHandle( ...
        K, ...
        plant, ...
        sim_time, ...
        optimization_mode, ...
        wantedovershoot, ...
        wantedrisetime, ...
        wantedess);

    %% Independent BHO runs
    for runIdx = 1:nRuns

        seed = baseSeed + (system_id - 1) * 10000 + (runIdx - 1);
        rng(seed, 'twister');

        setappdata(0, 'BHO_Stop', false);

        fprintf('[%s] Run %d/%d | Seed = %d ... ', ...
            systemName, runIdx, nRuns, seed);

        runTimer = tic;
        runStatus = "OK";
        errorMessage = "";
        bestK = [NaN NaN NaN];
        bestCost = NaN;

        try
            % The existing BlackHoleAlgorithm prints a very large table at
            % every iteration. evalc suppresses that Command Window output
            % without modifying BlackHoleAlgorithm.m.
            evalc('[bestK, bhoHistory] = BlackHoleAlgorithm(costFcn, nPop, MaxIt, VarMin, VarMax, controller_type);');

            if isempty(bestK) || numel(bestK) ~= 3 || any(~isfinite(bestK))
                error('BHO returned an invalid PID gain vector.');
            end

            bestK = reshape(bestK, 1, 3);

            % IMPORTANT:
            % BestCost is evaluated by the EXISTING cost_*.m function.
            % The stochastic script does not recreate the cost formula.
            bestCost = costFcn(bestK);

            if ~isfinite(bestCost)
                error('The existing cost function returned a non-finite cost.');
            end

            % Calculate reporting metrics from the optimized response.
            % These metrics do not replace or modify the existing cost.
            metrics = evaluatePIDResponse(bestK, plant, sim_time);

        catch ME
            runStatus = "FAILED";
            errorMessage = string(ME.message);
            metrics = emptyMetrics();

            warning('Run failed for %s, run %d: %s', ...
                systemName, runIdx, ME.message);
        end

        runtime_s = toc(runTimer);

        newRow = table( ...
            string(systemName), ...
            runIdx, ...
            seed, ...
            string(runStatus), ...
            errorMessage, ...
            bestK(1), ...
            bestK(2), ...
            bestK(3), ...
            bestCost, ...
            metrics.IAE, ...
            metrics.IAE_norm, ...
            metrics.RiseTime, ...
            metrics.SettlingTime, ...
            metrics.Overshoot, ...
            metrics.ESS_abs, ...
            metrics.ESS_percent, ...
            metrics.PeakTime, ...
            runtime_s, ...
            'VariableNames', { ...
            'System', ...
            'Run', ...
            'Seed', ...
            'Status', ...
            'ErrorMessage', ...
            'Kp', ...
            'Ki', ...
            'Kd', ...
            'BestCost', ...
            'IAE', ...
            'IAE_norm', ...
            'RiseTime_s', ...
            'SettlingTime_s', ...
            'Overshoot_percent', ...
            'ESS_abs', ...
            'ESS_percent', ...
            'PeakTime_s', ...
            'Runtime_s'});

        allRuns = [allRuns; newRow]; %#ok<AGROW>

        if runStatus == "OK"
            fprintf('Cost = %.10g | K = [%.6f %.6f %.6f] | %.1f s\n', ...
                bestCost, bestK(1), bestK(2), bestK(3), runtime_s);
        else
            fprintf('FAILED | %.1f s\n', runtime_s);
        end

        %% Save checkpoint after every run
        save(checkpointFile, ...
            'allRuns', ...
            'nRuns', ...
            'nPop', ...
            'MaxIt', ...
            'VarMin', ...
            'VarMax', ...
            'baseSeed', ...
            'systemsToRun', ...
            'optimization_mode');

        writetable(allRuns, outputAllRunsCsv);
    end
end

%% =========================
% Build final summaries
%% =========================
summaryTable = buildDetailedSummaryTable(allRuns);
paperSummaryTable = buildPaperSummaryTable(allRuns);

%% =========================
% Display final results
%% =========================
fprintf('\n\n============================================================\n');
fprintf('PAPER-READY STOCHASTIC SUMMARY\n');
fprintf('Statistics are calculated across independent successful BHO runs.\n');
fprintf('============================================================\n');
disp(paperSummaryTable);

fprintf('\n============================================================\n');
fprintf('DETAILED STOCHASTIC SUMMARY\n');
fprintf('============================================================\n');
disp(summaryTable);

fprintf('\nTotal stochastic-analysis runtime: %.1f s\n', toc(totalTimer));

%% =========================
% Export results
%% =========================
if isfile(outputXlsx)
    delete(outputXlsx);
end

writetable(allRuns, outputXlsx, 'Sheet', 'All_Runs');
writetable(summaryTable, outputXlsx, 'Sheet', 'Summary');
writetable(paperSummaryTable, outputXlsx, 'Sheet', 'Paper_Summary');

writetable(allRuns, outputAllRunsCsv);
writetable(summaryTable, outputSummaryCsv);
writetable(paperSummaryTable, outputPaperSummaryCsv);

save(checkpointFile, ...
    'allRuns', ...
    'summaryTable', ...
    'paperSummaryTable', ...
    'nRuns', ...
    'nPop', ...
    'MaxIt', ...
    'VarMin', ...
    'VarMax', ...
    'baseSeed', ...
    'systemsToRun', ...
    'optimization_mode');

fprintf('\nFiles saved successfully:\n');
fprintf('  %s\n', outputXlsx);
fprintf('  %s\n', outputAllRunsCsv);
fprintf('  %s\n', outputSummaryCsv);
fprintf('  %s\n', outputPaperSummaryCsv);
fprintf('  %s\n', checkpointFile);

%% ========================================================================
% Local function: system configuration
%% ========================================================================
function [systemName, initFcn, costFcnHandle, sim_time] = ...
    getSystemConfiguration(system_id)

    switch system_id
        case 1
            systemName = 'Ball and Beam';
            initFcn = @init_ballandbeam;
            costFcnHandle = @cost_ballandbeam;
            sim_time = 20;

        case 2
            systemName = 'Cruise Control';
            initFcn = @init_cruise;
            costFcnHandle = @cost_cruise;
            sim_time = 20;

        case 3
            systemName = 'DC Motor Speed';
            initFcn = @init_motor;
            costFcnHandle = @cost_motor;
            sim_time = 2;

        otherwise
            error('Invalid system_id. Use 1, 2, or 3.');
    end
end

%% ========================================================================
% Local function: validate plant returned by existing init file
%% ========================================================================
function validatePlantStructure(plant, systemName)

    requiredFields = {'num', 'den', 'step_amp', 'G'};

    if ~isstruct(plant)
        error('%s initialization function must return a plant structure.', ...
            systemName);
    end

    for i = 1:numel(requiredFields)
        if ~isfield(plant, requiredFields{i})
            error('%s plant is missing required field: %s', ...
                systemName, requiredFields{i});
        end
    end
end

%% ========================================================================
% Local function: evaluate optimized PID response for reporting
%% ========================================================================
function metrics = evaluatePIDResponse(K, plant, sim_time)

    kp = K(1);
    ki = K(2);
    kd = K(3);

    C = pid(kp, ki, kd);
    closedLoopSys = feedback(C * plant.G, 1);

    t = linspace(0, sim_time, 1000);
    [y, t] = step(plant.step_amp * closedLoopSys, t);

    t = t(:);
    y = y(:);

    if isempty(t) || isempty(y) || numel(t) ~= numel(y)
        error('Optimized response is empty or has inconsistent dimensions.');
    end

    if any(~isfinite(t)) || any(~isfinite(y))
        error('Optimized response contains NaN or Inf values.');
    end

    ref = plant.step_amp;

    try
        info = stepinfo(y, t, ref, 'SettlingTimeThreshold', 0.02);
    catch
        info = stepinfo(y, t, ref);
    end

    errorSignal = abs(ref - y);
    IAE = trapz(t, errorSignal);

    if abs(ref) < 1e-12
        IAE_norm = IAE / max(sim_time, 1e-12);
        ESS_abs = abs(y(end));
        ESS_percent = ESS_abs * 100;
    else
        IAE_norm = IAE / (abs(ref) * sim_time);
        ESS_abs = abs(ref - y(end));
        ESS_percent = (ESS_abs / abs(ref)) * 100;
    end

    metrics = struct();
    metrics.IAE = IAE;
    metrics.IAE_norm = IAE_norm;
    metrics.RiseTime = info.RiseTime;
    metrics.SettlingTime = info.SettlingTime;
    metrics.Overshoot = info.Overshoot;
    metrics.ESS_abs = ESS_abs;
    metrics.ESS_percent = ESS_percent;
    metrics.PeakTime = info.PeakTime;
end

%% ========================================================================
% Local function: empty metrics for failed runs
%% ========================================================================
function metrics = emptyMetrics()

    metrics = struct();
    metrics.IAE = NaN;
    metrics.IAE_norm = NaN;
    metrics.RiseTime = NaN;
    metrics.SettlingTime = NaN;
    metrics.Overshoot = NaN;
    metrics.ESS_abs = NaN;
    metrics.ESS_percent = NaN;
    metrics.PeakTime = NaN;
end

%% ========================================================================
% Local function: detailed statistical summary
%% ========================================================================
function summaryTable = buildDetailedSummaryTable(allRuns)

    systems = unique(allRuns.System, 'stable');
    summaryTable = table();

    for i = 1:numel(systems)

        systemName = systems(i);

        rows = allRuns( ...
            allRuns.System == systemName & ...
            allRuns.Status == "OK", :);

        if isempty(rows)
            newSummary = table( ...
                systemName, ...
                0, ...
                NaN, NaN, NaN, NaN, ...
                NaN, NaN, NaN, ...
                NaN, NaN, NaN, NaN, NaN, NaN, ...
                NaN, NaN, ...
                NaN, NaN, ...
                NaN, NaN, ...
                NaN, NaN, ...
                NaN, NaN, ...
                'VariableNames', { ...
                'System', ...
                'SuccessfulRuns', ...
                'BestCost', ...
                'MeanCost', ...
                'StdCost', ...
                'WorstCost', ...
                'BestKp', ...
                'BestKi', ...
                'BestKd', ...
                'MeanKp', ...
                'StdKp', ...
                'MeanKi', ...
                'StdKi', ...
                'MeanKd', ...
                'StdKd', ...
                'MeanRiseTime_s', ...
                'StdRiseTime_s', ...
                'MeanSettlingTime_s', ...
                'StdSettlingTime_s', ...
                'MeanOvershoot_percent', ...
                'StdOvershoot_percent', ...
                'MeanESS_percent', ...
                'StdESS_percent', ...
                'MeanIAE_norm', ...
                'StdIAE_norm'});

            summaryTable = [summaryTable; newSummary]; %#ok<AGROW>
            continue;
        end

        [bestCost, idxBest] = min(rows.BestCost);
        bestRow = rows(idxBest, :);

        newSummary = table( ...
            systemName, ...
            height(rows), ...
            bestCost, ...
            mean(rows.BestCost, 'omitnan'), ...
            std(rows.BestCost, 'omitnan'), ...
            max(rows.BestCost), ...
            bestRow.Kp, ...
            bestRow.Ki, ...
            bestRow.Kd, ...
            mean(rows.Kp, 'omitnan'), ...
            std(rows.Kp, 'omitnan'), ...
            mean(rows.Ki, 'omitnan'), ...
            std(rows.Ki, 'omitnan'), ...
            mean(rows.Kd, 'omitnan'), ...
            std(rows.Kd, 'omitnan'), ...
            mean(rows.RiseTime_s, 'omitnan'), ...
            std(rows.RiseTime_s, 'omitnan'), ...
            mean(rows.SettlingTime_s, 'omitnan'), ...
            std(rows.SettlingTime_s, 'omitnan'), ...
            mean(rows.Overshoot_percent, 'omitnan'), ...
            std(rows.Overshoot_percent, 'omitnan'), ...
            mean(rows.ESS_percent, 'omitnan'), ...
            std(rows.ESS_percent, 'omitnan'), ...
            mean(rows.IAE_norm, 'omitnan'), ...
            std(rows.IAE_norm, 'omitnan'), ...
            'VariableNames', { ...
            'System', ...
            'SuccessfulRuns', ...
            'BestCost', ...
            'MeanCost', ...
            'StdCost', ...
            'WorstCost', ...
            'BestKp', ...
            'BestKi', ...
            'BestKd', ...
            'MeanKp', ...
            'StdKp', ...
            'MeanKi', ...
            'StdKi', ...
            'MeanKd', ...
            'StdKd', ...
            'MeanRiseTime_s', ...
            'StdRiseTime_s', ...
            'MeanSettlingTime_s', ...
            'StdSettlingTime_s', ...
            'MeanOvershoot_percent', ...
            'StdOvershoot_percent', ...
            'MeanESS_percent', ...
            'StdESS_percent', ...
            'MeanIAE_norm', ...
            'StdIAE_norm'});

        summaryTable = [summaryTable; newSummary]; %#ok<AGROW>
    end
end

%% ========================================================================
% Local function: compact table for the research paper
%% ========================================================================
function paperSummaryTable = buildPaperSummaryTable(allRuns)

    systems = unique(allRuns.System, 'stable');
    paperSummaryTable = table();

    for i = 1:numel(systems)

        systemName = systems(i);

        rows = allRuns( ...
            allRuns.System == systemName & ...
            allRuns.Status == "OK", :);

        if isempty(rows)
            newRow = table( ...
                systemName, ...
                0, ...
                NaN, ...
                NaN, ...
                NaN, ...
                NaN, ...
                'VariableNames', { ...
                'System', ...
                'Runs', ...
                'BestCost', ...
                'MeanCost', ...
                'StdCost', ...
                'WorstCost'});

        else
            newRow = table( ...
                systemName, ...
                height(rows), ...
                min(rows.BestCost), ...
                mean(rows.BestCost, 'omitnan'), ...
                std(rows.BestCost, 'omitnan'), ...
                max(rows.BestCost), ...
                'VariableNames', { ...
                'System', ...
                'Runs', ...
                'BestCost', ...
                'MeanCost', ...
                'StdCost', ...
                'WorstCost'});
        end

        paperSummaryTable = [paperSummaryTable; newRow]; %#ok<AGROW>
    end
end
