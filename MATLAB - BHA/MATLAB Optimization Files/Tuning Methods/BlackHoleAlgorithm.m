function [best, bestHistory] = BlackHoleAlgorithm(costFcn, nPop, MaxIt, VarMin, VarMax, controller_type, model)

nVar = numel(VarMin);

%% Reset stop flag at start
setappdata(0, 'BHO_Stop', false);

%% Initialize stars
star = struct('pos', [], 'cost', []);

for i = 1:nPop

    if shouldStopOptimization()
        best = [];
        bestHistory = table();
        fprintf('\nOptimization stopped during initialization.\n');
        return;
    end

    star(i).pos  = VarMin + rand(1, nVar) .* (VarMax - VarMin);
    star(i).cost = costFcn(star(i).pos);
end

%% Initialize best history table
bestHistory = table();

%% Main loop
for it = 1:MaxIt

    if shouldStopOptimization()
        best = getCurrentBest(star);
        fprintf('\nOptimization stopped at iteration %d.\n', it);
        return;
    end

    %% Save old data
    oldPos  = zeros(nPop, nVar);
    oldCost = zeros(nPop, 1);

    for i = 1:nPop
        oldPos(i, :) = star(i).pos;
        oldCost(i)   = star(i).cost;
    end

    %% Step 1: Select black hole
    [~, idxBH] = min([star.cost]);
    BH = star(idxBH);

    %% Step 2: Move stars toward black hole
    movedStar = star;

    for i = 1:nPop

        if shouldStopOptimization()
            best = getCurrentBest(star);
            fprintf('\nOptimization stopped at iteration %d during movement.\n', it);
            return;
        end

        if i ~= idxBH
            movedStar(i).pos = movedStar(i).pos + rand(1, nVar) .* (BH.pos - movedStar(i).pos);

            movedStar(i).pos = max(movedStar(i).pos, VarMin);
            movedStar(i).pos = min(movedStar(i).pos, VarMax);

            movedStar(i).cost = costFcn(movedStar(i).pos);
        end
    end

    %% Step 3: Update black hole
    [~, idxBH_new] = min([movedStar.cost]);
    BH = movedStar(idxBH_new);

    %% Step 4: Event horizon
    allNewCosts = [movedStar.cost];
    sumCosts = sum(allNewCosts);

    if sumCosts == 0
        R = 0;
    else
        R = BH.cost / sumCosts;
    end

    %% Step 5: Reinitialize swallowed stars
    distVal = zeros(nPop, 1);
    status  = strings(nPop, 1);

    for i = 1:nPop

        if shouldStopOptimization()
            best = getCurrentBest(star);
            fprintf('\nOptimization stopped at iteration %d during event horizon check.\n', it);
            return;
        end

        if i == idxBH_new
            distVal(i) = 0;
            status(i)  = "Black Hole";
        else
            distVal(i) = norm(BH.pos - movedStar(i).pos);

            if distVal(i) < R
                movedStar(i).pos  = VarMin + rand(1, nVar) .* (VarMax - VarMin);
                movedStar(i).cost = costFcn(movedStar(i).pos);
                status(i) = "Reinitialized";
            else
                status(i) = "Moved toward BH";
            end
        end
    end

    %% Step 6: Update population
    star = movedStar;

    %% Current best
    [~, idxBest] = min([star.cost]);
    bestStar = star(idxBest);

    %% Evaluate current best solution with simulation metrics
    iterationMetrics = evaluateBestStarMetrics(bestStar, controller_type, model);

    %% Append one row to best history
    newRow = table( ...
        it, ...
        iterationMetrics.Kp, ...
        iterationMetrics.Ki, ...
        iterationMetrics.Kd, ...
        bestStar.cost, ...
        iterationMetrics.RiseTime, ...
        iterationMetrics.SettlingTime, ...
        iterationMetrics.Overshoot, ...
        iterationMetrics.PeakTime, ...
        iterationMetrics.ESS, ...
        iterationMetrics.Reference, ...
        iterationMetrics.FinalValue, ...
        'VariableNames', { ...
        'Iteration', ...
        'Kp', ...
        'Ki', ...
        'Kd', ...
        'Cost', ...
        'RiseTime', ...
        'SettlingTime', ...
        'Overshoot', ...
        'PeakTime', ...
        'ESS', ...
        'Reference', ...
        'FinalValue'});

    bestHistory = [bestHistory; newRow]; %#ok<AGROW>

    %% Prepare display arrays
    newPos  = zeros(nPop, nVar);
    newCost = zeros(nPop, 1);

    for i = 1:nPop
        newPos(i, :) = star(i).pos;
        newCost(i)   = star(i).cost;
    end

    %% Sort for display only
    [~, sortIdx] = sort(oldCost, 'descend');

    oldPos  = oldPos(sortIdx, :);
    oldCost = oldCost(sortIdx);
    newPos  = newPos(sortIdx, :);
    newCost = newCost(sortIdx);
    distVal = distVal(sortIdx);
    status  = status(sortIdx);

    %% Variable names
    switch upper(controller_type)
        case 'PI'
            varNamesOld = {'Old_Kp', 'Old_Ki'};
            varNamesNew = {'New_Kp', 'New_Ki'};

        case 'PD'
            varNamesOld = {'Old_Kp', 'Old_Kd'};
            varNamesNew = {'New_Kp', 'New_Kd'};

        case 'PID'
            varNamesOld = {'Old_Kp', 'Old_Ki', 'Old_Kd'};
            varNamesNew = {'New_Kp', 'New_Ki', 'New_Kd'};

        otherwise
            error('Invalid controller_type. Use PI, PD, or PID.');
    end

    %% Print table
    fprintf('\n================ Iteration %d ================\n', it);
    fprintf('Best Cost = %.6f\n', bestStar.cost);

    if nVar == 3
        T = table( ...
            (1:nPop)', ...
            oldPos(:,1), oldPos(:,2), oldPos(:,3), ...
            oldCost, ...
            newPos(:,1), newPos(:,2), newPos(:,3), ...
            newCost, ...
            repmat(R, nPop, 1), ...
            distVal, ...
            status, ...
            'VariableNames', { ...
            'Star', ...
            varNamesOld{1}, varNamesOld{2}, varNamesOld{3}, ...
            'OldCost', ...
            varNamesNew{1}, varNamesNew{2}, varNamesNew{3}, ...
            'NewCost', ...
            'R', 'Distance', 'Status'});

        printPlainTable(T);

    elseif nVar == 2
        T = table( ...
            (1:nPop)', ...
            oldPos(:,1), oldPos(:,2), ...
            oldCost, ...
            newPos(:,1), newPos(:,2), ...
            newCost, ...
            repmat(R, nPop, 1), ...
            distVal, ...
            status, ...
            'VariableNames', { ...
            'Star', ...
            varNamesOld{1}, varNamesOld{2}, ...
            'OldCost', ...
            varNamesNew{1}, varNamesNew{2}, ...
            'NewCost', ...
            'R', 'Distance', 'Status'});

        printPlainTable(T);
    end

    fprintf('Iteration %d Best Cost = %.6f\n', it, bestStar.cost);
    fprintf('Best Kp = %.6f | Ki = %.6f | Kd = %.6f\n', ...
        iterationMetrics.Kp, iterationMetrics.Ki, iterationMetrics.Kd);
    fprintf('Rise Time = %.6f | Settling Time = %.6f | Overshoot = %.6f | Peak Time = %.6f | ESS = %.6f\n', ...
        iterationMetrics.RiseTime, iterationMetrics.SettlingTime, ...
        iterationMetrics.Overshoot, iterationMetrics.PeakTime, iterationMetrics.ESS);
end

%% Final best
best = getCurrentBest(star);

end

%% =========================
% Local function: stop flag check
%% =========================
function stopFlag = shouldStopOptimization()

if isappdata(0, 'BHO_Stop')
    stopFlag = getappdata(0, 'BHO_Stop');
else
    stopFlag = false;
end

end

%% =========================
% Local function: get current best star
%% =========================
function best = getCurrentBest(star)

if isempty(star)
    best = [];
    return;
end

allCosts = [star.cost];
[~, idxBest] = min(allCosts);
best = star(idxBest).pos;

end

%% =========================
% Local function: evaluate best star metrics
%% =========================
function metrics = evaluateBestStarMetrics(bestStar, controller_type, model)

Kfull = expandControllerGainsLocal(bestStar.pos, controller_type);

assignin('base', 'kp', Kfull(1));
assignin('base', 'ki', Kfull(2));
assignin('base', 'kd', Kfull(3));

metrics.Kp = Kfull(1);
metrics.Ki = Kfull(2);
metrics.Kd = Kfull(3);
metrics.Reference = NaN;
metrics.FinalValue = NaN;
metrics.RiseTime = NaN;
metrics.SettlingTime = NaN;
metrics.Overshoot = NaN;
metrics.PeakTime = NaN;
metrics.ESS = NaN;

try
    if evalin('base', 'exist(''sim_time'',''var'')')
        sim_time = evalin('base', 'sim_time');
    else
        sim_time = 10;
    end

    simOut = sim(model, 'StopTime', num2str(sim_time), 'CaptureErrors', 'on');

    if ~isempty(simOut.ErrorMessage)
        return;
    end

    resp = simOut.OutputResponse;
    t = squeeze(resp.Time);
    y = squeeze(resp.Data);

    t = t(:);
    y = y(:);

    if isempty(t) || isempty(y) || numel(t) ~= numel(y)
        return;
    end

    if evalin('base', 'exist(''step_amp'',''var'')')
        ref = evalin('base', 'step_amp');
    elseif evalin('base', 'exist(''ref'',''var'')')
        ref = evalin('base', 'ref');
    else
        ref = 1;
    end

    metrics.Reference = ref;
    metrics.FinalValue = y(end);

    try
        info = stepinfo(y, t, ref, 'SettlingTimeThreshold', 0.02);
    catch
        info = stepinfo(y, t, ref);
    end

    metrics.RiseTime = info.RiseTime;
    metrics.SettlingTime = info.SettlingTime;
    metrics.Overshoot = info.Overshoot;
    metrics.PeakTime = info.PeakTime;

    actual_ess = abs(ref - y(end));
    if abs(ref) > 0
        metrics.ESS = (actual_ess / abs(ref)) * 100;
    else
        metrics.ESS = actual_ess;
    end

catch
end

end

%% =========================
% Local function: expand controller gains
%% =========================
function Kfull = expandControllerGainsLocal(Kopt, controller_type)

switch upper(controller_type)
    case 'PI'
        Kfull = [Kopt(1) Kopt(2) 0];

    case 'PD'
        Kfull = [Kopt(1) 0 Kopt(2)];

    case 'PID'
        Kfull = [Kopt(1) Kopt(2) Kopt(3)];

    otherwise
        error('Invalid controller_type. Use PI, PD, or PID.');
end

end

%% =========================
% Local function: print plain text table
%% =========================
function printPlainTable(T)

headers = T.Properties.VariableNames;
nCols = numel(headers);
nRows = height(T);

colWidths = zeros(1, nCols);
cellText = cell(nRows, nCols);

for j = 1:nCols
    colWidths(j) = length(headers{j});
end

for i = 1:nRows
    for j = 1:nCols
        value = T{i, j};

        if isstring(value) || ischar(value)
            txt = char(string(value));

        elseif isnumeric(value) || islogical(value)
            if isscalar(value)
                txt = sprintf('%.6g', value);
            else
                txt = mat2str(value);
            end

        elseif iscategorical(value)
            txt = char(string(value));

        else
            txt = char(string(value));
        end

        cellText{i, j} = txt;
        colWidths(j) = max(colWidths(j), length(txt));
    end
end

separator = '';
for j = 1:nCols
    separator = [separator, '+', repmat('-', 1, colWidths(j) + 2)]; %#ok<AGROW>
end
separator = [separator, '+'];

fprintf('%s\n', separator);

for j = 1:nCols
    fprintf('| %-*s ', colWidths(j), headers{j});
end
fprintf('|\n');

fprintf('%s\n', separator);

for i = 1:nRows
    for j = 1:nCols
        fprintf('| %-*s ', colWidths(j), cellText{i, j});
    end
    fprintf('|\n');
end

fprintf('%s\n', separator);

end