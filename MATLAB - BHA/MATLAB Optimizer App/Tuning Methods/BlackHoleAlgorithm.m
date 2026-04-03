function [best, history] = BlackHoleAlgorithm(costFcn, nPop, MaxIt, VarMin, VarMax, controller_type, model, stopFcn, progressFcn)

nVar = numel(VarMin);

if nargin < 8 || isempty(stopFcn)
    stopFcn = @() false;
end

if nargin < 9
    progressFcn = [];
end

% Initialize stars
star = struct('pos', [], 'cost', []);

for i = 1:nPop
    if stopFcn()
        best = [];
        history = struct('Iteration', [], 'Kp', [], 'Ki', [], 'Kd', [], ...
                         'Cost', [], 'Overshoot', [], 'RiseTime', [], 'ESS', []);
        return;
    end

    star(i).pos  = VarMin + rand(1, nVar) .* (VarMax - VarMin);
    star(i).cost = costFcn(star(i).pos);
end

% History initialization
history = struct( ...
    'Iteration', [], ...
    'Kp', [], ...
    'Ki', [], ...
    'Kd', [], ...
    'Cost', [], ...
    'Overshoot', [], ...
    'RiseTime', [], ...
    'ESS', []);

for it = 1:MaxIt

    if stopFcn()
        best = [];
        return;
    end

    % Step 1: Select black hole
    [~, idxBH] = min([star.cost]);
    BH = star(idxBH);

    % Step 2: Move stars toward black hole
    movedStar = star;

    for i = 1:nPop
        if stopFcn()
            best = [];
            return;
        end

        if i ~= idxBH
            movedStar(i).pos = movedStar(i).pos + rand(1, nVar) .* (BH.pos - movedStar(i).pos);

            movedStar(i).pos = max(movedStar(i).pos, VarMin);
            movedStar(i).pos = min(movedStar(i).pos, VarMax);

            movedStar(i).cost = costFcn(movedStar(i).pos);
        end
    end

    % Step 3: Update black hole
    [~, idxBH_new] = min([movedStar.cost]);
    BH = movedStar(idxBH_new);

    % Step 4: Event horizon
    allNewCosts = [movedStar.cost];
    sumCosts = sum(allNewCosts);

    if sumCosts == 0
        R = 0;
    else
        R = BH.cost / sumCosts;
    end

    % Step 5: Reinitialize swallowed stars
    for i = 1:nPop
        if stopFcn()
            best = [];
            return;
        end

        if i ~= idxBH_new
            distVal = norm(BH.pos - movedStar(i).pos);

            if distVal < R
                movedStar(i).pos  = VarMin + rand(1, nVar) .* (VarMax - VarMin);
                movedStar(i).cost = costFcn(movedStar(i).pos);
            end
        end
    end

    % Step 6: Update population
    star = movedStar;

    % Step 7: Current best
    [~, idxBest] = min([star.cost]);
    bestStar = star(idxBest);

    % Expand gains to full PID form
    switch upper(controller_type)
        case 'PI'
            Kfull = [bestStar.pos(1) bestStar.pos(2) 0];
        case 'PD'
            Kfull = [bestStar.pos(1) 0 bestStar.pos(2)];
        case 'PID'
            Kfull = [bestStar.pos(1) bestStar.pos(2) bestStar.pos(3)];
        otherwise
            error('Invalid controller_type. Use PI, PD, or PID.');
    end

    % Simulate best solution of current iteration to extract OS / RT / ESS
    assignin('base', 'kp', Kfull(1));
    assignin('base', 'ki', Kfull(2));
    assignin('base', 'kd', Kfull(3));

    overshootVal = NaN;
    riseTimeVal  = NaN;
    essVal       = NaN;

    try
        sim_time = evalin('base', 'sim_time');
        simOut = sim(model, 'StopTime', num2str(sim_time), 'CaptureErrors', 'on');

        if isprop(simOut, 'ErrorMessage') && ~isempty(simOut.ErrorMessage)
            error(simOut.ErrorMessage);
        end

        resp = simOut.OutputResponse;
        t = resp.Time;
        y = resp.Data;

        if evalin('base', 'exist(''step_amp'', ''var'')')
            ref = evalin('base', 'step_amp');
        elseif evalin('base', 'exist(''ref'', ''var'')')
            ref = evalin('base', 'ref');
        else
            ref = 1;
        end

        try
            info = stepinfo(y, t, ref, 'SettlingTimeThreshold', 0.02);
        catch
            info = stepinfo(y, t);
        end

        overshootVal = info.Overshoot;
        riseTimeVal  = info.RiseTime;

        if abs(ref) < eps
            essVal = 0;
        else
            essVal = (abs(ref - y(end)) / abs(ref)) * 100;
        end

    catch
        overshootVal = NaN;
        riseTimeVal  = NaN;
        essVal       = NaN;
    end

    % Save current best iteration result
    history.Iteration(end + 1, 1) = it;
    history.Kp(end + 1, 1)        = Kfull(1);
    history.Ki(end + 1, 1)        = Kfull(2);
    history.Kd(end + 1, 1)        = Kfull(3);
    history.Cost(end + 1, 1)      = bestStar.cost;
    history.Overshoot(end + 1, 1) = overshootVal;
    history.RiseTime(end + 1, 1)  = riseTimeVal;
    history.ESS(end + 1, 1)       = essVal;

    % Live UI update callback
    if ~isempty(progressFcn)
        progressFcn(history);
    end
end

best = bestStar.pos;
end