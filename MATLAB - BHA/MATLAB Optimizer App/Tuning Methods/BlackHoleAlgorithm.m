function [best, history] = BlackHoleAlgorithm(costFcn, nPop, MaxIt, VarMin, VarMax, controller_type, model, stopFcn, progressFcn)

nVar = numel(VarMin);

if nargin < 8 || isempty(stopFcn)
    stopFcn = @() false;
end

if nargin < 9
    progressFcn = [];
end

% Read once instead of calling evalin repeatedly inside the loop
sim_time = evalin('base', 'sim_time');

if evalin('base', 'exist(''step_amp'', ''var'')')
    ref = evalin('base', 'step_amp');
elseif evalin('base', 'exist(''ref'', ''var'')')
    ref = evalin('base', 'ref');
else
    ref = 1;
end

% Initialize stars
star = struct('pos', cell(1, nPop), 'cost', cell(1, nPop));

for i = 1:nPop
    if stopFcn()
        best = [];
        history = struct( ...
            'Iteration', [], ...
            'Kp', [], ...
            'Ki', [], ...
            'Kd', [], ...
            'Cost', [], ...
            'Overshoot', [], ...
            'RiseTime', [], ...
            'ESS', []);
        return;
    end

    star(i).pos  = VarMin + rand(1, nVar) .* (VarMax - VarMin);
    star(i).cost = costFcn(star(i).pos);
end

% Preallocate history for speed
history = struct( ...
    'Iteration', zeros(MaxIt, 1), ...
    'Kp', zeros(MaxIt, 1), ...
    'Ki', zeros(MaxIt, 1), ...
    'Kd', zeros(MaxIt, 1), ...
    'Cost', zeros(MaxIt, 1), ...
    'Overshoot', NaN(MaxIt, 1), ...
    'RiseTime', NaN(MaxIt, 1), ...
    'ESS', NaN(MaxIt, 1));

% Cache last simulated best gains to avoid repeated simulation
lastSimulatedKfull = [NaN NaN NaN];
lastOvershootVal   = NaN;
lastRiseTimeVal    = NaN;
lastEssVal         = NaN;

bestStar = [];

for it = 1:MaxIt

    if stopFcn()
        best = [];
        history.Iteration = history.Iteration(1:it-1);
        history.Kp        = history.Kp(1:it-1);
        history.Ki        = history.Ki(1:it-1);
        history.Kd        = history.Kd(1:it-1);
        history.Cost      = history.Cost(1:it-1);
        history.Overshoot = history.Overshoot(1:it-1);
        history.RiseTime  = history.RiseTime(1:it-1);
        history.ESS       = history.ESS(1:it-1);
        return;
    end

    % Step 1: Select black hole
    allCosts = [star.cost];
    [~, idxBH] = min(allCosts);
    BH = star(idxBH);

    % Step 2: Move stars toward black hole
    movedStar = star;

    for i = 1:nPop
        if stopFcn()
            best = [];
            history.Iteration = history.Iteration(1:it-1);
            history.Kp        = history.Kp(1:it-1);
            history.Ki        = history.Ki(1:it-1);
            history.Kd        = history.Kd(1:it-1);
            history.Cost      = history.Cost(1:it-1);
            history.Overshoot = history.Overshoot(1:it-1);
            history.RiseTime  = history.RiseTime(1:it-1);
            history.ESS       = history.ESS(1:it-1);
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
    allNewCosts = [movedStar.cost];
    [~, idxBH_new] = min(allNewCosts);
    BH = movedStar(idxBH_new);

    % Step 4: Event horizon
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
            history.Iteration = history.Iteration(1:it-1);
            history.Kp        = history.Kp(1:it-1);
            history.Ki        = history.Ki(1:it-1);
            history.Kd        = history.Kd(1:it-1);
            history.Cost      = history.Cost(1:it-1);
            history.Overshoot = history.Overshoot(1:it-1);
            history.RiseTime  = history.RiseTime(1:it-1);
            history.ESS       = history.ESS(1:it-1);
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
    allCosts = [star.cost];
    [~, idxBest] = min(allCosts);
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

    % Re-simulate only if best gains changed
    if ~isequaln(Kfull, lastSimulatedKfull)

        assignin('base', 'kp', Kfull(1));
        assignin('base', 'ki', Kfull(2));
        assignin('base', 'kd', Kfull(3));

        overshootVal = NaN;
        riseTimeVal  = NaN;
        essVal       = NaN;

        try
            simOut = sim(model, 'StopTime', num2str(sim_time), 'CaptureErrors', 'on');

            if isprop(simOut, 'ErrorMessage') && ~isempty(simOut.ErrorMessage)
                error(simOut.ErrorMessage);
            end

            resp = simOut.OutputResponse;
            t = resp.Time;
            y = resp.Data;

            try
                info = stepinfo(y, t, ref, 'SettlingTimeThreshold', 0.02);
            catch
                info = stepinfo(y, t);
            end

            overshootVal = info.Overshoot;
            riseTimeVal  = info.RiseTime;

            if isempty(y) || abs(ref) < eps
                essVal = 0;
            else
                essVal = (abs(ref - y(end)) / abs(ref)) * 100;
            end

        catch
            overshootVal = NaN;
            riseTimeVal  = NaN;
            essVal       = NaN;
        end

        lastSimulatedKfull = Kfull;
        lastOvershootVal   = overshootVal;
        lastRiseTimeVal    = riseTimeVal;
        lastEssVal         = essVal;

    else
        overshootVal = lastOvershootVal;
        riseTimeVal  = lastRiseTimeVal;
        essVal       = lastEssVal;
    end

    % Save current best iteration result
    history.Iteration(it) = it;
    history.Kp(it)        = Kfull(1);
    history.Ki(it)        = Kfull(2);
    history.Kd(it)        = Kfull(3);
    history.Cost(it)      = bestStar.cost;
    history.Overshoot(it) = overshootVal;
    history.RiseTime(it)  = riseTimeVal;
    history.ESS(it)       = essVal;

    % Live UI update callback
    if ~isempty(progressFcn)
        currentHistory = struct( ...
            'Iteration', history.Iteration(1:it), ...
            'Kp', history.Kp(1:it), ...
            'Ki', history.Ki(1:it), ...
            'Kd', history.Kd(1:it), ...
            'Cost', history.Cost(1:it), ...
            'Overshoot', history.Overshoot(1:it), ...
            'RiseTime', history.RiseTime(1:it), ...
            'ESS', history.ESS(1:it));

        progressFcn(currentHistory);
    end
end

best = bestStar.pos;

% Trim history to actual iterations
history.Iteration = history.Iteration(1:MaxIt);
history.Kp        = history.Kp(1:MaxIt);
history.Ki        = history.Ki(1:MaxIt);
history.Kd        = history.Kd(1:MaxIt);
history.Cost      = history.Cost(1:MaxIt);
history.Overshoot = history.Overshoot(1:MaxIt);
history.RiseTime  = history.RiseTime(1:MaxIt);
history.ESS       = history.ESS(1:MaxIt);

end