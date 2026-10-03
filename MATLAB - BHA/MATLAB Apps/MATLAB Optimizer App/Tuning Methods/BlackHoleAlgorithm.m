function [best, history] = BlackHoleAlgorithm(costFcn, nPop, MaxIt, VarMin, VarMax, controller_type, plant, sim_time, stopFcn, progressFcn, responseFcn)

nVar = numel(VarMin);

if nargin < 9 || isempty(stopFcn)
    stopFcn = @() false;
end

if nargin < 10
    progressFcn = [];
end

if nargin < 11 || isempty(responseFcn)
    error('responseFcn is required.');
end

ref = plant.step_amp;

star = struct('pos', cell(1, nPop), 'cost', cell(1, nPop));

for i = 1:nPop
    if stopFcn()
        best = [];
        history = emptyHistory();
        return;
    end

    star(i).pos  = VarMin + rand(1, nVar) .* (VarMax - VarMin);
    star(i).cost = costFcn(star(i).pos);
end

history = struct( ...
    'Iteration', zeros(MaxIt, 1), ...
    'Kp', zeros(MaxIt, 1), ...
    'Ki', zeros(MaxIt, 1), ...
    'Kd', zeros(MaxIt, 1), ...
    'Cost', zeros(MaxIt, 1), ...
    'Overshoot', NaN(MaxIt, 1), ...
    'RiseTime', NaN(MaxIt, 1), ...
    'ESS', NaN(MaxIt, 1));

bestStar = [];

for it = 1:MaxIt

    if stopFcn()
        best = [];
        history = trimHistory(history, it - 1);
        return;
    end

    allCosts = [star.cost];
    [~, idxBH] = min(allCosts);
    BH = star(idxBH);

    movedStar = star;

    for i = 1:nPop
        if stopFcn()
            best = [];
            history = trimHistory(history, it - 1);
            return;
        end

        if i ~= idxBH
            movedStar(i).pos = movedStar(i).pos + rand(1, nVar) .* (BH.pos - movedStar(i).pos);

            movedStar(i).pos = max(movedStar(i).pos, VarMin);
            movedStar(i).pos = min(movedStar(i).pos, VarMax);

            movedStar(i).cost = costFcn(movedStar(i).pos);
        end
    end

    allNewCosts = [movedStar.cost];
    [~, idxBH_new] = min(allNewCosts);
    BH = movedStar(idxBH_new);

    sumCosts = sum(allNewCosts);

    if sumCosts == 0
        R = 0;
    else
        R = BH.cost / sumCosts;
    end

    for i = 1:nPop
        if stopFcn()
            best = [];
            history = trimHistory(history, it - 1);
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

    star = movedStar;

    allCosts = [star.cost];
    [~, idxBest] = min(allCosts);
    bestStar = star(idxBest);

    Kfull = expandControllerGainsLocal(bestStar.pos, controller_type);

    overshootVal = NaN;
    riseTimeVal  = NaN;
    essVal       = NaN;

    try
        [t, y] = responseFcn(Kfull);

        if ~isempty(t) && ~isempty(y) && numel(t) == numel(y)
            try
                info = stepinfo(y, t, ref, 'SettlingTimeThreshold', 0.02);
            catch
                info = stepinfo(y, t, ref);
            end

            overshootVal = info.Overshoot;
            riseTimeVal  = info.RiseTime;

            if abs(ref) > eps
                essVal = (abs(ref - y(end)) / abs(ref)) * 100;
            else
                essVal = abs(ref - y(end));
            end
        end

    catch
        overshootVal = NaN;
        riseTimeVal  = NaN;
        essVal       = NaN;
    end

    history.Iteration(it) = it;
    history.Kp(it)        = Kfull(1);
    history.Ki(it)        = Kfull(2);
    history.Kd(it)        = Kfull(3);
    history.Cost(it)      = bestStar.cost;
    history.Overshoot(it) = overshootVal;
    history.RiseTime(it)  = riseTimeVal;
    history.ESS(it)       = essVal;

    if ~isempty(progressFcn)
        currentHistory = trimHistory(history, it);
        progressFcn(currentHistory);
    end
end

best = bestStar.pos;
history = trimHistory(history, MaxIt);

end

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

function history = emptyHistory()

history = struct( ...
    'Iteration', [], ...
    'Kp', [], ...
    'Ki', [], ...
    'Kd', [], ...
    'Cost', [], ...
    'Overshoot', [], ...
    'RiseTime', [], ...
    'ESS', []);

end

function history = trimHistory(history, n)

if n <= 0
    history = emptyHistory();
    return;
end

history.Iteration = history.Iteration(1:n);
history.Kp        = history.Kp(1:n);
history.Ki        = history.Ki(1:n);
history.Kd        = history.Kd(1:n);
history.Cost      = history.Cost(1:n);
history.Overshoot = history.Overshoot(1:n);
history.RiseTime  = history.RiseTime(1:n);
history.ESS       = history.ESS(1:n);

end