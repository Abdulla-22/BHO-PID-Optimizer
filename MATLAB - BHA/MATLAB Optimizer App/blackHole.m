function [bestPos, history] = blackHole(costFcn, nPop, MaxIt, VarMin, VarMax, controllerType, model, stopFcn)
% BLACKHOLE Black Hole Optimization with stop support
%
% Inputs:
%   costFcn         Cost function handle
%   nPop            Number of stars
%   MaxIt           Number of iterations
%   VarMin          Lower bounds
%   VarMax          Upper bounds
%   controllerType  PI / PD / PID
%   model           Simulink model name
%   stopFcn         Function handle that returns true when stop is requested
%
% Outputs:
%   bestPos         Best solution position
%   history         Struct containing optimization history

    if nargin < 8 || isempty(stopFcn)
        stopFcn = @() false;
    end

    nVar = numel(VarMin);

    emptyStar.pos = [];
    emptyStar.cost = [];
    star = repmat(emptyStar, nPop, 1);

    history.Iteration = [];
    history.Kp = [];
    history.Ki = [];
    history.Kd = [];
    history.Cost = [];
    history.Overshoot = [];
    history.RiseTime = [];

    % Initialize stars
    for i = 1:nPop
        checkStop(stopFcn);

        star(i).pos = VarMin + rand(1, nVar) .* (VarMax - VarMin);
        star(i).cost = costFcn(star(i).pos);
    end

    % Find initial black hole
    [~, idx] = min([star.cost]);
    BH = star(idx);

    % Main loop
    for it = 1:MaxIt
        checkStop(stopFcn);

        for i = 1:nPop
            checkStop(stopFcn);

            % Skip the current black hole itself
            if isequal(star(i).pos, BH.pos) && star(i).cost == BH.cost
                continue;
            end

            % Move stars toward black hole
            star(i).pos = star(i).pos + rand(1, nVar) .* (BH.pos - star(i).pos);

            % Keep inside bounds
            star(i).pos = max(star(i).pos, VarMin);
            star(i).pos = min(star(i).pos, VarMax);

            % Evaluate cost
            star(i).cost = costFcn(star(i).pos);

            % Replace black hole if a better star is found
            if star(i).cost < BH.cost
                temp = BH;
                BH = star(i);
                star(i) = temp;
            end
        end

        checkStop(stopFcn);

        % Event horizon
        totalCost = sum([star.cost]);

        if totalCost == 0
            R = 0;
        else
            R = BH.cost / totalCost;
        end

        for i = 1:nPop
            checkStop(stopFcn);

            if isequal(star(i).pos, BH.pos) && star(i).cost == BH.cost
                continue;
            end

            distanceToBH = norm(star(i).pos - BH.pos);

            if distanceToBH < R
                star(i).pos = VarMin + rand(1, nVar) .* (VarMax - VarMin);
                star(i).cost = costFcn(star(i).pos);

                if star(i).cost < BH.cost
                    temp = BH;
                    BH = star(i);
                    star(i) = temp;
                end
            end
        end

        % Save history
        Kfull = expandToFullGains(BH.pos, controllerType);

        history.Iteration(end+1,1) = it;
        history.Kp(end+1,1) = Kfull(1);
        history.Ki(end+1,1) = Kfull(2);
        history.Kd(end+1,1) = Kfull(3);
        history.Cost(end+1,1) = BH.cost;

        [osValue, rtValue] = evaluateResponseMetrics(model, Kfull, stopFcn);
        history.Overshoot(end+1,1) = osValue;
        history.RiseTime(end+1,1) = rtValue;

        fprintf('Iteration %d Best Cost = %.6f\n', it, BH.cost);
        drawnow;
    end

    bestPos = BH.pos;
end

function Kfull = expandToFullGains(Kopt, controllerType)
% Expand PI/PD/PID optimized vector to [Kp Ki Kd]

    switch upper(controllerType)
        case 'PI'
            Kfull = [Kopt(1) Kopt(2) 0];
        case 'PD'
            Kfull = [Kopt(1) 0 Kopt(2)];
        case 'PID'
            Kfull = [Kopt(1) Kopt(2) Kopt(3)];
        otherwise
            error('Invalid controller type.');
    end
end

function [overshootValue, riseTimeValue] = evaluateResponseMetrics(model, Kfull, stopFcn)
% Evaluate overshoot and rise time for the current best solution

    checkStop(stopFcn);

    overshootValue = NaN;
    riseTimeValue = NaN;

    try
        assignin('base', 'kp', Kfull(1));
        assignin('base', 'ki', Kfull(2));
        assignin('base', 'kd', Kfull(3));

        simOut = sim(model);

        checkStop(stopFcn);

        resp = simOut.OutputResponse;
        info = stepinfo(resp.Data, resp.Time);

        if isfield(info, 'Overshoot') && ~isempty(info.Overshoot) && ~isnan(info.Overshoot)
            overshootValue = info.Overshoot;
        end

        if isfield(info, 'RiseTime') && ~isempty(info.RiseTime) && ~isnan(info.RiseTime)
            riseTimeValue = info.RiseTime;
        end
    catch
        overshootValue = NaN;
        riseTimeValue = NaN;
    end
end

function checkStop(stopFcn)
% Throw stop error when user requests cancellation

    if stopFcn()
        error('BHA_App:StoppedByUser', 'Process stopped by user.');
    end
end