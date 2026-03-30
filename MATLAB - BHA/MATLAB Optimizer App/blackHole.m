function [best, history] = blackHole(costFcn, nPop, MaxIt, VarMin, VarMax, controller_type, model)

nVar = numel(VarMin);

%% Initialize stars
star = struct('pos', [], 'cost', []);
for i = 1:nPop
    star(i).pos  = VarMin + rand(1,nVar).*(VarMax - VarMin);
    star(i).cost = costFcn(star(i).pos);
end

%% Initialize history
history.Iteration  = zeros(MaxIt,1);
history.Kp         = zeros(MaxIt,1);
history.Ki         = zeros(MaxIt,1);
history.Kd         = zeros(MaxIt,1);
history.Cost       = zeros(MaxIt,1);
history.Overshoot  = zeros(MaxIt,1);
history.RiseTime   = zeros(MaxIt,1);

%% Main loop
for it = 1:MaxIt
    
    drawnow;
    pause(0.01);

    %% Select black hole
    [~, idxBH] = min([star.cost]);
    BH = star(idxBH);

    %% Move stars toward black hole
    movedStar = star;

    for i = 1:nPop
        if i ~= idxBH
            movedStar(i).pos = movedStar(i).pos + rand(1,nVar).*(BH.pos - movedStar(i).pos);

            movedStar(i).pos = max(movedStar(i).pos, VarMin);
            movedStar(i).pos = min(movedStar(i).pos, VarMax);

            movedStar(i).cost = costFcn(movedStar(i).pos);
        end
    end

    %% Update black hole using moved population
    [~, idxBH_new] = min([movedStar.cost]);
    BH = movedStar(idxBH_new);

    %% Event horizon
    allNewCosts = [movedStar.cost];
    R = BH.cost / sum(allNewCosts);

    %% Reinitialize swallowed stars
    for i = 1:nPop
        if i ~= idxBH_new
            distVal = norm(BH.pos - movedStar(i).pos);

            if distVal < R
                movedStar(i).pos  = VarMin + rand(1,nVar).*(VarMax - VarMin);
                movedStar(i).cost = costFcn(movedStar(i).pos);
            end
        end
    end

    %% Update population
    star = movedStar;

    %% Best star this iteration
    [~, idxBest] = min([star.cost]);
    bestStar = star(idxBest);

    %% Expand to full controller gains
    switch upper(controller_type)
        case 'PI'
            Kfull = [bestStar.pos(1) bestStar.pos(2) 0];

        case 'PD'
            Kfull = [bestStar.pos(1) 0 bestStar.pos(2)];

        case 'PID'
            Kfull = [bestStar.pos(1) bestStar.pos(2) bestStar.pos(3)];

        otherwise
            error('Invalid controller_type.');
    end

    %% Simulate current best gains
    assignin('base','kp',Kfull(1));
    assignin('base','ki',Kfull(2));
    assignin('base','kd',Kfull(3));

    try
        simOut = sim(model);
        resp = simOut.OutputResponse;
        info = stepinfo(resp.Data, resp.Time);

        history.Overshoot(it) = info.Overshoot;
        history.RiseTime(it)  = info.RiseTime;
    catch
        history.Overshoot(it) = NaN;
        history.RiseTime(it)  = NaN;
    end

    %% Save history
    history.Iteration(it) = it;
    history.Kp(it)        = Kfull(1);
    history.Ki(it)        = Kfull(2);
    history.Kd(it)        = Kfull(3);
    history.Cost(it)      = bestStar.cost;

    fprintf('Iteration %d Best Cost = %.6f\n', it, bestStar.cost);
end

%% Final best
[~, idxBest] = min([star.cost]);
best = star(idxBest).pos;

end