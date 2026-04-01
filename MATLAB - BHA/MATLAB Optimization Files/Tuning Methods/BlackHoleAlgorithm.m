function best = BlackHoleAlgorithm(costFcn, nPop, MaxIt, VarMin, VarMax, controller_type)

nVar = numel(VarMin);

%% Initialize stars
star = struct('pos', [], 'cost', []);
for i = 1:nPop
    star(i).pos  = VarMin + rand(1,nVar).*(VarMax - VarMin);
    star(i).cost = costFcn(star(i).pos);
end

%% Main loop
for it = 1:MaxIt

    if mod(it,5) == 0
        drawnow;
    end

    %% Save old data
    oldPos  = zeros(nPop, nVar);
    oldCost = zeros(nPop, 1);
    for i = 1:nPop
        oldPos(i,:) = star(i).pos;
        oldCost(i)  = star(i).cost;
    end

    %% Step 1: Select black hole from current population
    [~, idxBH] = min([star.cost]);
    BH = star(idxBH);

    %% Step 2: Move stars toward black hole
    movedStar = star;   % this will represent iteration t+1 stars before event horizon

    for i = 1:nPop
        if i ~= idxBH
            movedStar(i).pos = movedStar(i).pos + rand(1,nVar).*(BH.pos - movedStar(i).pos);

            % Bound
            movedStar(i).pos = max(movedStar(i).pos, VarMin);
            movedStar(i).pos = min(movedStar(i).pos, VarMax);

            % New cost after movement
            movedStar(i).cost = costFcn(movedStar(i).pos);
        end
    end

    %% Step 3: Update black hole using NEW stars
    [~, idxBH_new] = min([movedStar.cost]);
    BH = movedStar(idxBH_new);

    %% Step 4: Compute R using NEW stars
    allNewCosts = [movedStar.cost];
    R = BH.cost / sum(allNewCosts);

    %% Step 5: Compute distance using NEW stars
    distVal = zeros(nPop,1);
    status  = strings(nPop,1);

    for i = 1:nPop
        if i == idxBH_new
            distVal(i) = 0;
            status(i)  = "Black Hole";
        else
            distVal(i) = norm(BH.pos - movedStar(i).pos);

            if distVal(i) < R
                % Star dies and a completely new one is generated
                movedStar(i).pos  = VarMin + rand(1,nVar).*(VarMax - VarMin);
                movedStar(i).cost = costFcn(movedStar(i).pos);
                status(i) = "Reinitialized";
            else
                status(i) = "Moved toward BH";
            end
        end
    end

    %% Step 6: The new population for next iteration
    star = movedStar;

    %% Final best after reinitialization (for reporting only)
    [~, idxBest] = min([star.cost]);
    bestStar = star(idxBest);

    %% Prepare display arrays
    newPos  = zeros(nPop, nVar);
    newCost = zeros(nPop, 1);

    for i = 1:nPop
        newPos(i,:) = star(i).pos;
        newCost(i)  = star(i).cost;
    end

    %% Sort for display only
    [~, sortIdx] = sort(oldCost, 'descend');

    oldPos  = oldPos(sortIdx,:);
    oldCost = oldCost(sortIdx);
    newPos  = newPos(sortIdx,:);
    newCost = newCost(sortIdx);
    distVal = distVal(sortIdx);
    status  = status(sortIdx);

    %% Variable names based on controller type
    switch upper(controller_type)
        case 'PI'
            varNamesOld = {'Old_Kp','Old_Ki'};
            varNamesNew = {'New_Kp','New_Ki'};

        case 'PD'
            varNamesOld = {'Old_Kp','Old_Kd'};
            varNamesNew = {'New_Kp','New_Kd'};

        case 'PID'
            varNamesOld = {'Old_Kp','Old_Ki','Old_Kd'};
            varNamesNew = {'New_Kp','New_Ki','New_Kd'};

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
            repmat(R,nPop,1), ...
            distVal, ...
            status, ...
            'VariableNames', { ...
            'Star', ...
            varNamesOld{1}, varNamesOld{2}, varNamesOld{3}, ...
            'OldCost', ...
            varNamesNew{1}, varNamesNew{2}, varNamesNew{3}, ...
            'NewCost', ...
            'R','Distance','Status'});
        disp(T)

    elseif nVar == 2
        T = table( ...
            (1:nPop)', ...
            oldPos(:,1), oldPos(:,2), ...
            oldCost, ...
            newPos(:,1), newPos(:,2), ...
            newCost, ...
            repmat(R,nPop,1), ...
            distVal, ...
            status, ...
            'VariableNames', { ...
            'Star', ...
            varNamesOld{1}, varNamesOld{2}, ...
            'OldCost', ...
            varNamesNew{1}, varNamesNew{2}, ...
            'NewCost', ...
            'R','Distance','Status'});
        disp(T)
    end

    fprintf('Iteration %d Best Cost = %.6f\n', it, bestStar.cost);
end

%% Final best
[~, idxBest] = min([star.cost]);
best = star(idxBest).pos;

end