function best = blackHole(costFcn, nPop, MaxIt, VarMin, VarMax)

nVar = numel(VarMin);

%% Initialize stars
star = struct('pos', [], 'cost', []);
for i = 1:nPop
    star(i).pos  = VarMin + rand(1,nVar).*(VarMax-VarMin);
    star(i).cost = costFcn(star(i).pos);
end

%% Main loop
for it = 1:MaxIt
    
    %% Fix Black Hole at start of iteration
    [~, idxBH] = min([star.cost]);
    BH = star(idxBH);   % fixed for this whole iteration
    
    % Save data for table
    oldPos  = zeros(nPop, nVar);
    oldCost = zeros(nPop, 1);
    newPos  = zeros(nPop, nVar);
    newCost = zeros(nPop, 1);
    distVal = zeros(nPop, 1);
    status  = strings(nPop, 1);
    
    for i = 1:nPop
        
        % Save old values
        oldPos(i,:) = star(i).pos;
        oldCost(i)  = star(i).cost;
        
        if i == idxBH
            % Keep Black Hole fixed
            newPos(i,:) = star(i).pos;
            newCost(i)  = star(i).cost;
            status(i)   = "Black Hole";
        else
            % Move stars toward fixed BH
            star(i).pos = star(i).pos + rand(1,nVar).*(BH.pos - star(i).pos);
            
            % Bound
            star(i).pos = max(star(i).pos, VarMin);
            star(i).pos = min(star(i).pos, VarMax);
            
            % Evaluate
            star(i).cost = costFcn(star(i).pos);
            
            % Save new values
            newPos(i,:) = star(i).pos;
            newCost(i)  = star(i).cost;
            status(i)   = "Moved toward BH";
        end
    end
    
    % Event horizon using fixed BH
    costs = [star.cost];
    R = BH.cost / sum(costs);
    
    for i = 1:nPop
        distVal(i) = norm(star(i).pos - BH.pos);
        
        if i ~= idxBH && distVal(i) < R
            star(i).pos = VarMin + rand(1,nVar).*(VarMax-VarMin);
            star(i).cost = costFcn(star(i).pos);
            
            newPos(i,:) = star(i).pos;
            newCost(i)  = star(i).cost;
            status(i)   = "Reinitialized";
            distVal(i)  = norm(star(i).pos - BH.pos);
        end
    end
    
    %% Sort for printing only: OldCost from largest to smallest
    [~, sortIdx] = sort(oldCost, 'descend');
    
    oldPos  = oldPos(sortIdx,:);
    oldCost = oldCost(sortIdx);
    newPos  = newPos(sortIdx,:);
    newCost = newCost(sortIdx);
    distVal = distVal(sortIdx);
    status  = status(sortIdx);
    
    %% Print table
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
            'Old_Kp','Old_Ki','Old_Kd', ...
            'OldCost', ...
            'New_Kp','New_Ki','New_Kd', ...
            'NewCost', ...
            'R','Distance','Status'});
        
        fprintf('\n================ Iteration %d ================\n', it);
        fprintf('Fixed BH Cost = %.6f\n', BH.cost);
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
            'Old_Kp','Old_Ki', ...
            'OldCost', ...
            'New_Kp','New_Ki', ...
            'NewCost', ...
            'R','Distance','Status'});
        
        fprintf('\n================ Iteration %d ================\n', it);
        fprintf('Fixed BH Cost = %.6f\n', BH.cost);
        disp(T)
    end
    
    fprintf('Iteration %d Best Cost = %.6f\n', it, BH.cost);
    
end

%% Final best
[~, idxBest] = min([star.cost]);
best = star(idxBest).pos;

end