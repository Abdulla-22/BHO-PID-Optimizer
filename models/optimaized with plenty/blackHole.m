function best = blackHole(costFcn, nPop, MaxIt, VarMin, VarMax)

nVar = numel(VarMin);

% Initialize stars
for i=1:nPop
    star(i).pos = VarMin + rand(1,nVar).*(VarMax-VarMin);
    star(i).cost = costFcn(star(i).pos);
end

% Find initial black hole
[~, idx] = min([star.cost]);
BH = star(idx);

%% Main loop
for it=1:MaxIt
    
    for i=1:nPop
        
        % Move stars toward BH
        star(i).pos = star(i).pos + rand(1,nVar).*(BH.pos - star(i).pos);
        
        % Bound
        star(i).pos = max(star(i).pos, VarMin);
        star(i).pos = min(star(i).pos, VarMax);
        
        % Evaluate
        star(i).cost = costFcn(star(i).pos);
        
        % Update BH
        if star(i).cost < BH.cost
            temp = BH;
            BH = star(i);
            star(i) = temp;
        end
        
    end
    
    % Event horizon
    costs = [star.cost];
    R = BH.cost / sum(costs);
    
    for i=1:nPop
        dist = norm(star(i).pos - BH.pos);
        
        if dist < R
            star(i).pos = VarMin + rand(1,nVar).*(VarMax-VarMin);
            star(i).cost = costFcn(star(i).pos);
        end
    end
    
    disp(['Iteration ' num2str(it) ' Best Cost = ' num2str(BH.cost)])
    
end

best = BH.pos;

end