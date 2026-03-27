function best = blackHole(costFcn, nPop, MaxIt, VarMin, VarMax)

nVar = numel(VarMin);

%% =========================
% Initialize stars
%% =========================
star = struct('pos', [], 'cost', []);

for i = 1:nPop
    star(i).pos  = VarMin + rand(1,nVar).*(VarMax - VarMin);
    star(i).cost = costFcn(star(i).pos);
end

fprintf('\n============================================================\n');
fprintf('         BLACK HOLE OPTIMIZATION - SLIDE STYLE\n');
fprintf('============================================================\n');

%% =========================
% Main loop
%% =========================
for it = 1:MaxIt
    
    fprintf('\n============================================================\n');
    fprintf('Iteration %d\n', it);
    fprintf('============================================================\n');
    
    %% 1) Sort OLD stars from largest cost to smallest cost
    oldCosts = [star.cost];
    [~, idx] = sort(oldCosts, 'descend');
    star = star(idx);
    
    % After descending sort, the LAST star has the minimum cost
    BH = star(end);
    
    %% Save initialization data
    oldPos  = zeros(nPop, nVar);
    oldCost = zeros(nPop, 1);
    
    for i = 1:nPop
        oldPos(i,:) = star(i).pos;
        oldCost(i)  = star(i).cost;
    end
    
    %% 2) Move stars toward the BH
    % BH stays fixed
    newPos  = zeros(nPop, nVar);
    newCost = zeros(nPop, 1);
    D       = zeros(nPop, 1);
    status  = strings(nPop, 1);
    
    for i = 1:nPop
        
        if i == nPop
            % This is the Black Hole (smallest cost), keep it fixed
            newPos(i,:) = star(i).pos;
            newCost(i)  = star(i).cost;
            status(i)   = "Black Hole";
        else
            % Move star toward BH using one scalar r
            r = rand;
            star(i).pos = star(i).pos + r * (BH.pos - star(i).pos);
            
            % Apply bounds
            star(i).pos = max(star(i).pos, VarMin);
            star(i).pos = min(star(i).pos, VarMax);
            
            % Recalculate cost
            star(i).cost = costFcn(star(i).pos);
            
            newPos(i,:) = star(i).pos;
            newCost(i)  = star(i).cost;
            status(i)   = "Moved toward BH";
        end
    end
    
    %% 3) Compute R from the fixed BH using NEW costs
    % BH is still the same star in this iteration
    BH_cost = newCost(nPop);
    R = BH_cost / sum(newCost);
    
    %% 4) Compute D for each star from BH
    for i = 1:nPop
        D(i) = norm(newPos(i,:) - BH.pos);
    end
    
    %% Same R for all stars
    Rcol = R * ones(nPop,1);
    
    %% 5) Print table after sorting
    if nVar == 3
        T = table( ...
            oldPos(:,1), oldPos(:,2), oldPos(:,3), ...
            oldCost, ...
            newPos(:,1), newPos(:,2), newPos(:,3), ...
            newCost, ...
            Rcol, ...
            D, ...
            status, ...
            'VariableNames', { ...
            'Old_Kp','Old_Ki','Old_Kd', ...
            'OldCost', ...
            'New_Kp','New_Ki','New_Kd', ...
            'NewCost', ...
            'R','D','Status'});
        
    elseif nVar == 2
        T = table( ...
            oldPos(:,1), oldPos(:,2), ...
            oldCost, ...
            newPos(:,1), newPos(:,2), ...
            newCost, ...
            Rcol, ...
            D, ...
            status, ...
            'VariableNames', { ...
            'Old_Kp','Old_Ki', ...
            'OldCost', ...
            'New_Kp','New_Ki', ...
            'NewCost', ...
            'R','D','Status'});
        
    else
        fprintf('\nNumber of variables not supported in table format.\n');
        for i = 1:nPop
            fprintf('Star %d | OldPos=%s | OldCost=%.6f | NewPos=%s | NewCost=%.6f | R=%.6f | D=%.6f | %s\n', ...
                i, mat2str(oldPos(i,:),4), oldCost(i), ...
                mat2str(newPos(i,:),4), newCost(i), ...
                R, D(i), status(i));
        end
        T = [];
    end
    
    if ~isempty(T)
        disp(T)
    end
    
    fprintf('Fixed BH Position = %s\n', mat2str(BH.pos, 5));
    fprintf('Fixed BH Cost     = %.6f\n', BH_cost);
    fprintf('R                 = %.6f\n', R);
    
    %% 6) Optional sucking check for next iteration
    % If you want to mark sucked stars:
    for i = 1:nPop-1
        if D(i) < R
            fprintf('Star %d satisfies D < R (sucked into BH)\n', i);
        end
    end
    
    %% 7) Prepare next iteration
    % Use updated stars population for the next loop
    for i = 1:nPop
        star(i).pos  = newPos(i,:);
        star(i).cost = newCost(i);
    end
end

%% Best solution after final iteration
finalCosts = [star.cost];
[~, idxBest] = min(finalCosts);
best = star(idxBest).pos;

fprintf('\n============================================================\n');
fprintf('Optimization Finished\n');
fprintf('Best Position = %s\n', mat2str(best, 6));
fprintf('Best Cost     = %.6f\n', min(finalCosts));
fprintf('============================================================\n');

end