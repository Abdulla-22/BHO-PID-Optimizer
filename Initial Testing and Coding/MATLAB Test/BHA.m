clc;
clear;

% SETTINGS
numStars = 10;
iterations = 100;

% Bounds
xmin = 1.5; xmax = 3.5;
ymin = 0.5; ymax = 7;

% STEP 1 — Generate Random Stars
x = xmin + (xmax - xmin) * rand(numStars,1);
y = ymin + (ymax - ymin) * rand(numStars,1);

% Fitness Function
f = x.^2 + y.^2;

for iter = 1:iterations
    
    fprintf('\n================ Iteration %d ================\n', iter);
    
    % STEP 2 — Find Black Hole
    [bestFitness, index] = min(f);
    xBH = x(index);
    yBH = y(index);
    
    fprintf('Black Hole -> x=%.4f  y=%.4f  f=%.4f\n', xBH, yBH, bestFitness);
    
    % STEP 3 — Move Stars Toward Black Hole
    for i = 1:numStars
        if i ~= index
            r = rand;
            x(i) = x(i) + (xBH - x(i)) * r;
            y(i) = y(i) + (yBH - y(i)) * r;
        end
    end
    
    % STEP 4 — Recalculate Fitness
    f = x.^2 + y.^2;
    
    % STEP 5 — Calculate Radius R
    R = bestFitness / sum(f);
    fprintf('Event Horizon Radius R = %.6f\n', R);
    
    % STEP 6 — Calculate Distances D
    D = zeros(numStars,1);
    
    for i = 1:numStars
        D(i) = sqrt((x(i)-xBH)^2 + (y(i)-yBH)^2);
        fprintf('Star %d -> D = %.6f\n', i, D(i));
        
        % Swallow condition
        if D(i) < R && i ~= index
            fprintf('Star %d swallowed and replaced\n', i);
            
            x(i) = xmin + (xmax-xmin)*rand;
            y(i) = ymin + (ymax-ymin)*rand;
        end
    end
    
    % Update fitness after possible replacement
    f = x.^2 + y.^2;
    
    % Display full table
    disp(table(x,y,f,D))
end

% Final Best Solution
[finalBest, index] = min(f);

fprintf('\nFinal Solution:\n');
fprintf('x = %.6f , y = %.6f , f = %.6f\n', x(index), y(index), finalBest);