clc; clear; close all;

%% =========================
% Plant (your identified model)
% G(s) = 0.3572 * exp(-0.0886 s) / (0.8918 s + 1)
%% =========================
s = tf('s');
Kplant   = 0.2872;
tauPlant = 0.6542;
Ldelay   = 0.251;

G = Kplant/(tauPlant*s + 1);
G.InputDelay = Ldelay;

% Use Pade for delay during optimization (rational model)
PADE_ORDER = 3;
[numD, denD] = pade(Ldelay, PADE_ORDER);
Gp = (Kplant/(tauPlant*s + 1)) * tf(numD, denD);

%% =========================
% BH settings
%% =========================
numStars   = 100;
iterations = 200;

% PID bounds (tighten to avoid constant instability)
Kp_min = 0;   Kp_max = 15;
Ki_min = 0;   Ki_max = 10;
Kd_min = 0;   Kd_max = 0.3;

% Derivative filter (IMPORTANT)
N = 100;  % 20..200 typical. Higher = less filtering.

% Simulation time/grid
Ts   = 0.01;
Tend = 6;              % motor time constant ~0.9s, so 6s is enough
t = (0:Ts:Tend)';

% Reference (unit step)
r = ones(size(t));

% Fitness weights
W_ITAE = 1.0;
W_OS   = 1.0;
W_SSE  = 1.0;
W_U    = 0.002;     % control effort penalty

%% =========================
% Initialize stars: [Kp Ki Kd]
%% =========================
stars = zeros(numStars,3);
stars(:,1) = Kp_min + (Kp_max-Kp_min)*rand(numStars,1);
stars(:,2) = Ki_min + (Ki_max-Ki_min)*rand(numStars,1);
stars(:,3) = Kd_min + (Kd_max-Kd_min)*rand(numStars,1);

fitness = nan(numStars,1);
for i=1:numStars
    fitness(i) = pidFitness(stars(i,:), N, Gp, t, r, W_ITAE, W_OS, W_SSE, W_U);
end

%% =========================
% Black Hole Optimization
%% =========================
for iter = 1:iterations

    [bestFit, idxBH] = min(fitness);
    BH = stars(idxBH,:);

    fprintf('Iter %4d | Best = %.6g | Kp=%.4f Ki=%.4f Kd=%.4f\n',...
        iter, bestFit, BH(1), BH(2), BH(3));

    % Event horizon radius
    denom = sum(fitness(~isnan(fitness) & isfinite(fitness)));
    if ~isfinite(denom) || denom <= 0
        R = 0;
    else
        R = bestFit / denom;
    end

    for i = 1:numStars
        if i == idxBH, continue; end

        % Move toward black hole
        rmove = rand(1,3);
        stars(i,:) = stars(i,:) + (BH - stars(i,:)) .* rmove;

        % Clamp bounds
        stars(i,1) = min(max(stars(i,1),Kp_min),Kp_max);
        stars(i,2) = min(max(stars(i,2),Ki_min),Ki_max);
        stars(i,3) = min(max(stars(i,3),Kd_min),Kd_max);

        % Swallow
        D = norm(stars(i,:) - BH);
        if D < R
            stars(i,1) = Kp_min + (Kp_max-Kp_min)*rand;
            stars(i,2) = Ki_min + (Ki_max-Ki_min)*rand;
            stars(i,3) = Kd_min + (Kd_max-Kd_min)*rand;
        end
    end

    % Recompute fitness
    for i=1:numStars
        fitness(i) = pidFitness(stars(i,:), N, Gp, t, r, W_ITAE, W_OS, W_SSE, W_U);
    end
end

%% =========================
% Final best PID
%% =========================
[finalBest, idx] = min(fitness);
bestPID = stars(idx,:);
Kp = bestPID(1); Ki = bestPID(2); Kd = bestPID(3);

fprintf('\n===== FINAL BEST PID =====\n');
fprintf('Fitness = %.6g\n', finalBest);
fprintf('Kp = %.6f\nKi = %.6f\nKd = %.6f\nN  = %.1f\n', Kp, Ki, Kd, N);

%% =========================
% Validate on REAL plant with real delay (not pade)
%% =========================
Cbest = pid(Kp,Ki,Kd,N);
Tbest = feedback(Cbest*G, 1);
[ybest, tt] = step(Tbest, t);
info = stepinfo(ybest, tt);
SSE  = abs(1 - ybest(end));

figure('Name','Best PID on Real Delayed Plant','NumberTitle','off');
grid on; hold on;
plot(tt, ybest, 'LineWidth', 1.7);
yline(1,'--');
xlabel('Time (s)'); ylabel('Output');
title('BH-Optimized PID (Real Delay Plant)');

fprintf('\n===== METRICS (unit step) =====\n');
fprintf('Overshoot    = %.3f %%\n', info.Overshoot);
fprintf('SettlingTime = %.4f s\n', info.SettlingTime);
fprintf('RiseTime     = %.4f s\n', info.RiseTime);
fprintf('SSE (abs)    = %.6f\n', SSE);

%% ============================================================
% Fitness function (robust, with filtered derivative PID)
% ============================================================
function J = pidFitness(pidVec, N, Gp, t, r, W_ITAE, W_OS, W_SSE, W_U)

    Kp = pidVec(1); Ki = pidVec(2); Kd = pidVec(3);

    % Filtered-derivative PID (PROPER)
    C = pid(Kp, Ki, Kd, N);

    try
        Tcl = feedback(C*Gp, 1);

        % Stability check
        if ~isstable(Tcl)
            J = 1e12;
            return;
        end

        y = lsim(Tcl, r, t);
        if any(~isfinite(y))
            J = 1e12;
            return;
        end

        e = r - y;

        % ITAE
        ITAE = trapz(t, t .* abs(e));

        % Overshoot (absolute above steady-state)
        yss = y(end);
        OS  = max(0, max(y) - yss);

        % SSE
        SSE = abs(r(end) - yss);

        % Control effort: u = C/(1+C*Gp) * r
        Ucl = feedback(C, Gp);
        u   = lsim(Ucl, r, t);
        if any(~isfinite(u))
            J = 1e12;
            return;
        end
        Ueff = trapz(t, u.^2);

        J = W_ITAE*ITAE + W_OS*OS + W_SSE*SSE + W_U*Ueff;

        if ~isfinite(J)
            J = 1e12;
        end
    catch
        J = 1e12;
    end
end