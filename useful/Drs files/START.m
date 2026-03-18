%% PID Controller
clc
clear all
warning off
format long

Optimizers = {'RW', 'DE'}; % DO NOT TOUCH IT!

%% user-defined parameters
Algorithm = Optimizers{2}; % select either 1 or 2 >> RW takes long time because it searches through a while-loop till detecting feasible solutions 
MaxIter = 100; % number of iterations (for RW, try to make it small, like 5 or 10 -- for DE, try to make it large, like 200 or 500)
MaxOS = 5; % maximum accepted overshoot in % (for RW, try to make it large, like 30 or 50 to avoid long while loop processing -- for DE, you can easily set it to 5% or even less!)
Controller = 'PID'; % it could be 'P', 'PI', 'PD', or 'PID'

%% search-space (lower and upper limits)
Kp_min = -10; % minimum Kp
Kp_max = 50; % maximum Kp
Ki_min = -5; % minimum Ki
Ki_max = 30; % maximum Ki
Kd_min = -5; % maximum Kd
Kd_max = 30; % maximum Kd

%--------------------------------- NOTE ---------------------------------%
% If the random-walking (RW) algorithm takes very long time to finish:   %
% 1. try to minimize the search-space                                    %
% 2. try to increase MAxOS                                               %
%------------------------------------------------------------------------%

% parameters for the DE algorithm only
popSize = 20;   % Population size
F = 0.8;        % Mutation factor
CR = 0.9;       % Crossover probability
t = 0:0.1:500;  % simulation time and step-size resolution

s = tf('s');
A = 12; % amplitude for step input
B = 1; % amplitude for distrubrance

G = tf([0.05],[0.04 0.105 0.0525]); % process transfer function; P(s)
Delay = tf([1 -60 1200],[1 60 1200]); % time-delay via Pade approximation
H = tf(1, [1 1]); % measurement filter's transfer function; H(s)

if strcmp(Algorithm, 'RW') == 1
    Random_Walk(MaxIter, Kp_min, Kp_max, Ki_min, Ki_max, Kd_min, Kd_max, MaxOS, G, Delay, H, A, B, Controller, s);
elseif strcmp(Algorithm, 'DE') == 1
    Differential_Evolution(MaxIter, Kp_min, Kp_max, Ki_min, Ki_max, Kd_min, Kd_max, MaxOS, G, Delay, H, A, B, Controller, t, s, popSize, F, CR);
end
