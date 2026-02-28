%% =====================================
% DC Motor Open-Loop Step Test -> Excel -> Identify G(s) (ROBUST)
% Board: Arduino Nano | COM5
% Encoder: D2 (A), D3 (B)
% Driver : ENA D9 (PWM), IN1 D8, IN2 D7
% Output: RPM vs time, then estimate optimized:
%   G(s) = K/(tau*s+1) * e^{-L s}
% STOP: close figure or press STOP
%% =====================================

clear; clc; close all;

%% ---- Settings ----
COM   = "COM5";
BOARD = "Nano3";

ENA   = "D9";
IN1   = "D8";
IN2   = "D7";

ENC_A = "D2";
ENC_B = "D3";

CPR_MOTOR_4X  = 64.0;
GEAR_RATIO    = 131.25;
RPM_AT_OUTPUT = true;

SAMPLE_MS = 50;
TsTarget  = SAMPLE_MS/1000;

PWM_MIN = 0;
PWM_MAX = 255;

% ===== Step Test Settings =====
CAPTURE_SEC = 20;          % total test duration
STEP_TIME   = 0.40;        % time to apply the step (sec) after start
PWM_STEP    = 255;         % step PWM magnitude (0->PWM_STEP)
forwardDir  = true;

% Filters (for RPM)
rpmAlpha = 0.25;

% Optional soft-start (before step)
SOFTSTART_PWM = 50;        % small pwm before step (reduces jerk)
SOFTSTART_SEC = STEP_TIME; % keep until step time

%% ---- Derived CPR ----
if RPM_AT_OUTPUT
    CPR_TOTAL = CPR_MOTOR_4X * GEAR_RATIO;
else
    CPR_TOTAL = CPR_MOTOR_4X;
end

%% ---- Connect Arduino ----
a = arduino(COM, BOARD, "Libraries", "rotaryEncoder");

configurePin(a, ENA, "PWM");
configurePin(a, IN1, "DigitalOutput");
configurePin(a, IN2, "DigitalOutput");

enc = rotaryEncoder(a, ENC_A, ENC_B, round(CPR_TOTAL));

% Motor OFF initially
applyMotorPWM(a, ENA, IN1, IN2, 0, forwardDir, PWM_MIN, PWM_MAX);

% Reset encoder baseline
lastCount = readCount(enc);

%% ---- Figure (Fullscreen + STOP) ----
fig = figure('Name','Open-Loop Step Test (RPM)','NumberTitle','off');
fig.WindowState = 'maximized';

setappdata(fig,'stopFlag',false);

uicontrol('Style','pushbutton',...
          'String','STOP',...
          'FontSize',14,...
          'FontWeight','bold',...
          'BackgroundColor',[1 0.2 0.2],...
          'Units','normalized',...
          'Position',[0.88 0.93 0.1 0.05],...
          'Callback',@(src,event)setappdata(fig,'stopFlag',true));

fig.CloseRequestFcn = @(src,event)setappdata(fig,'stopFlag',true);

ax = axes(fig); grid(ax,'on'); hold(ax,'on');
hRPM = plot(ax, nan, nan, 'LineWidth', 1.5);
xlabel(ax,'Time (s)'); ylabel(ax,'RPM');
title(ax, sprintf('Open-Loop Step Test | PWM step=%d at t=%.2fs', PWM_STEP, STEP_TIME));

%% ---- Pre-allocate logs ----
Ncap = ceil(CAPTURE_SEC / TsTarget) + 50;
tLog   = nan(Ncap,1);
rpmLog = nan(Ncap,1);
uLog   = nan(Ncap,1);

k = 0;
rpmFilt = 0.0;
firstSample = true;

%% ---- Run test ----
disp("Running open-loop step test...");
disp("Close window or press STOP to abort.");

t0 = tic;
tPrev = toc(t0);
nextTick = tPrev;
lastPlotUpdate = 0;

while isvalid(fig) && ~getappdata(fig,'stopFlag')

    t = toc(t0);

    if t >= CAPTURE_SEC
        break;
    end

    if t < nextTick
        pause(0.001);
        continue;
    end
    nextTick = nextTick + TsTarget;

    dt = t - tPrev;
    tPrev = t;
    if dt <= 0
        continue;
    end

    % Encoder delta -> RPM
    count = readCount(enc);
    delta = count - lastCount;
    lastCount = count;

    rpmRaw = (double(delta) * 60.0) / (double(CPR_TOTAL) * dt);
    rpmRawMag = abs(rpmRaw);

    rpmFilt = rpmAlpha*rpmRawMag + (1-rpmAlpha)*rpmFilt;
    if firstSample
        rpmFilt = rpmRawMag;
        firstSample = false;
    end

    % Input PWM: soft-start then step
    if t < SOFTSTART_SEC
        uPWM = SOFTSTART_PWM;
    else
        uPWM = PWM_STEP;
    end
    applyMotorPWM(a, ENA, IN1, IN2, uPWM, forwardDir, PWM_MIN, PWM_MAX);

    % Log
    k = k + 1;
    if k > numel(tLog)
        tLog   = [tLog;   nan(200,1)];
        rpmLog = [rpmLog; nan(200,1)];
        uLog   = [uLog;   nan(200,1)];
    end
    tLog(k)   = t;
    rpmLog(k) = rpmFilt;
    uLog(k)   = uPWM;

    % Plot (rate limited)
    if (t - lastPlotUpdate) >= 0.1
        lastPlotUpdate = t;
        set(hRPM,'XData',tLog(1:k),'YData',rpmLog(1:k));
        xlim(ax,[max(0,t-10) t+0.2]);
        ylim(ax,[0 max(50, max(rpmLog(max(1,k-200):k),[],'omitnan')*1.2)]);
        drawnow limitrate;
    end
end

%% ---- SAFE STOP ----
applyMotorPWM(a, ENA, IN1, IN2, 0, forwardDir, PWM_MIN, PWM_MAX);

if isvalid(fig)
    delete(fig);
end

% Trim
tLog   = tLog(1:k);
rpmLog = rpmLog(1:k);
uLog   = uLog(1:k);

%% ---- Save to Excel ----
T = table(tLog, rpmLog, uLog, 'VariableNames', {'time_s','rpm','pwm'});
fileName = "step_test_rpm_data.xlsx";
writetable(T, fileName);

disp("Stopped.");
disp("Saved Excel: " + fileName);

% Show table at end
openvar("T");
disp("First 10 rows:");
disp(T(1:min(10,height(T)), :));

%% =========================================================
% ROBUST First-Order + Dead-Time Identification (Optimized)
% Model: G(s) = K/(tau*s + 1) * e^{-L s}
%% =========================================================

% Shift time so step occurs at t = 0
t_id = tLog - STEP_TIME;
mask = t_id >= 0;
t_id = t_id(mask);
y_id = rpmLog(mask);

if numel(t_id) < 20
    warning("Not enough data after step to identify G(s). Increase CAPTURE_SEC.");
    return;
end

% Build uniform time vector
Ts = TsTarget;
t_end = t_id(end);
t_u = (0:Ts:t_end)';

% Resample measured RPM onto uniform grid
y_u = interp1(t_id, y_id, t_u, 'linear', 'extrap');

% Remove DC offset (baseline)
y_u = y_u - y_u(1);

% ===== Initial guesses =====
yss = mean(y_u(end-max(10,round(0.2*numel(y_u)))+1:end), "omitnan");
K0  = max(1e-6, yss / double(PWM_STEP));
tau0 = max(0.1, t_end/3);
L0  = 0.02;

% Optimize in log-domain (keeps parameters positive)
p0 = [log(K0); log(tau0); log(max(L0,1e-4))];

% Input is a step PWM_STEP after t=0
u_u = PWM_STEP * ones(size(t_u));

cost = @(p) firstOrderCost(p, t_u, u_u, y_u);

opts = optimset('Display','iter','MaxIter',250,'TolX',1e-7,'TolFun',1e-7);
pHat = fminsearch(cost, p0, opts);

% Decode parameters
K   = exp(pHat(1));
tau = exp(pHat(2));
L   = exp(pHat(3));

% Build transfer function
s = tf('s');
G = K / (tau*s + 1);
G.InputDelay = L;

disp("=====================================");
disp("Optimized First-Order Plant Transfer Function G(s):");
G
disp("Estimated parameters (optimized):");
fprintf("K   = %.6f (RPM/PWM)\n", K);
fprintf("tau = %.6f s\n", tau);
fprintf("L   = %.6f s (dead time)\n", L);
disp("=====================================");

% Simulate model
y_model = lsim(G, u_u, t_u);

% Compare
figure('Name','First-Order Identification (Optimized)','NumberTitle','off');
grid on; hold on;
plot(t_u, y_u, 'LineWidth', 1.5);
plot(t_u, y_model, '--', 'LineWidth', 1.5);
xlabel("Time after step (s)");
ylabel("RPM");
title("Measured vs Optimized First-Order Model (Open-loop)");
legend("Measured RPM (resampled)","Model RPM","Location","best");

%% ---- Helper ----
function J = firstOrderCost(p, t, u, y)
    % Decode positive parameters
    K   = exp(p(1));
    tau = exp(p(2));
    L   = exp(p(3));

    s = tf('s');
    G = K / (tau*s + 1);
    G.InputDelay = L;

    try
        yhat = lsim(G, u, t);
        e = y - yhat;

        % Weight early part more (better dynamics fit)
        w = 1 + 3*exp(-t/0.5);
        J = sum((w .* e).^2, 'omitnan');

        if ~isfinite(J)
            J = 1e30;
        end
    catch
        J = 1e30;
    end
end

function applyMotorPWM(a, ENA, IN1, IN2, pwm, forward, PWM_MIN, PWM_MAX)

    pwm = max(PWM_MIN, min(PWM_MAX, pwm));

    if pwm == 0
        writeDigitalPin(a, IN1, 0);
        writeDigitalPin(a, IN2, 0);
        writePWMDutyCycle(a, ENA, 0);
        return;
    end

    if forward
        writeDigitalPin(a, IN1, 1);
        writeDigitalPin(a, IN2, 0);
    else
        writeDigitalPin(a, IN1, 0);
        writeDigitalPin(a, IN2, 1);
    end

    writePWMDutyCycle(a, ENA, pwm/255);
end