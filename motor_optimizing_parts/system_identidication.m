%% =====================================
% DC Motor Open-Loop Step Test -> Excel -> Identify G(s) (ROBUST)
% BTS7960 version
% NO soft-start, NO slew rate: motor gets PWM instantly at step time.
% Board: Arduino Nano | COM5
% Encoder: D2 (A), D3 (B)
% Driver : BTS7960 -> R_PWM D9, L_PWM D10, R_EN D8, L_EN D7
% Output: RPM vs time, then estimate optimized:
%   G(s) = K/(tau*s+1) * e^{-L s}
% STOP: close figure or press STOP
%% =====================================

clear; clc; close all;

%% ---- Settings ----
COM   = "COM5";
BOARD = "Nano3";

% ===== BTS7960 Pins =====
R_PWM = "D9";
L_PWM = "D10";
R_EN  = "D8";
L_EN  = "D7";

% Optional current sense
R_IS  = "A0";
L_IS  = "A1";

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
CAPTURE_SEC = 20;      % total test duration
STEP_TIME   = 1.00;    % time to apply the step (sec) after start
PWM_STEP    = 170;     % step PWM magnitude (0->PWM_STEP)
forwardDir  = true;

% RPM low-pass filter
rpmAlpha = 0.25;

%% ---- Derived CPR ----
if RPM_AT_OUTPUT
    CPR_TOTAL = CPR_MOTOR_4X * GEAR_RATIO;
else
    CPR_TOTAL = CPR_MOTOR_4X;
end

%% ---- Connect Arduino ----
a = arduino(COM, BOARD, "Libraries", "rotaryEncoder");

% BTS7960 configuration
configurePin(a, R_PWM, "PWM");
configurePin(a, L_PWM, "PWM");
configurePin(a, R_EN , "DigitalOutput");
configurePin(a, L_EN , "DigitalOutput");

% Optional analog current sense
configurePin(a, R_IS, "AnalogInput");
configurePin(a, L_IS, "AnalogInput");

enc = rotaryEncoder(a, ENC_A, ENC_B, round(CPR_TOTAL));

% Enable BTS7960
writeDigitalPin(a, R_EN, 1);
writeDigitalPin(a, L_EN, 1);

% Motor OFF initially
applyMotorPWM_BTS7960(a, R_PWM, L_PWM, R_EN, L_EN, 0, forwardDir, PWM_MIN, PWM_MAX);

% Reset encoder baseline
lastCount = readCount(enc);

%% ---- Figure (Fullscreen + STOP) ----
fig = figure('Name','Open-Loop Step Test (RPM) - BTS7960','NumberTitle','off');
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

ax1 = subplot(2,1,1,'Parent',fig); grid(ax1,'on'); hold(ax1,'on');
hRPM = plot(ax1, nan, nan, 'LineWidth', 1.5);
xlabel(ax1,'Time (s)'); ylabel(ax1,'RPM');
title(ax1, sprintf('Open-Loop Step Test | PWM step=%d at t=%.2fs', PWM_STEP, STEP_TIME));

ax2 = subplot(2,1,2,'Parent',fig); grid(ax2,'on'); hold(ax2,'on');
hU = stairs(ax2, nan, nan, 'LineWidth', 1.5);
xlabel(ax2,'Time (s)'); ylabel(ax2,'PWM');
title(ax2,'Input PWM');

%% ---- Pre-allocate logs ----
Ncap = ceil(CAPTURE_SEC / TsTarget) + 200;
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

    % No abs(). Clamp negatives due to noise/reverse ticks.
    rpmRaw = max(0, rpmRaw);

    % Filter
    rpmFilt = rpmAlpha*rpmRaw + (1-rpmAlpha)*rpmFilt;
    if firstSample
        rpmFilt = rpmRaw;
        firstSample = false;
    end

    % Instant step: 0 -> PWM_STEP
    if t < STEP_TIME
        uPWM = 0;
    else
        uPWM = PWM_STEP;
    end
    applyMotorPWM_BTS7960(a, R_PWM, L_PWM, R_EN, L_EN, uPWM, forwardDir, PWM_MIN, PWM_MAX);

    % Log
    k = k + 1;
    if k > numel(tLog)
        tLog   = [tLog;   nan(400,1)];
        rpmLog = [rpmLog; nan(400,1)];
        uLog   = [uLog;   nan(400,1)];
    end
    tLog(k)   = t;
    rpmLog(k) = rpmFilt;
    uLog(k)   = uPWM;

    % Plot (rate limited)
    if (t - lastPlotUpdate) >= 0.1
        lastPlotUpdate = t;

        if isgraphics(hRPM)
            set(hRPM,'XData',tLog(1:k),'YData',rpmLog(1:k));
        end
        if isgraphics(hU)
            set(hU,'XData',tLog(1:k),'YData',uLog(1:k));
        end

        xlim(ax1,[max(0,t-10) t+0.2]);
        xlim(ax2,[max(0,t-10) t+0.2]);

        ymax = max(rpmLog(max(1,k-300):k),[],'omitnan');
        ylim(ax1,[0 max(50, ymax*1.2)]);
        ylim(ax2,[-5 260]);

        drawnow limitrate;
    end
end

%% ---- SAFE STOP ----
applyMotorPWM_BTS7960(a, R_PWM, L_PWM, R_EN, L_EN, 0, forwardDir, PWM_MIN, PWM_MAX);

if isvalid(fig)
    delete(fig);
end

% Trim logs
tLog   = tLog(1:k);
rpmLog = rpmLog(1:k);
uLog   = uLog(1:k);

%% ---- Save to Excel ----
T = table(tLog, rpmLog, uLog, 'VariableNames', {'time_s','rpm','pwm'});
fileName = "step_test_rpm_data.xlsx";
writetable(T, fileName);

disp("Stopped.");
disp("Saved Excel: " + fileName);

%% =========================================================
% First-Order + Dead-Time Identification (Optimized)
% Model: G(s) = K/(tau*s + 1) * e^{-L s}
%% =========================================================

% Find actual step time from uLog (robust)
idxStep = find(uLog >= PWM_STEP, 1, 'first');
if isempty(idxStep)
    warning("PWM step was never applied. Check STEP_TIME / PWM_STEP / logging.");
    return;
end
tStepActual = tLog(idxStep);

% Use data after actual step
t_id = tLog - tStepActual;
mask = t_id >= 0;
t_id = t_id(mask);
y_id = rpmLog(mask);

MIN_AFTER_STEP_SEC = 5;
if isempty(t_id) || t_id(end) < MIN_AFTER_STEP_SEC
    warning("Not enough time after step (need >= %.1fs). Increase CAPTURE_SEC or reduce STEP_TIME.", MIN_AFTER_STEP_SEC);
    fprintf("Captured after step: %.3f s | STEP at t=%.3f s | CAPTURE=%.3f s\n", ...
        (isempty(t_id)*0 + (~isempty(t_id))*t_id(end)), tStepActual, CAPTURE_SEC);
    return;
end

% Uniform time base
Ts = TsTarget;
t_end = t_id(end);
t_u = (0:Ts:t_end)';

% Resample
y_u = interp1(t_id, y_id, t_u, 'linear');
y_u = fillmissing(y_u,'nearest');

% Baseline removal
y_u = y_u - y_u(1);

% Input step
u_u = PWM_STEP * ones(size(t_u));

% Initial guesses
tailN = max(10, round(0.2*numel(y_u)));
yss = mean(y_u(end-tailN+1:end), "omitnan");
K0  = max(1e-6, yss / double(PWM_STEP));
tau0 = max(0.1, t_end/3);
L0  = 0.02;

p0 = [log(K0); log(tau0); log(max(L0,1e-4))];
cost = @(p) firstOrderCost(p, t_u, u_u, y_u);

opts = optimset('Display','iter','MaxIter',250,'TolX',1e-7,'TolFun',1e-7);
pHat = fminsearch(cost, p0, opts);

K   = exp(pHat(1));
tau = exp(pHat(2));
L   = exp(pHat(3));

% Transfer function
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
fprintf("Step detected at t = %.6f s (from PWM log)\n", tStepActual);
disp("=====================================");

% Model simulation
y_model = lsim(G, u_u, t_u);

% Compare plot
figure('Name','First-Order Identification (Optimized)','NumberTitle','off');
grid on; hold on;
plot(t_u, y_u, 'LineWidth', 1.5);
plot(t_u, y_model, '--', 'LineWidth', 1.5);
xlabel("Time after step (s)");
ylabel("RPM (baseline removed)");
title("Measured vs Optimized First-Order Model (Open-loop)");
legend("Measured RPM","Model RPM","Location","best");

%% ===================== Helpers =====================

function J = firstOrderCost(p, t, u, y)
    K   = exp(p(1));
    tau = exp(p(2));
    L   = exp(p(3));

    s = tf('s');
    G = K / (tau*s + 1);
    G.InputDelay = L;

    try
        yhat = lsim(G, u, t);
        e = y - yhat;

        % Weight early dynamics more
        w = 1 + 3*exp(-t/0.5);
        J = sum((w .* e).^2, 'omitnan');

        if ~isfinite(J)
            J = 1e30;
        end
    catch
        J = 1e30;
    end
end

function applyMotorPWM_BTS7960(a, R_PWM, L_PWM, R_EN, L_EN, pwm, forward, PWM_MIN, PWM_MAX)

    pwm = max(PWM_MIN, min(PWM_MAX, pwm));
    duty = pwm / 255;

    % Keep bridge enabled
    writeDigitalPin(a, R_EN, 1);
    writeDigitalPin(a, L_EN, 1);

    if pwm == 0
        writePWMDutyCycle(a, R_PWM, 0);
        writePWMDutyCycle(a, L_PWM, 0);
        return;
    end

    if forward
        writePWMDutyCycle(a, R_PWM, duty);
        writePWMDutyCycle(a, L_PWM, 0);
    else
        writePWMDutyCycle(a, R_PWM, 0);
        writePWMDutyCycle(a, L_PWM, duty);
    end
end