clc;
clear;
close all;

%% =========================
% Hardware
%% =========================
COM   = "COM5";
BOARD = "Nano3";

R_PWM = 'D9';
L_PWM = 'D10';
R_EN  = 'D8';
L_EN  = 'D7';

ENC_A = 'D2';
ENC_B = 'D3';

CPR_TOTAL = 64 * 131.25;

%% =========================
% Fixed PID values
%% =========================
kp = 8.465543539;
ki = 4.902590596;
kd = 0.032982879;

%% =========================
% Test settings
%% =========================
targetRPM = 50;
testTime  = 5;
Ts        = 0.05;

pauseTime = 1.5;

RPM_SAFETY_LIMIT = targetRPM * 2.5;

%% =========================
% Connect Arduino
%% =========================
fprintf("Connecting to Arduino on %s...\n", COM);

a = arduino(COM, BOARD, "Libraries", "rotaryEncoder");
enc = rotaryEncoder(a, ENC_A, ENC_B, round(CPR_TOTAL));

configurePin(a, R_PWM, "PWM");
configurePin(a, L_PWM, "PWM");
configurePin(a, R_EN, "DigitalOutput");
configurePin(a, L_EN, "DigitalOutput");

stopMotor(a, R_PWM, L_PWM, R_EN, L_EN);

%% =========================
% Create real-time figure
%% =========================
fig = figure("Name", "Real-Time PID Alpha Tuning", "NumberTitle", "off");

ax1 = subplot(2,1,1);
hold(ax1, "on");
grid(ax1, "on");
rpmLine = plot(ax1, nan, nan, "LineWidth", 2);
targetLine = plot(ax1, nan, nan, "--", "LineWidth", 2);
xlabel(ax1, "Time [s]");
ylabel(ax1, "RPM");
title(ax1, "Motor Response");

ax2 = subplot(2,1,2);
hold(ax2, "on");
grid(ax2, "on");
pwmLine = plot(ax2, nan, nan, "LineWidth", 2);
xlabel(ax2, "Time [s]");
ylabel(ax2, "PWM");
title(ax2, "PWM Signal");

%% =========================
% Stage 1: Coarse search
%% =========================
rpmList = [0.08 0.10 0.15 0.20 0.25 0.30];
dList   = [0.05 0.10 0.20 0.35 0.50 0.70];

results = [];

fprintf("\n=== COARSE SEARCH ===\n");

for i = 1:length(rpmList)
    for j = 1:length(dList)

        rpmAlpha = rpmList(i);
        dAlpha   = dList(j);

        fprintf("\nTesting rpmAlpha=%.4f | dAlpha=%.4f\n", rpmAlpha, dAlpha);

        try
            [t, rpm, pwm] = runTest( ...
                a, enc, R_PWM, L_PWM, R_EN, L_EN, ...
                kp, ki, kd, rpmAlpha, dAlpha, ...
                targetRPM, testTime, Ts, CPR_TOTAL, RPM_SAFETY_LIMIT, ...
                fig, ax1, ax2, rpmLine, targetLine, pwmLine);

            score = calcScore(t, rpm, pwm, targetRPM);

        catch ME
            fprintf("Test failed: %s\n", ME.message);
            score = 1e9;
        end

        fprintf("Score = %.6f\n", score);

        results = [results; rpmAlpha dAlpha score];

        stopMotor(a, R_PWM, L_PWM, R_EN, L_EN);
        pause(pauseTime);
    end
end

%% =========================
% Pick best coarse result
%% =========================
[~, idx] = min(results(:,3));
best = results(idx,:);

bestRpmAlpha = best(1);
bestDAlpha   = best(2);

fprintf("\nBest coarse result:\n");
fprintf("rpmAlpha = %.6f\n", bestRpmAlpha);
fprintf("dAlpha   = %.6f\n", bestDAlpha);
fprintf("score    = %.6f\n", best(3));

%% =========================
% Stage 2: Fine search
%% =========================
range = 0.05;

rpmListFine = linspace(max(0.02, bestRpmAlpha - range), min(0.30, bestRpmAlpha + range), 5);
dListFine   = linspace(max(0.02, bestDAlpha   - range), min(0.80, bestDAlpha   + range), 5);

resultsFine = [];

fprintf("\n=== FINE SEARCH ===\n");

for i = 1:length(rpmListFine)
    for j = 1:length(dListFine)

        rpmAlpha = rpmListFine(i);
        dAlpha   = dListFine(j);

        fprintf("\nRefining rpmAlpha=%.6f | dAlpha=%.6f\n", rpmAlpha, dAlpha);

        try
            [t, rpm, pwm] = runTest( ...
                a, enc, R_PWM, L_PWM, R_EN, L_EN, ...
                kp, ki, kd, rpmAlpha, dAlpha, ...
                targetRPM, testTime, Ts, CPR_TOTAL, RPM_SAFETY_LIMIT, ...
                fig, ax1, ax2, rpmLine, targetLine, pwmLine);

            score = calcScore(t, rpm, pwm, targetRPM);

        catch ME
            fprintf("Test failed: %s\n", ME.message);
            score = 1e9;
        end

        fprintf("Score = %.6f\n", score);

        resultsFine = [resultsFine; rpmAlpha dAlpha score];

        stopMotor(a, R_PWM, L_PWM, R_EN, L_EN);
        pause(pauseTime);
    end
end

%% =========================
% Final best
%% =========================
allResults = [results; resultsFine];

[~, idx] = min(allResults(:,3));
finalBest = allResults(idx,:);

finalRpmAlpha = finalBest(1);
finalDAlpha   = finalBest(2);
finalScore    = finalBest(3);

fprintf("\n==============================\n");
fprintf("FINAL BEST RESULT\n");
fprintf("==============================\n");
fprintf("Kp       = %.9f\n", kp);
fprintf("Ki       = %.9f\n", ki);
fprintf("Kd       = %.9f\n", kd);
fprintf("rpmAlpha = %.9f\n", finalRpmAlpha);
fprintf("dAlpha   = %.9f\n", finalDAlpha);
fprintf("score    = %.9f\n", finalScore);
fprintf("==============================\n");

%% =========================
% Final run using best values
%% =========================
fprintf("\nRunning final test with best values...\n");

[tFinal, rpmFinal, pwmFinal] = runTest( ...
    a, enc, R_PWM, L_PWM, R_EN, L_EN, ...
    kp, ki, kd, finalRpmAlpha, finalDAlpha, ...
    targetRPM, testTime, Ts, CPR_TOTAL, RPM_SAFETY_LIMIT, ...
    fig, ax1, ax2, rpmLine, targetLine, pwmLine);

finalMetrics = calcMetrics(tFinal, rpmFinal, pwmFinal, targetRPM);

fprintf("\nFinal Metrics:\n");
fprintf("Rise Time          = %.4f s\n", finalMetrics.RiseTime);
fprintf("Overshoot          = %.4f %%\n", finalMetrics.Overshoot);
fprintf("Settling Time      = %.4f s\n", finalMetrics.SettlingTime);
fprintf("Steady-State Error = %.4f\n", finalMetrics.SteadyStateError);
fprintf("IAE                = %.6f\n", finalMetrics.IAE);
fprintf("Final Score        = %.6f\n", finalMetrics.Score);

%% =========================
% Save results
%% =========================
resultsTable = array2table(allResults, ...
    "VariableNames", ["rpmAlpha", "dAlpha", "Score"]);

resultsTable = sortrows(resultsTable, "Score", "ascend");

bestTable = table( ...
    kp, ki, kd, finalRpmAlpha, finalDAlpha, finalScore, ...
    finalMetrics.RiseTime, finalMetrics.Overshoot, ...
    finalMetrics.SettlingTime, finalMetrics.SteadyStateError, ...
    finalMetrics.IAE, finalMetrics.Score, ...
    "VariableNames", ["Kp", "Ki", "Kd", "Best_rpmAlpha", "Best_dAlpha", ...
    "OptimizationScore", "RiseTime", "OvershootPercent", ...
    "SettlingTime", "SteadyStateError", "IAE", "FinalScore"]);

writetable(resultsTable, "alpha_all_test_results.csv");
writetable(bestTable, "alpha_best_result.csv");

fprintf("\nSaved files:\n");
fprintf("alpha_all_test_results.csv\n");
fprintf("alpha_best_result.csv\n");

%% =========================
% Stop motor
%% =========================
stopMotor(a, R_PWM, L_PWM, R_EN, L_EN);

%% ========================================================================
% Local functions
%% ========================================================================

function [tLog, rpmLog, pwmLog] = runTest( ...
    a, enc, R_PWM, L_PWM, R_EN, L_EN, ...
    kp, ki, kd, rpmAlpha, dAlpha, ...
    targetRPM, testTime, Ts, CPR_TOTAL, RPM_SAFETY_LIMIT, ...
    fig, ax1, ax2, rpmLine, targetLine, pwmLine)

PWM_MIN = 0;
PWM_MAX = 255;

I_LIMIT = 200;

pwmSlewUp   = 200;
pwmSlewDown = 400;

softStartSec = 0.4;
softStartPWM = 50;

updatePlotEvery = 0.20;

writeDigitalPin(a, R_EN, 1);
writeDigitalPin(a, L_EN, 1);

resetCount(enc);
lastCount = readCount(enc);

integral = 0;
prevErr = 0;
rpmFilt = 0;
dFilt = 0;
lastPWM = 0;

firstSample = true;

maxSamples = ceil(testTime / Ts) + 10;

tLog   = nan(maxSamples, 1);
rpmLog = nan(maxSamples, 1);
pwmLog = nan(maxSamples, 1);

set(rpmLine, "XData", nan, "YData", nan);
set(targetLine, "XData", nan, "YData", nan);
set(pwmLine, "XData", nan, "YData", nan);

title(ax1, sprintf("Response | rpmAlpha = %.4f | dAlpha = %.4f", rpmAlpha, dAlpha));
title(ax2, "PWM Signal");

ylim(ax1, [0 max(targetRPM * 1.8, 10)]);
ylim(ax2, [0 255]);
xlim(ax1, [0 testTime]);
xlim(ax2, [0 testTime]);

drawnow;

k = 0;
tStart = tic;
tPrev = toc(tStart);
lastPlotUpdate = 0;

while true

    if ~isvalid(fig)
        error("Figure closed. Test stopped.");
    end

    tNow = toc(tStart);

    if tNow >= testTime
        break;
    end

    dt = tNow - tPrev;

    if dt < Ts
        pause(0.001);
        continue;
    end

    tPrev = tNow;

    count = readCount(enc);
    delta = count - lastCount;
    lastCount = count;

    rpmRaw = (double(delta) * 60.0) / (double(CPR_TOTAL) * dt);
    rpmRaw = abs(rpmRaw);

    rpmFilt = rpmAlpha * rpmRaw + (1 - rpmAlpha) * rpmFilt;

    if firstSample
        rpmFilt = rpmRaw;
        prevErr = targetRPM - rpmFilt;
        dFilt = 0;
        integral = 0;
        firstSample = false;
    end

    if rpmFilt > RPM_SAFETY_LIMIT
        stopMotor(a, R_PWM, L_PWM, R_EN, L_EN);
        error("Safety stop: RPM exceeded %.2f RPM.", RPM_SAFETY_LIMIT);
    end

    if tNow < softStartSec
        pwmCmd = softStartPWM;
        err = targetRPM - rpmFilt;
    else
        err = targetRPM - rpmFilt;

        integral = integral + err * dt;
        integral = max(-I_LIMIT, min(I_LIMIT, integral));

        d = (err - prevErr) / dt;
        dFilt = dAlpha * d + (1 - dAlpha) * dFilt;

        u = kp * err + ki * integral + kd * dFilt;
        pwmCmd = round(u);

        if pwmCmd > PWM_MAX
            pwmCmd = PWM_MAX;
            if err > 0
                integral = integral - err * dt;
            end
        elseif pwmCmd < PWM_MIN
            pwmCmd = PWM_MIN;
            if err < 0
                integral = integral - err * dt;
            end
        end

        prevErr = err;
    end

    stepUp = pwmSlewUp * dt;
    stepDown = pwmSlewDown * dt;

    if pwmCmd > lastPWM
        pwmCmd = min(pwmCmd, lastPWM + stepUp);
    else
        pwmCmd = max(pwmCmd, lastPWM - stepDown);
    end

    pwmCmd = round(max(PWM_MIN, min(PWM_MAX, pwmCmd)));
    lastPWM = pwmCmd;

    duty = pwmCmd / 255;

    writeDigitalPin(a, R_EN, 1);
    writeDigitalPin(a, L_EN, 1);

    writePWMDutyCycle(a, R_PWM, duty);
    writePWMDutyCycle(a, L_PWM, 0);

    k = k + 1;

    tLog(k)   = tNow;
    rpmLog(k) = rpmFilt;
    pwmLog(k) = pwmCmd;

    if (tNow - lastPlotUpdate) >= updatePlotEvery
        validIdx = ~isnan(tLog);

        set(rpmLine, "XData", tLog(validIdx), "YData", rpmLog(validIdx));
        set(targetLine, "XData", tLog(validIdx), "YData", targetRPM * ones(sum(validIdx), 1));
        set(pwmLine, "XData", tLog(validIdx), "YData", pwmLog(validIdx));

        drawnow limitrate;

        lastPlotUpdate = tNow;
    end
end

stopMotor(a, R_PWM, L_PWM, R_EN, L_EN);

validIdx = ~isnan(tLog);

tLog   = tLog(validIdx);
rpmLog = rpmLog(validIdx);
pwmLog = pwmLog(validIdx);

set(rpmLine, "XData", tLog, "YData", rpmLog);
set(targetLine, "XData", tLog, "YData", targetRPM * ones(size(tLog)));
set(pwmLine, "XData", tLog, "YData", pwmLog);
drawnow;

end

function score = calcScore(t, rpm, pwm, targetRPM)

metrics = calcMetrics(t, rpm, pwm, targetRPM);
score = metrics.Score;

end

function metrics = calcMetrics(t, rpm, pwm, targetRPM)

if isempty(t) || isempty(rpm) || numel(t) < 5
    metrics.RiseTime = inf;
    metrics.Overshoot = inf;
    metrics.SettlingTime = inf;
    metrics.SteadyStateError = inf;
    metrics.IAE = inf;
    metrics.Score = 1e9;
    return;
end

err = targetRPM - rpm;

IAE = trapz(t, abs(err)) / max(targetRPM * t(end), eps);

maxRPM = max(rpm);
overshoot = max(0, ((maxRPM - targetRPM) / max(targetRPM, eps)) * 100);

idxRise = find(rpm >= 0.9 * targetRPM, 1, "first");

if isempty(idxRise)
    riseTime = t(end);
else
    riseTime = t(idxRise);
end

band = 0.05 * targetRPM;
settlingTime = t(end);

for i = 1:numel(t)
    if all(abs(rpm(i:end) - targetRPM) <= band)
        settlingTime = t(i);
        break;
    end
end

lastN = max(3, round(0.15 * numel(rpm)));
steadyStateError = abs(targetRPM - mean(rpm(end-lastN+1:end))) / max(targetRPM, eps);

pwmSaturationRatio = mean(pwm >= 250);

metrics.RiseTime = riseTime;
metrics.Overshoot = overshoot;
metrics.SettlingTime = settlingTime;
metrics.SteadyStateError = steadyStateError;
metrics.IAE = IAE;

metrics.Score = ...
    0.40 * IAE + ...
    0.20 * (overshoot / 100) + ...
    0.15 * (riseTime / max(t(end), eps)) + ...
    0.15 * (settlingTime / max(t(end), eps)) + ...
    0.10 * steadyStateError + ...
    0.20 * pwmSaturationRatio;

end

function stopMotor(a, R_PWM, L_PWM, R_EN, L_EN)

try
    writePWMDutyCycle(a, R_PWM, 0);
    writePWMDutyCycle(a, L_PWM, 0);
    writeDigitalPin(a, R_EN, 0);
    writeDigitalPin(a, L_EN, 0);
catch
end

end