clc;
clear;
close all;

%% =========================
% Arduino / Motor pins
%% =========================
COM   = "COM5";       % Change this to your Arduino port
BOARD = "Nano3";

R_PWM = "D9";
L_PWM = "D10";
R_EN  = "D8";
L_EN  = "D7";

ENC_A = "D2";
ENC_B = "D3";

%% =========================
% Encoder / motor settings
%% =========================
CPR_MOTOR_4X = 64.0;
GEAR_RATIO   = 131.25;
CPR_TOTAL    = CPR_MOTOR_4X * GEAR_RATIO;

%% =========================
% Fixed PID values
%% =========================
Kp = 4.9066;
Ki = 5.7438;
Kd = 2.5421;

%% =========================
% Test settings
%% =========================
targetRPM = 50;
testSec   = 5;
Ts        = 0.05;

PWM_MIN = 0;
PWM_MAX = 255;

I_LIMIT = 200;

SOFTSTART_SEC = 0.4;
SOFTSTART_PWM = 50;

pwmSlewUp   = 200;
pwmSlewDown = 400;

%% =========================
% Alpha search range
%% =========================
rpmAlpha_range = 0.05:0.025:0.30;
dAlpha_range   = 0.02:0.02:0.20;

%% =========================
% Connect Arduino
%% =========================
fprintf("Connecting to Arduino...\n");

a = arduino(COM, BOARD, "Libraries", "rotaryEncoder");

configurePin(a, R_PWM, "PWM");
configurePin(a, L_PWM, "PWM");
configurePin(a, R_EN,  "DigitalOutput");
configurePin(a, L_EN,  "DigitalOutput");

writeDigitalPin(a, R_EN, 1);
writeDigitalPin(a, L_EN, 1);

enc = rotaryEncoder(a, ENC_A, ENC_B, round(CPR_TOTAL));

applyPWM(a, R_PWM, L_PWM, R_EN, L_EN, 0, true);
pause(1);

%% =========================
% Results table
%% =========================
Results = table([], [], [], [], [], [], [], ...
    'VariableNames', {'rpmAlpha','dAlpha','Cost','IAE','Overshoot','Ess','RiseTime'});

bestCost = inf;
bestRPMAlpha = NaN;
bestDAlpha = NaN;
bestData = [];

testNumber = 0;
totalTests = numel(rpmAlpha_range) * numel(dAlpha_range);

%% =========================
% Main tuning loop
%% =========================
try
    for r = 1:numel(rpmAlpha_range)
        for d = 1:numel(dAlpha_range)

            testNumber = testNumber + 1;

            rpmAlpha = rpmAlpha_range(r);
            dAlpha   = dAlpha_range(d);

            fprintf("\nTest %d/%d | rpmAlpha = %.4f | dAlpha = %.4f\n", ...
                testNumber, totalTests, rpmAlpha, dAlpha);

            applyPWM(a, R_PWM, L_PWM, R_EN, L_EN, 0, true);
            pause(1.0);

            [cost, metrics, data] = runAlphaTest( ...
                a, enc, ...
                R_PWM, L_PWM, R_EN, L_EN, ...
                targetRPM, testSec, Ts, ...
                CPR_TOTAL, ...
                Kp, Ki, Kd, ...
                rpmAlpha, dAlpha, ...
                I_LIMIT, ...
                SOFTSTART_SEC, SOFTSTART_PWM, ...
                PWM_MIN, PWM_MAX, ...
                pwmSlewUp, pwmSlewDown);

            Results = [Results; table( ...
                rpmAlpha, dAlpha, cost, metrics.IAE, metrics.Overshoot, metrics.Ess, metrics.RiseTime, ...
                'VariableNames', {'rpmAlpha','dAlpha','Cost','IAE','Overshoot','Ess','RiseTime'})];

            fprintf("Cost = %.6f | OS = %.2f %% | Ess = %.4f | RT = %.3f s\n", ...
                cost, metrics.Overshoot, metrics.Ess, metrics.RiseTime);

            if cost < bestCost
                bestCost = cost;
                bestRPMAlpha = rpmAlpha;
                bestDAlpha = dAlpha;
                bestData = data;

                fprintf(">>> New best found\n");
            end
        end
    end

catch ME
    applyPWM(a, R_PWM, L_PWM, R_EN, L_EN, 0, true);
    writeDigitalPin(a, R_EN, 0);
    writeDigitalPin(a, L_EN, 0);
    rethrow(ME);
end

%% =========================
% Stop motor
%% =========================
applyPWM(a, R_PWM, L_PWM, R_EN, L_EN, 0, true);
writeDigitalPin(a, R_EN, 0);
writeDigitalPin(a, L_EN, 0);

%% =========================
% Show best result
%% =========================
fprintf("\n================ BEST RESULT ================\n");
fprintf("Best rpmAlpha = %.6f\n", bestRPMAlpha);
fprintf("Best dAlpha   = %.6f\n", bestDAlpha);
fprintf("Best Cost     = %.6f\n", bestCost);

disp(sortrows(Results, "Cost"));

%% =========================
% Save results
%% =========================
timeStamp = char(datetime("now", "Format", "yyyy-MM-dd_HH-mm-ss"));
csvName = "Alpha_Tuning_Results_" + timeStamp + ".csv";
writetable(Results, csvName);

fprintf("\nResults saved to: %s\n", csvName);

%% =========================
% Plot best response
%% =========================
if ~isempty(bestData)
    figure;
    hold on;
    grid on;

    plot(bestData.t, bestData.target, "LineWidth", 1.5);
    plot(bestData.t, bestData.rpm, "LineWidth", 1.5);
    plot(bestData.t, bestData.pwm, "LineWidth", 1.5);

    xlabel("Time (s)");
    ylabel("RPM / PWM");
    title("Best Hardware Motor Response");
    legend("Target RPM", "Actual RPM", "PWM", "Location", "best");
end

%% ============================================================
% Local functions
%% ============================================================

function [J, metrics, data] = runAlphaTest( ...
    a, enc, ...
    R_PWM, L_PWM, R_EN, L_EN, ...
    targetRPM, testSec, Ts, ...
    CPR_TOTAL, ...
    Kp, Ki, Kd, ...
    rpmAlpha, dAlpha, ...
    I_LIMIT, ...
    SOFTSTART_SEC, SOFTSTART_PWM, ...
    PWM_MIN, PWM_MAX, ...
    pwmSlewUp, pwmSlewDown)

    tData = [];
    rpmData = [];
    pwmData = [];
    errData = [];

    integral = 0;
    prevError = 0;
    dFiltered = 0;
    rpmFilt = 0;
    lastPWM = 0;
    firstSample = true;

    lastCount = readCount(enc);

    tStart = tic;
    tPrev = 0;

    while toc(tStart) < testSec

        tNow = toc(tStart);
        dt = tNow - tPrev;

        if dt < Ts
            pause(Ts / 5);
            continue;
        end

        if dt <= 0 || ~isfinite(dt)
            continue;
        end

        tPrev = tNow;

        countNow = readCount(enc);
        delta = countNow - lastCount;
        lastCount = countNow;

        rpmRaw = abs((double(delta) * 60.0) / (double(CPR_TOTAL) * dt));

        if ~isfinite(rpmRaw)
            J = 1e8;
            metrics = emptyMetrics();
            data = [];
            return;
        end

        rpmFilt = rpmAlpha * rpmRaw + (1 - rpmAlpha) * rpmFilt;

        if firstSample
            rpmFilt = rpmRaw;
            prevError = targetRPM - rpmFilt;
            dFiltered = 0;
            integral = 0;
            firstSample = false;
        end

        if tNow < SOFTSTART_SEC
            pwmCmd = SOFTSTART_PWM;
            errNow = targetRPM - rpmFilt;
        else
            errNow = targetRPM - rpmFilt;

            integral = integral + errNow * dt;
            integral = max(-I_LIMIT, min(I_LIMIT, integral));

            dRaw = (errNow - prevError) / dt;
            dFiltered = dAlpha * dRaw + (1 - dAlpha) * dFiltered;

            u = Kp * errNow + Ki * integral + Kd * dFiltered;
            pwmCmd = round(u);

            if pwmCmd > PWM_MAX
                pwmCmd = PWM_MAX;
                if errNow > 0
                    integral = integral - errNow * dt;
                end
            elseif pwmCmd < PWM_MIN
                pwmCmd = PWM_MIN;
                if errNow < 0
                    integral = integral - errNow * dt;
                end
            end

            prevError = errNow;
        end

        pwmCmd = limitPwmSlewLocal(pwmCmd, lastPWM, dt, PWM_MIN, PWM_MAX, pwmSlewUp, pwmSlewDown);
        lastPWM = pwmCmd;

        applyPWM(a, R_PWM, L_PWM, R_EN, L_EN, pwmCmd, true);

        tData(end+1,1) = tNow;
        rpmData(end+1,1) = rpmFilt;
        pwmData(end+1,1) = pwmCmd;
        errData(end+1,1) = errNow;
    end

    applyPWM(a, R_PWM, L_PWM, R_EN, L_EN, 0, true);
    pause(0.3);

    if numel(tData) < 5 || any(~isfinite(rpmData))
        J = 1e8;
        metrics = emptyMetrics();
        data = [];
        return;
    end

    absError = abs(targetRPM - rpmData);

    IAE = trapz(tData, absError);
    IAE_norm = IAE / max(targetRPM * testSec, eps);

    overshoot = max(0, (max(rpmData) - targetRPM) / max(targetRPM, eps)) * 100;
    ess = abs(targetRPM - rpmData(end)) / max(targetRPM, eps);

    pwmPenalty = mean(pwmData) / 255;

    idx10 = find(rpmData >= 0.10 * targetRPM, 1, 'first');
    idx90 = find(rpmData >= 0.90 * targetRPM, 1, 'first');

    if isempty(idx10) || isempty(idx90) || idx90 <= idx10
        riseTime = testSec;
    else
        riseTime = tData(idx90) - tData(idx10);
    end

    riseNorm = riseTime / max(testSec, eps);

    J = (0.45 * IAE_norm) + ...
        (0.20 * ess) + ...
        (0.20 * overshoot / 100) + ...
        (0.10 * riseNorm) + ...
        (0.05 * pwmPenalty);

    if max(rpmData) > targetRPM * 2.0
        J = J * 10;
    end

    if rpmData(end) < targetRPM * 0.10
        J = J * 5;
    end

    metrics.IAE = IAE;
    metrics.Overshoot = overshoot;
    metrics.Ess = ess;
    metrics.RiseTime = riseTime;

    data.t = tData;
    data.rpm = rpmData;
    data.pwm = pwmData;
    data.err = errData;
    data.target = targetRPM * ones(size(tData));
end

function applyPWM(a, R_PWM, L_PWM, R_EN, L_EN, pwm, forwardDir)
    pwm = max(0, min(255, pwm));
    duty = pwm / 255;

    writeDigitalPin(a, R_EN, 1);
    writeDigitalPin(a, L_EN, 1);

    if pwm == 0
        writePWMDutyCycle(a, R_PWM, 0);
        writePWMDutyCycle(a, L_PWM, 0);
        return;
    end

    if forwardDir
        writePWMDutyCycle(a, R_PWM, duty);
        writePWMDutyCycle(a, L_PWM, 0);
    else
        writePWMDutyCycle(a, R_PWM, 0);
        writePWMDutyCycle(a, L_PWM, duty);
    end
end

function pwmOut = limitPwmSlewLocal(pwmIn, lastPWMValue, dt, PWM_MIN, PWM_MAX, pwmSlewUp, pwmSlewDown)
    pwmIn = max(PWM_MIN, min(PWM_MAX, pwmIn));
    lastPWMValue = max(PWM_MIN, min(PWM_MAX, lastPWMValue));

    stepUp = pwmSlewUp * dt;
    stepDown = pwmSlewDown * dt;

    if pwmIn > lastPWMValue
        pwmOut = min(pwmIn, lastPWMValue + stepUp);
    else
        pwmOut = max(pwmIn, lastPWMValue - stepDown);
    end

    pwmOut = round(pwmOut);
end

function metrics = emptyMetrics()
    metrics.IAE = NaN;
    metrics.Overshoot = NaN;
    metrics.Ess = NaN;
    metrics.RiseTime = NaN;
end