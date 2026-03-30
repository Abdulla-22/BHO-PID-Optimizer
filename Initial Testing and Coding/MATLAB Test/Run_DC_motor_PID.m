%% =====================================
% DC Motor Speed PID Control (RPM)
% STOP button + Safe exit + Fullscreen
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

SETPOINT_RPM = 50.0;

Kp = 5.0;
Ki = 3.0;
Kd = 0.03;

forwardDir = true;

rpmAlpha = 0.25;
dAlpha   = 0.15;
I_LIMIT  = 200.0;

SOFTSTART_SEC = 0.4;
SOFTSTART_PWM = 50;

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

applyMotorPWM(a, ENA, IN1, IN2, 0, forwardDir, PWM_MIN, PWM_MAX);
lastCount = readCount(enc);

%% ---- PID states ----
integral  = 0.0;
prevError = 0.0;
dFiltered = 0.0;
rpmFilt   = 0.0;

%% ---- Data buffers ----
N0 = 4000;
tLog    = nan(N0,1);
rpmRawL = nan(N0,1);
rpmFilL = nan(N0,1);
pwmL    = nan(N0,1);
errL    = nan(N0,1);
spL     = nan(N0,1);
k = 0;

%% ---- Figure Full Screen ----
fig = figure('Name','DC Motor RPM Control','NumberTitle','off');
fig.WindowState = 'maximized';   % FULL SCREEN

%% ---- STOP FLAG ----
setappdata(fig,'stopFlag',false);

% Stop button
uicontrol('Style','pushbutton',...
          'String','STOP',...
          'FontSize',14,...
          'FontWeight','bold',...
          'BackgroundColor',[1 0.2 0.2],...
          'Units','normalized',...
          'Position',[0.88 0.93 0.1 0.05],...
          'Callback',@(src,event)setappdata(fig,'stopFlag',true));

% Close window → stop
fig.CloseRequestFcn = @(src,event)setappdata(fig,'stopFlag',true);

%% ---- Subplots ----
ax1 = subplot(3,2,[1 2]); grid on; hold on;
hRPM = plot(nan,nan,'LineWidth',1.5);
hSP  = plot(nan,nan,'--','LineWidth',1.5);
xlabel('Time (s)'); ylabel('RPM');
title('Motor Speed vs Setpoint');
legend({'Filtered RPM','Setpoint'});

ax2 = subplot(3,2,3); grid on; hold on;
hPWM = plot(nan,nan,'LineWidth',1.5);
xlabel('Time (s)'); ylabel('PWM');
ylim([0 255]); title('PWM Output');

ax3 = subplot(3,2,4); grid on; hold on;
hERR = plot(nan,nan,'LineWidth',1.5);
xlabel('Time (s)'); ylabel('Error');
title('Control Error');

ax4 = subplot(3,2,5); grid on; hold on;
hRAW = plot(nan,nan,'LineWidth',1.5);
xlabel('Time (s)'); ylabel('Raw RPM');
title('Raw Encoder RPM');

ax5 = subplot(3,2,6); axis off;
hDBG = text(0.05,0.9,"",'FontSize',12,'VerticalAlignment','top');

%% ---- Run Loop ----
disp("Running... Close window or press STOP.");

t0 = tic;
tPrev = toc(t0);
nextTick = tPrev;
firstSample = true;
lastPlotUpdate = 0;

while isvalid(fig) && ~getappdata(fig,'stopFlag')

    t = toc(t0);
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

    count = readCount(enc);
    delta = count - lastCount;
    lastCount = count;

    rpmRaw = (double(delta) * 60.0) / (double(CPR_TOTAL) * dt);
    rpmRawMag = abs(rpmRaw);

    rpmFilt = rpmAlpha*rpmRawMag + (1-rpmAlpha)*rpmFilt;

    if firstSample
        rpmFilt = rpmRawMag;
        prevError = SETPOINT_RPM - rpmFilt;
        integral = 0;
        dFiltered = 0;
        firstSample = false;
    end

    if t < SOFTSTART_SEC
        pwmCmd = SOFTSTART_PWM;
    else
        error = SETPOINT_RPM - rpmFilt;

        integral = integral + error*dt;
        integral = max(-I_LIMIT,min(I_LIMIT,integral));

        d = (error-prevError)/dt;
        dFiltered = dAlpha*d + (1-dAlpha)*dFiltered;

        u = Kp*error + Ki*integral + Kd*dFiltered;
        pwmCmd = round(u);

        pwmCmd = max(PWM_MIN,min(PWM_MAX,pwmCmd));
        prevError = error;
    end

    applyMotorPWM(a, ENA, IN1, IN2, pwmCmd, forwardDir, PWM_MIN, PWM_MAX);

    err = SETPOINT_RPM - rpmFilt;

    k = k + 1;
    if k > numel(tLog)
        tLog = [tLog; nan(N0,1)];
        rpmRawL = [rpmRawL; nan(N0,1)];
        rpmFilL = [rpmFilL; nan(N0,1)];
        pwmL = [pwmL; nan(N0,1)];
        errL = [errL; nan(N0,1)];
        spL = [spL; nan(N0,1)];
    end

    tLog(k)=t;
    rpmRawL(k)=rpmRawMag;
    rpmFilL(k)=rpmFilt;
    pwmL(k)=pwmCmd;
    errL(k)=err;
    spL(k)=SETPOINT_RPM;

    if (t-lastPlotUpdate)>=0.1
        lastPlotUpdate=t;
        idx=1:k;

        set(hRPM,'XData',tLog(idx),'YData',rpmFilL(idx));
        set(hSP ,'XData',tLog(idx),'YData',spL(idx));
        set(hPWM,'XData',tLog(idx),'YData',pwmL(idx));
        set(hERR,'XData',tLog(idx),'YData',errL(idx));
        set(hRAW,'XData',tLog(idx),'YData',rpmRawL(idx));

        xlim(ax1,[max(0,t-10) t+0.2]);
        xlim(ax2,[max(0,t-10) t+0.2]);
        xlim(ax3,[max(0,t-10) t+0.2]);
        xlim(ax4,[max(0,t-10) t+0.2]);

        ylim(ax1,[0 max(SETPOINT_RPM*1.5,50)]);
        ylim(ax4,[0 max(SETPOINT_RPM*1.5,50)]);

        dbg = sprintf(['Time: %.2f s\nRPM: %.2f\nError: %.2f\nPWM: %d\n' ...
                       'Kp=%.2f Ki=%.2f Kd=%.2f'],...
                       t,rpmFilt,err,pwmCmd,Kp,Ki,Kd);
        set(hDBG,'String',dbg);

        drawnow limitrate;
    end
end

%% ---- SAFE STOP ----
applyMotorPWM(a, ENA, IN1, IN2, 0, forwardDir, PWM_MIN, PWM_MAX);

if isvalid(fig)
    delete(fig);
end

disp("Stopped.");

%% ---- Motor Function ----
function applyMotorPWM(a, ENA, IN1, IN2, pwm, forward, PWM_MIN, PWM_MAX)

    pwm = max(PWM_MIN,min(PWM_MAX,pwm));

    if pwm==0
        writeDigitalPin(a,IN1,0);
        writeDigitalPin(a,IN2,0);
        writePWMDutyCycle(a,ENA,0);
        return;
    end

    if forward
        writeDigitalPin(a,IN1,1);
        writeDigitalPin(a,IN2,0);
    else
        writeDigitalPin(a,IN1,0);
        writeDigitalPin(a,IN2,1);
    end

    writePWMDutyCycle(a,ENA,pwm/255);
end