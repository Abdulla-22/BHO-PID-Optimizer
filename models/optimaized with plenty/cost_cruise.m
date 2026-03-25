function J = cost_cruise(K, model, desired)

%% PID
kp = K(1);
ki = K(2);
kd = K(3);

assignin('base','kp',kp);
assignin('base','ki',ki);
assignin('base','kd',kd);

init_cruise();

%% Simulation
try
    simOut = sim(model,'StopTime','120','FastRestart','on');
catch
    J = 1e6;
    return
end

resp = simOut.OutputResponse;
t = resp.Time;
y = resp.Data;

info = stepinfo(y,t);

%% Check simulation validity
if isempty(info.SettlingTime) || isnan(info.SettlingTime)
    J = 1e6;
    return
end

ess = abs(y(end)); % steady-state error
OS  = info.Overshoot / 100;
RT  = info.RiseTime;

%% Cost function: penalize deviations from desired OS & RT
J = info.SettlingTime + 5*ess + 5*abs(OS - desired.OS) + 2*abs(RT - desired.RT);

end