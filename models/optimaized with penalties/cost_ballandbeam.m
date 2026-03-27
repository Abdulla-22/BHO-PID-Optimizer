function J = cost_ballandbeam(K, model, desired)

%% PID
kp = K(1);
ki = K(2);
kd = K(3);

assignin('base','kp',kp);
assignin('base','ki',ki);
assignin('base','kd',kd);

init_ballandbeam();

%% Simulation
try
    simOut = sim(model,'StopTime','10','FastRestart','on');
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

step_amp = evalin('base','step_amp');
e = abs(y(end) - step_amp); % tracking error
OS  = info.Overshoot / 100;
RT  = info.RiseTime;

%% Cost function: penalize deviations from desired OS & RT
J = 0.3*info.Overshoot + 0.2*info.SettlingTime + 0.5*e;

end