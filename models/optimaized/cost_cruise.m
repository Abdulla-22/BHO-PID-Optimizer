function J = cost_cruise(K, model)

%% PID
kp = K(1);
ki = K(2);
kd = K(3);

assignin('base','kp',kp);
assignin('base','ki',ki);
assignin('base','kd',kd);

init_cruise();

%% Simulation
simOut = sim(model,'StopTime','120','FastRestart','on');

resp = simOut.OutputResponse;
t = resp.Time;
y = resp.Data;

info = stepinfo(y,t);

%% Cost
if isnan(info.SettlingTime)
    J = 1e6;
    return
end

ess = abs(y(end)); % steady-state error

J = info.SettlingTime + info.Overshoot + 5*ess;

end