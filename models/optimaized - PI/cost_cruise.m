function J = cost_cruise(K, model)

%% PID
kp = K(1);
ki = K(2);

assignin('base','kp',kp);
assignin('base','ki',ki);

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

J = 0.1*info.SettlingTime + 0.4*info.Overshoot + 0.5*ess;

end