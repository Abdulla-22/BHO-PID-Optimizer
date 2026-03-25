function J = cost_ballandbeam(K, model)

%% PID
kp = K(1);
ki = K(2);
kd = K(3);

assignin('base','kp',kp);
assignin('base','ki',ki);
assignin('base','kd',kd);

init_ballandbeam();

%% Simulation
simOut = sim(model,'StopTime','10','FastRestart','on');

resp = simOut.OutputResponse;
t = resp.Time;
y = resp.Data;

info = stepinfo(y,t);

%% Cost
if isnan(info.Overshoot) || isempty(info.SettlingTime)
    J = 1e6;
    return
end

e = abs(y(end) - 0.25); % tracking error

J = 5*info.Overshoot + info.SettlingTime + 20*e;

end