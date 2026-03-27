function J = cost_ballandbeam(K, model)

%% PID
kp = K(1);
ki = K(2);

assignin('base','kp',kp);
assignin('base','ki',ki);

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

% Use step_amp from the workspace instead of fixed 0.25
step_amp = evalin('base','step_amp');
e = abs(y(end) - step_amp); % tracking error

J = 0.3*info.Overshoot + 0.2*info.SettlingTime + 0.5*e;

end