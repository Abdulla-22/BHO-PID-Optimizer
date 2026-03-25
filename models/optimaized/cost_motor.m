function J = cost_motor(K, model)

kp = K(1);
ki = K(2);
kd = K(3);

assignin('base','kp',kp);
assignin('base','ki',ki);
assignin('base','kd',kd);


init_motor();

simOut = sim(model,'StopTime','5','FastRestart','on');

resp = simOut.OutputResponse;
t = resp.Time;
y = resp.Data;

info = stepinfo(y,t);

J = info.Overshoot + info.SettlingTime + info.RiseTime;

end