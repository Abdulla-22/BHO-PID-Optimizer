function J = cost_motor(K, model)

    %% PID
    kp = K(1);
    ki = K(2);
    kd = K(3);

    assignin('base','kp',kp);
    assignin('base','ki',ki);
    assignin('base','kd',kd);

    %% System init
    init_motor();

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

    if isempty(info.SettlingTime) || isnan(info.SettlingTime)
        J = 1e6;
        return
    end

    %% Performance
    step_amp = evalin('base','step_amp');
    ess = abs(step_amp - y(end));
    OS  = info.Overshoot / 100;
    Ts  = info.SettlingTime;

    %% Cost
    J = Ts + 2*OS + 5*ess; 

end