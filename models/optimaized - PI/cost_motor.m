function J = cost_motor(K, model)

    %% PID
    kp = K(1);
    ki = K(2);

    assignin('base','kp',kp);
    assignin('base','ki',ki);

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

    %% Cost: penalize deviations from desired OS and RT
    J = 0.8*OS + 0.1*ess + 0.1*info.PeakTime; %% sum of them should be 1
end