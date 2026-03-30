function J = cost_motor(K, model)

    %% Controller gains
    kp = K(1);
    ki = K(2);
    kd = K(3);

    assignin('base','kp',kp);
    assignin('base','ki',ki);
    assignin('base','kd',kd);

    %% System initialization
    init_motor();

    %% Get simulation time
    if evalin('base', 'exist(''sim_time'', ''var'')')
        sim_time = evalin('base','sim_time');
    else
        sim_time = 10;
    end

    %% Run simulation
    drawnow;
    pause(0.01);

    try
        simOut = sim(model, 'StopTime', num2str(sim_time), 'FastRestart', 'on');
    catch
        J = 1e6;
        return
    end

    %% Get response
    resp = simOut.OutputResponse;
    t = resp.Time;
    y = resp.Data;

    %% Validate response
    if isempty(t) || isempty(y) || any(isnan(y)) || any(isinf(y)) || max(abs(y)) > 1e5
        J = 1e6;
        return
    end

    %% Get reference amplitude
    if evalin('base', 'exist(''step_amp'', ''var'')')
        ref = evalin('base','step_amp');
    elseif evalin('base', 'exist(''ref'', ''var'')')
        ref = evalin('base','ref');
    else
        ref = 1;
    end

    %% Step response information
    info = stepinfo(y, t, ref);

    if isempty(info.SettlingTime) || isnan(info.SettlingTime)
        J = 1e6;
        return
    end

    %% Performance terms
    ess = abs(ref - y(end));
    OS  = info.Overshoot / 100;
    Tp  = info.PeakTime;

    if isempty(Tp) || isnan(Tp)
        Tp = sim_time;
    end

    %% Cost function
    % The sum of weights is 1
    J = 0.8 * OS + 0.1 * ess + 0.1 * Tp;

end