function J = cost_motor(K, model)

    %% Controller gains
    kp = K(1);
    ki = K(2);
    kd = K(3);

    assignin('base','kp',kp);
    assignin('base','ki',ki);
    assignin('base','kd',kd);

    %% Get simulation time
    if evalin('base', 'exist(''sim_time'', ''var'')')
        sim_time = evalin('base','sim_time');
    else
        sim_time = 10;
    end

    %% Get user target specifications
    if evalin('base', 'exist(''wantedovershoot'', ''var'')')
        wantedovershoot = evalin('base','wantedovershoot');
    else
        wantedovershoot = 5;
    end

    if evalin('base', 'exist(''wantedrisetime'', ''var'')')
        wantedrisetime = evalin('base','wantedrisetime');
    else
        wantedrisetime = 1.5;
    end

    if evalin('base', 'exist(''wantedsettlingtime'', ''var'')')
        wantedsettlingtime = evalin('base','wantedsettlingtime');
    else
        wantedsettlingtime = 3.0;
    end

    if evalin('base', 'exist(''wantedess'', ''var'')')
        wantedess = evalin('base','wantedess');
    else
        wantedess = 0.01;
    end

    %% Run simulation
    try
        simOut = sim(model, ...
            'StopTime', num2str(sim_time), ...
            'CaptureErrors', 'on');
    catch
        J = 1e6;
        return
    end

    if ~isempty(simOut.ErrorMessage)
        J = 1e6;
        return
    end

    %% Get response
    try
        resp = simOut.OutputResponse;
        t = resp.Time;
        y = resp.Data;
    catch
        J = 1e6;
        return
    end

    %% Validate response
    if isempty(t) || isempty(y)
        J = 1e6;
        return
    end

    if any(isnan(y)) || any(isinf(y)) || max(abs(y)) > 1e4
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
    try
        info = stepinfo(y, t, ref);
    catch
        J = 1e6;
        return
    end

    %% Validate extracted info
    if isempty(info.RiseTime) || isnan(info.RiseTime) || ...
       isempty(info.SettlingTime) || isnan(info.SettlingTime) || ...
       isempty(info.Overshoot) || isnan(info.Overshoot)
        J = 1e6;
        return
    end

    %% Actual performance
    actual_overshoot    = info.Overshoot;
    actual_risetime     = info.RiseTime;
    actual_settlingtime = info.SettlingTime;
    actual_ess          = abs(ref - y(end));

    %% Exact target errors
    e_overshoot    = abs(actual_overshoot    - wantedovershoot);
    e_risetime     = abs(actual_risetime     - wantedrisetime);
    e_settlingtime = abs(actual_settlingtime - wantedsettlingtime);
    e_ess          = abs(actual_ess          - wantedess);

    %% Normalization
    e_overshoot_norm    = e_overshoot    / max(1, wantedovershoot);
    e_risetime_norm     = e_risetime     / max(1e-3, wantedrisetime);
    e_settlingtime_norm = e_settlingtime / max(1e-3, wantedsettlingtime);
    e_ess_norm          = e_ess          / max(1e-6, wantedess);

    %% Weights (sum = 1)
    w1 = 0.60;   % Overshoot
    w2 = 0.10;   % Rise time
    w3 = 0.10;   % Settling time
    w4 = 0.20;   % Steady-state error

    %% Final cost
    J = w1 * e_overshoot_norm + ...
        w2 * e_risetime_norm + ...
        w3 * e_settlingtime_norm + ...
        w4 * e_ess_norm;

    %% Protection
    if isnan(J) || isinf(J)
        J = 1e6;
    end

end