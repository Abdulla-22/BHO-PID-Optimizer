function J = cost_cruise(K, model)

    penalty = 1000;

    %% Controller gains
    kp = K(1);
    ki = K(2);
    kd = K(3);

    assignin('base','kp',kp);
    assignin('base','ki',ki);
    assignin('base','kd',kd);

    %% Initialize system
    init_cruise();

    %% Get simulation time
    if evalin('base', 'exist(''sim_time'', ''var'')')
        sim_time = evalin('base','sim_time');
    else
        sim_time = 120;
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
        wantedrisetime = 5;
    end

    if evalin('base', 'exist(''wantedsettlingtime'', ''var'')')
        wantedsettlingtime = evalin('base','wantedsettlingtime');
    else
        wantedsettlingtime = 7;
    end

    if evalin('base', 'exist(''wantedess'', ''var'')')
        wantedess = evalin('base','wantedess');
    else
        wantedess = 0.001;
    end

    %% Get reference
    if evalin('base', 'exist(''ref'', ''var'')')
        ref = evalin('base','ref');
    elseif evalin('base', 'exist(''step_amp'', ''var'')')
        ref = evalin('base','step_amp');
    else
        ref = 1;
    end

    %% Run simulation
    try
        simOut = sim(model, ...
            'StopTime', num2str(sim_time), ...
            'CaptureErrors', 'on');
    catch
        J = penalty;
        return
    end

    if ~isempty(simOut.ErrorMessage)
        J = penalty;
        return
    end

    %% Get response
    try
        resp = simOut.OutputResponse;
        t = resp.Time;
        y = resp.Data;
    catch
        J = penalty;
        return
    end

    %% Validate response
    if isempty(t) || isempty(y)
        J = penalty;
        return
    end

    if any(isnan(y)) || any(isinf(y)) || max(abs(y)) > 1e4
        J = penalty;
        return
    end

    %% Step response information
    try
        info = stepinfo(y, t, ref, 'SettlingTimeThreshold', 0.05);
    catch
        J = penalty;
        return
    end

    %% Actual performance
    actual_overshoot = info.Overshoot;

    if isempty(actual_overshoot) || isnan(actual_overshoot) || isinf(actual_overshoot)
        actual_overshoot = 100;
    end

    actual_risetime = info.RiseTime;
    if isempty(actual_risetime) || isnan(actual_risetime) || isinf(actual_risetime)
        actual_risetime = sim_time;
    end

    actual_settlingtime = info.SettlingTime;

    %% Manual fallback for SettlingTime if stepinfo fails
    if isempty(actual_settlingtime) || isnan(actual_settlingtime) || isinf(actual_settlingtime)
        band = 0.05 * abs(ref);

        if band == 0
            band = 0.05;
        end

        err = abs(y - ref);
        outsideIdx = find(err > band);

        if isempty(outsideIdx)
            actual_settlingtime = 0;
        elseif outsideIdx(end) < length(t)
            actual_settlingtime = t(outsideIdx(end) + 1);
        else
            actual_settlingtime = sim_time;
        end
    end

    actual_ess = abs(ref - y(end));

    %% Absolute errors to user targets
    e_overshoot    = abs(actual_overshoot    - wantedovershoot);
    e_risetime     = abs(actual_risetime     - wantedrisetime);
    e_settlingtime = abs(actual_settlingtime - wantedsettlingtime);
    e_ess          = abs(actual_ess          - wantedess);

    %% Normalization
    e_overshoot_norm    = e_overshoot    / max(1, wantedovershoot);
    e_risetime_norm     = e_risetime     / max(1e-3, wantedrisetime);
    e_settlingtime_norm = e_settlingtime / max(1e-3, wantedsettlingtime);
    e_ess_norm          = e_ess          / max(1e-6, wantedess);

    %% Weights
    w1 = 0.30;
    w2 = 0.20;
    w3 = 0.20;
    w4 = 0.30;

    %% Final cost
    J = w1 * e_overshoot_norm + ...
        w2 * e_risetime_norm + ...
        w3 * e_settlingtime_norm + ...
        w4 * e_ess_norm;

    %% Protection
    if isnan(J) || isinf(J)
        J = penalty;
    end

end