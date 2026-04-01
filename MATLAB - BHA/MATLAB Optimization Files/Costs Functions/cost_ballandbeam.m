function J = cost_ballandbeam(K, model)

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

    %% Simulation
    try
        simOut = sim(model, 'StopTime', num2str(sim_time));
    catch
        J = 1e6;
        return
    end

    %% Response
    resp = simOut.OutputResponse;
    t = resp.Time;
    y = resp.Data;

    if isempty(t) || isempty(y) || any(isnan(y)) || any(isinf(y))
        J = 1e6;
        return
    end

    %% Reference
    if evalin('base', 'exist(''step_amp'', ''var'')')
        ref = evalin('base','step_amp');
    elseif evalin('base', 'exist(''ref'', ''var'')')
        ref = evalin('base','ref');
    else
        ref = 1;
    end

    %% Step info
    info = stepinfo(y, t, ref);

    if isnan(info.SettlingTime) || isnan(info.RiseTime) || isnan(info.Overshoot)
        J = 1e6;
        return
    end

    %% Errors
    actual_overshoot    = info.Overshoot;
    actual_risetime     = info.RiseTime;
    actual_settlingtime = info.SettlingTime;
    actual_ess          = abs(ref - y(end));

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
    w1 = 0.60;
    w2 = 0.10;
    w3 = 0.10;
    w4 = 0.20;

    %% Final cost
    J = w1 * e_overshoot_norm + ...
        w2 * e_risetime_norm + ...
        w3 * e_settlingtime_norm + ...
        w4 * e_ess_norm;

    if isnan(J) || isinf(J)
        J = 1e6;
    end

end