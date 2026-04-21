function J = cost_motor(K, model)
    % --- Controller gains assignment ---
    kp = K(1);
    ki = K(2);
    kd = K(3);

    assignin('base','kp',kp);
    assignin('base','ki',ki);
    assignin('base','kd',kd);

    % --- Fetch parameters and targets from workspace ---
    sim_time          = evalin('base', 'sim_time');
    optimization_mode = evalin('base', 'optimization_mode');
    wantedovershoot   = evalin('base', 'wantedovershoot');
    wantedrisetime    = evalin('base', 'wantedrisetime');
    wantedess         = evalin('base', 'wantedess');   % Expected in %
    ref               = evalin('base', 'step_amp');

    % --- Execute Simulink Model ---
    try
        simOut = sim(model, 'StopTime', num2str(sim_time), 'CaptureErrors', 'on');
        t = simOut.OutputResponse.Time;
        y = simOut.OutputResponse.Data;
    catch
        J = 1e8;
        return;
    end

    % --- Performance Analysis ---
    if isempty(y) || isnan(y(end)) || isempty(t)
        J = 1e8;
        return;
    end

    try
        info = stepinfo(y, t, ref);
    catch
        J = 1e7;
        return;
    end

    actual_OS = info.Overshoot;
    actual_RT = info.RiseTime;

    if isnan(actual_OS) || isnan(actual_RT)
        J = 1e7;
        return;
    end

    % --- Common metrics ---
    error_signal = abs(ref - y);
    IAE = trapz(t, error_signal);

    if abs(ref) < 1e-12
        IAE_norm = IAE / max(sim_time, 1e-12);
        actual_ESS_percent = abs(y(end)) * 100;
    else
        IAE_norm = IAE / (abs(ref) * sim_time);
        actual_ESS = abs(ref - y(end));
        actual_ESS_percent = (actual_ESS / abs(ref)) * 100;
    end

    % --- BEST mode ---
    if optimization_mode == 1
        E_OS  = actual_OS / 100;
        E_RT  = actual_RT / max(sim_time, 1e-12);
        E_ESS = actual_ESS_percent / 100;

        w1 = 0.10;
        w2 = 0.35;
        w3 = 0.35;
        w4 = 0.20;

        J = (w1 * IAE_norm) + (w2 * E_OS) + (w3 * E_RT) + (w4 * E_ESS);
        return;
    end

    % --- CONSTRAINED mode ---
    % This mode chases the requested values, not only maximum limits
    E_OS  = abs(actual_OS - wantedovershoot) / max(wantedovershoot, 1e-6);
    E_RT  = abs(actual_RT - wantedrisetime)  / max(wantedrisetime,  1e-6);
    E_ESS = abs(actual_ESS_percent - wantedess) / max(wantedess, 1e-6);

    w1 = 0.10;   % Tracking
    w2 = 0.35;   % Overshoot target matching
    w3 = 0.35;   % Rise time target matching
    w4 = 0.20;   % ESS target matching

    J = (w1 * IAE_norm) + (w2 * E_OS) + (w3 * E_RT) + (w4 * E_ESS);

    % Extra penalty for violating requested limits
    if actual_OS > wantedovershoot
        J = J + 5 * ((actual_OS - wantedovershoot) / max(wantedovershoot, 1e-6));
    end

    if actual_RT > wantedrisetime
        J = J + 5 * ((actual_RT - wantedrisetime) / max(wantedrisetime, 1e-6));
    end

    if actual_ESS_percent > wantedess
        J = J + 5 * ((actual_ESS_percent - wantedess) / max(wantedess, 1e-6));
    end
end