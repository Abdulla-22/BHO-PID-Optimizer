function J = cost_custom(K, model)

    %% Controller gains
    kp = K(1);
    ki = K(2);
    kd = K(3);

    assignin('base','kp',kp);
    assignin('base','ki',ki);
    assignin('base','kd',kd);

    %% Get simulation settings and targets
    sim_time        = evalin('base','sim_time');
    wantedovershoot = evalin('base','wantedovershoot');
    wantedrisetime  = evalin('base','wantedrisetime');
    ref             = evalin('base','step_amp');

    %% Run simulation
    try
        simOut = sim(model, 'StopTime', num2str(sim_time), 'CaptureErrors', 'on');

        if ~isempty(simOut.ErrorMessage)
            J = 1e8;
            return;
        end

        t = simOut.OutputResponse.Time;
        y = simOut.OutputResponse.Data;

    catch
        J = 1e8;
        return;
    end

    %% Basic checks
    if isempty(t) || isempty(y) || any(isnan(y)) || any(isinf(y))
        J = 1e8;
        return;
    end

    %% Step response info
    try
        info = stepinfo(y, t, ref, 'SettlingTimeThreshold', 0.02);
    catch
        try
            info = stepinfo(y, t, ref);
        catch
            J = 1e7;
            return;
        end
    end

    actual_OS = info.Overshoot;
    actual_RT = info.RiseTime;

    if isnan(actual_OS) || isnan(actual_RT)
        J = 1e7;
        return;
    end

    %% Tracking error
    IAE = trapz(t, abs(ref - y));
    IAE_norm = IAE / (abs(ref) * sim_time + eps);

    %% Constraint errors
    E_OS = max(0, (actual_OS - wantedovershoot) / 100);
    E_RT = max(0, (actual_RT - wantedrisetime) / sim_time);

    %% Weighted cost
    w1 = 0.5;
    w2 = 0.3;
    w3 = 0.2;

    J = (w1 * IAE_norm) + (w2 * E_OS) + (w3 * E_RT);

    %% Penalty if user specs are violated
    if actual_OS > wantedovershoot || actual_RT > wantedrisetime
        J = J * 10;
    end

end