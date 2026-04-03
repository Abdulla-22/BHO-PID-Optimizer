function J = cost_ballandbeam(K, model)
    % --- Controller gains assignment ---
    kp = K(1);
    ki = K(2);
    kd = K(3);

    assignin('base','kp',kp);
    assignin('base','ki',ki);
    assignin('base','kd',kd);

    % --- Initialize system parameters ---
    init_ballandbeam();

    % --- Fetch parameters and targets ---
    sim_time        = evalin('base', 'sim_time');
    wantedovershoot = evalin('base', 'wantedovershoot');
    wantedrisetime  = evalin('base', 'wantedrisetime');
    wantedess       = evalin('base', 'wantedess');   % Expected in %
    ref             = evalin('base', 'step_amp');

    % --- Execute Simulation ---
    try
        simOut = sim(model, 'StopTime', num2str(sim_time), 'CaptureErrors', 'on');
        t = simOut.OutputResponse.Time;
        y = simOut.OutputResponse.Data;
    catch
        J = 1e8;
        return;
    end

    % --- Performance Analysis ---
    if isempty(y) || isnan(y(end))
        J = 1e8;
        return;
    end

    info = stepinfo(y, t, ref);
    actual_OS = info.Overshoot;
    actual_RT = info.RiseTime;

    if isnan(actual_OS) || isnan(actual_RT)
        J = 1e7;
        return;
    end

    % --- Normalized Errors ---
    % 1. IAE (Tracking accuracy)
    IAE = trapz(t, abs(ref - y));
    IAE_norm = IAE / (ref * sim_time);

    % 2. Overshoot Error
    E_OS = max(0, (actual_OS - wantedovershoot) / 100);

    % 3. Rise Time Error
    E_RT = max(0, (actual_RT - wantedrisetime) / sim_time);

    % 4. Steady-State Error in percentage
    actual_ESS = abs(ref - y(end));
    actual_ESS_percent = (actual_ESS / abs(ref)) * 100;
    E_ESS = max(0, (actual_ESS_percent - wantedess) / 100);

    % --- Weighted Sum (Weights sum = 1) ---
    % For Ball & Beam, stability and low overshoot are more critical
    w1 = 0.5;   % High importance for IAE
    w2 = 0.3;   % Overshoot
    w3 = 0.1;   % Rise Time
    w4 = 0.1;   % Steady-State Error

    J = (w1 * IAE_norm) + (w2 * E_OS) + (w3 * E_RT) + (w4 * E_ESS);

    % --- Hard constraint penalty ---
    if actual_OS > wantedovershoot || ...
       actual_RT > wantedrisetime || ...
       actual_ESS_percent > wantedess
        J = J * 20; % Stronger penalty for this unstable system
    end
end