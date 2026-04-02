function J = cost_ballandbeam(K, model)
    % --- Controller gains assignment ---
    kp = K(1); ki = K(2); kd = K(3);
    assignin('base','kp',kp);
    assignin('base','ki',ki);
    assignin('base','kd',kd);

    % --- Initialize system parameters ---
    init_ballandbeam();

    % --- Fetch parameters and targets ---
    sim_time        = evalin('base', 'sim_time');
    wantedovershoot = evalin('base', 'wantedovershoot');
    wantedrisetime  = evalin('base', 'wantedrisetime');
    ref             = evalin('base', 'step_amp');

    % --- Execute Simulation ---
    try
        simOut = sim(model, 'StopTime', num2str(sim_time), 'CaptureErrors', 'on');
        t = simOut.OutputResponse.Time;
        y = simOut.OutputResponse.Data;
    catch
        J = 1e8; return;
    end

    % --- Performance Analysis ---
    if isempty(y) || isnan(y(end)), J = 1e8; return; end
    
    info = stepinfo(y, t, ref);
    actual_OS = info.Overshoot;
    actual_RT = info.RiseTime;
    
    if isnan(actual_OS) || isnan(actual_RT), J = 1e7; return; end

    % --- Normalized Errors ---
    % 1. IAE (Tracking accuracy)
    IAE = trapz(t, abs(ref - y));
    IAE_norm = IAE / (ref * sim_time);

    % 2. Overshoot Error (Normalized)
    E_OS = max(0, (actual_OS - wantedovershoot) / 100);

    % 3. Rise Time Error (Normalized)
    E_RT = max(0, (actual_RT - wantedrisetime) / sim_time);

    % --- Weighted Sum (Weights sum = 1) ---
    % For Ball & Beam, stability (IAE) and Overshoot are critical
    w1 = 0.6;   % High importance for IAE to keep the ball stable
    w2 = 0.3;   % Weight for Overshoot
    w3 = 0.1;   % Weight for Rise Time
    
    J = (w1 * IAE_norm) + (w2 * E_OS) + (w3 * E_RT);
    
    % Hard constraint penalty
    if actual_OS > wantedovershoot || actual_RT > wantedrisetime
        J = J * 20; % Stronger penalty for this unstable system
    end
end