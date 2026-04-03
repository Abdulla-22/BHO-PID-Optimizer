function J = cost_motor(K, model)
    % --- Controller gains assignment ---
    kp = K(1); 
    ki = K(2); 
    kd = K(3);
    
    assignin('base','kp',kp);
    assignin('base','ki',ki);
    assignin('base','kd',kd);

    % --- Fetch parameters and targets from workspace ---
    sim_time        = evalin('base', 'sim_time');
    wantedovershoot = evalin('base', 'wantedovershoot');
    wantedrisetime  = evalin('base', 'wantedrisetime');
    wantedess       = evalin('base', 'wantedess');   % Expected in %
    ref             = evalin('base', 'step_amp');

    % --- Execute Simulink Model ---
    try
        simOut = sim(model, 'StopTime', num2str(sim_time), 'CaptureErrors', 'on');
        t = simOut.OutputResponse.Time;
        y = simOut.OutputResponse.Data;
    catch
        J = 1e8; % High penalty for simulation crash
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
    
    % Check for instability
    if isnan(actual_OS) || isnan(actual_RT)
        J = 1e7;
        return;
    end

    % --- Calculate Errors (Normalized) ---
    % 1. IAE (Integral Absolute Error)
    error_signal = abs(ref - y);
    IAE = trapz(t, error_signal); 
    IAE_norm = IAE / (ref * sim_time);

    % 2. Overshoot Error
    E_OS = max(0, (actual_OS - wantedovershoot) / 100);

    % 3. Rise Time Error
    E_RT = max(0, (actual_RT - wantedrisetime) / sim_time);

    % 4. Steady-State Error in percentage
    actual_ESS = abs(ref - y(end));
    actual_ESS_percent = (actual_ESS / abs(ref)) * 100;

    % Penalty only if ESS exceeds user target
    E_ESS = max(0, (actual_ESS_percent - wantedess) / 100);

    % --- Weighted Sum (Weights sum = 1) ---
    w1 = 0.4;   % Weight for tracking accuracy (IAE)
    w2 = 0.3;   % Weight for Overshoot constraint
    w3 = 0.2;   % Weight for Rise Time constraint
    w4 = 0.1;   % Weight for Steady-State Error constraint
    
    % Total Cost Function
    J = (w1 * IAE_norm) + (w2 * E_OS) + (w3 * E_RT) + (w4 * E_ESS);
    
    % Add a severe multiplier if requirements are not met
    if actual_OS > wantedovershoot || ...
       actual_RT > wantedrisetime || ...
       actual_ESS_percent > wantedess
        J = J * 10; 
    end
end