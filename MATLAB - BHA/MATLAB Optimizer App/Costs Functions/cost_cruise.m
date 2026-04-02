function J = cost_cruise(K, model)
    % --- Controller gains assignment ---
    kp = K(1); ki = K(2); kd = K(3);
    assignin('base','kp',kp);
    assignin('base','ki',ki);
    assignin('base','kd',kd);

    % --- Initialize system parameters ---
    init_cruise();

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
    % 1. IAE 
    IAE = trapz(t, abs(ref - y));
    IAE_norm = IAE / (ref * sim_time);

    % 2. Overshoot Error
    E_OS = max(0, (actual_OS - wantedovershoot) / 100);

    % 3. Rise Time Error
    E_RT = max(0, (actual_RT - wantedrisetime) / sim_time);

    % --- Weighted Sum (Weights sum = 1) ---
    % Cruise Control usually prioritizes comfort (Low OS) and steady speed (IAE)
    w1 = 0.5;   % Accuracy
    w2 = 0.3;   % Comfort (Overshoot)
    w3 = 0.2;   % Responsiveness (Rise Time)
    
    J = (w1 * IAE_norm) + (w2 * E_OS) + (w3 * E_RT);
    
    % Penalty for violating user specs
    if actual_OS > wantedovershoot || actual_RT > wantedrisetime
        J = J * 10;
    end
end