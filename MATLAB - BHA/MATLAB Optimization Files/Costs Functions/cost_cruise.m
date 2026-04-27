function J = cost_cruise(K, plant, sim_time, optimization_mode, wantedovershoot, wantedrisetime, wantedess)

kp = K(1);
ki = K(2);
kd = K(3);

assignin('base','kp',kp);
assignin('base','ki',ki);
assignin('base','kd',kd);

ref = plant.step_amp;

try
    [t, y] = simulateLocalResponse(K, plant, sim_time);
catch
    J = 1e8;
    return;
end

if isempty(y) || isempty(t) || isnan(y(end)) || any(isnan(y)) || any(isinf(y))
    J = 1e8;
    return;
end

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

IAE = trapz(t, abs(ref - y));

if abs(ref) < 1e-12
    IAE_norm = IAE / max(sim_time, 1e-12);
    actual_ESS_percent = abs(y(end)) * 100;
else
    IAE_norm = IAE / (abs(ref) * sim_time);
    actual_ESS = abs(ref - y(end));
    actual_ESS_percent = (actual_ESS / abs(ref)) * 100;
end

if optimization_mode == 1
    E_OS  = actual_OS / 100;
    E_RT  = actual_RT / max(sim_time, 1e-12);
    E_ESS = actual_ESS_percent / 100;

    w1 = 0.15;
    w2 = 0.30;
    w3 = 0.25;
    w4 = 0.30;

    J = (w1 * IAE_norm) + (w2 * E_OS) + (w3 * E_RT) + (w4 * E_ESS);
    return;
end

E_OS  = abs(actual_OS - wantedovershoot) / max(wantedovershoot, 1e-6);
E_RT  = abs(actual_RT - wantedrisetime) / max(wantedrisetime, 1e-6);
E_ESS = abs(actual_ESS_percent - wantedess) / max(wantedess, 1e-6);

w1 = 0.15;
w2 = 0.30;
w3 = 0.25;
w4 = 0.30;

J = (w1 * IAE_norm) + (w2 * E_OS) + (w3 * E_RT) + (w4 * E_ESS);

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

function [t, y] = simulateLocalResponse(K, plant, sim_time)

kp = K(1);
ki = K(2);
kd = K(3);

C = pid(kp, ki, kd);
closedLoopSys = feedback(C * plant.G, 1);

t = linspace(0, sim_time, 1000);
[y, t] = step(plant.step_amp * closedLoopSys, t);

t = t(:);
y = y(:);

end