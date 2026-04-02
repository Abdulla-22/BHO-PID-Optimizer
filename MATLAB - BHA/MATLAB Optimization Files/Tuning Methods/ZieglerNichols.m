function [Kp, Ki, Kd] = ZieglerNichols(Ku, Tu, type)
% ZieglerNichols - Ziegler-Nichols tuning for PI / PD / PID

switch upper(type)
    
    case 'PI'
        Kp = 0.45 * Ku;
        Ki = 1.2 * Kp / Tu;
        Kd = 0;

    case 'PD'
        Kp = 0.8 * Ku;
        Ki = 0;
        Kd = Kp * Tu / 8;

    case 'PID'
        Kp = 0.6 * Ku;
        Ki = 1.2 * Ku / Tu;
        Kd = 0.075 * Ku * Tu;

    otherwise
        error('Unknown controller type: PI, PD, PID only');
end
end