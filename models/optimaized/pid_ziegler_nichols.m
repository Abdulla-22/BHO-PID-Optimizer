function [Kp, Ki, Kd] = pid_ziegler_nichols(Ku, Tu, method)
% PID_ZIEGLER_NICHOLS - Compute PID gains using Ziegler-Nichols tuning
%
% Syntax:
%   [Kp, Ki, Kd] = pid_ziegler_nichols(Ku, Tu)
%   [Kp, Ki, Kd] = pid_ziegler_nichols(Ku, Tu, method)
%
% Inputs:
%   Ku     - Ultimate gain (system starts sustained oscillation)
%   Tu     - Oscillation period at Ku
%   method - 'classic' (default) or 'modified' Ziegler-Nichols rules
%
% Outputs:
%   Kp, Ki, Kd - PID controller gains

if nargin < 3
    method = 'classic';
end

switch lower(method)
    case 'classic'
        Kp = 0.6 * Ku;
        Ki = 1.2 * Ku / Tu;
        Kd = 0.075 * Ku * Tu;
    case 'pessen'
        Kp = 0.7 * Ku;
        Ki = 1.0 * Ku / Tu;
        Kd = 0.125 * Ku * Tu;
    otherwise
        error('Unknown method. Choose ''classic'' or ''pessen''.');
end
end