function init_motor()

% Read all required parameters from base workspace
J = evalin('base', 'J');
b = evalin('base', 'b');
K = evalin('base', 'K');
R = evalin('base', 'R');
L = evalin('base', 'L');

% Build transfer function coefficients using user-defined values
num = K;
den = [(J * L) ((J * R) + (L * b)) ((b * R) + K^2)];

% Assign derived values only
assignin('base', 'num', num);
assignin('base', 'den', den);

end