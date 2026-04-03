function init_ballandbeam()

% Read parameters from base workspace (user input from app)
m = evalin('base','m');
R = evalin('base','R');
g = evalin('base','g');
L = evalin('base','L');
d = evalin('base','d');
J = evalin('base','J');

% Build transfer function using user-defined values
K = (m * g * d) / (L * (J / R^2 + m));

num = [K];
den = [1 0 0];

% Assign only derived values
assignin('base','num',num);
assignin('base','den',den);

end