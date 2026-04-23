function init_cruise()

% Read parameters from base workspace (user input from app)
m = evalin('base','m');
b = evalin('base','b');

% Build transfer function using user-defined values
num = 1;
den = [m b];

% Assign only derived values
assignin('base','num',num);
assignin('base','den',den);

end