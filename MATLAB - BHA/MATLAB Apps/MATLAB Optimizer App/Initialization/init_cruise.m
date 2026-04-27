function plant = init_cruise()

% Read parameters from base workspace
m = evalin('base','m');
b = evalin('base','b');
step_amp = evalin('base','step_amp');

% Build transfer function
num = 1;
den = [m b];

% Return plant structure
plant.name = "CruiseControl";
plant.num = num;
plant.den = den;
plant.step_amp = step_amp;
plant.G = tf(num, den);

% Assign derived values to base workspace
assignin('base','num',num);
assignin('base','den',den);
assignin('base','ref',step_amp);

end