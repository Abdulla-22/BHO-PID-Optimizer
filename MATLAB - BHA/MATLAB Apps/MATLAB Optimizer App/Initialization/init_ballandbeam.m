function plant = init_ballandbeam()

% Read parameters from base workspace
m = evalin('base','m');
R = evalin('base','R');
g = evalin('base','g');
L = evalin('base','L');
d = evalin('base','d');
J = evalin('base','J');
step_amp = evalin('base','step_amp');

% Build transfer function
K = (m * g * d) / (L * (J / R^2 + m));

num = [K];
den = [1 0 0];

% Return plant structure
plant.name = "BallandBeam";
plant.num = num;
plant.den = den;
plant.step_amp = step_amp;
plant.G = tf(num, den);

% Assign derived values to base workspace
assignin('base','num',num);
assignin('base','den',den);
assignin('base','ref',step_amp);

end