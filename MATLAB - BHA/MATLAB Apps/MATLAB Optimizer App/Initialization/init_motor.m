function plant = init_motor()

% Read parameters from base workspace
J = evalin('base','J');
b = evalin('base','b');
K = evalin('base','K');
R = evalin('base','R');
L = evalin('base','L');
step_amp = evalin('base','step_amp');

% Build transfer function
num = K;
den = [(J * L) ((J * R) + (L * b)) ((b * R) + K^2)];

% Return plant structure
plant.name = "DCMotorSpeed";
plant.num = num;
plant.den = den;
plant.step_amp = step_amp;
plant.G = tf(num, den);

% Assign derived values to base workspace
assignin('base','num',num);
assignin('base','den',den);
assignin('base','ref',step_amp);

end