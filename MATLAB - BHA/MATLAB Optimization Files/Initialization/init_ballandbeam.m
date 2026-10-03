function plant = init_ballandbeam()

% Parameters
m = 0.111;
R = 0.015;
g = 9.8;
L = 1.0;
d = 0.03;
J = 9.99e-6;

step_amp = 10;

K = (m * g * d) / (L * (J / R^2 + m));

num = [K];
den = [1 0 0];

plant.name = "BallandBeam";
plant.num = num;
plant.den = den;
plant.step_amp = step_amp;
plant.G = tf(num, den);

end