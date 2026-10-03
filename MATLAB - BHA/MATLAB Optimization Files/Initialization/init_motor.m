function plant = init_motor()

J = 0.02;
b = 2.2e-6;
K = 0.0115;
R = 2.18;
L = 2.3e-3;

step_amp = 50;

num = K;
den = [(J * L) ((J * R) + (L * b)) ((b * R) + K^2)];

plant.name = "DCMotorSpeed";
plant.num = num;
plant.den = den;
plant.step_amp = step_amp;
plant.G = tf(num, den);

end