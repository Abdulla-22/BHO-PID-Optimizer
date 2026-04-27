function plant = init_cruise()

m = 1000;
b = 50;

step_amp = 10;

num = 1;
den = [m b];

plant.name = "CruiseControl";
plant.num = num;
plant.den = den;
plant.step_amp = step_amp;
plant.G = tf(num, den);

end