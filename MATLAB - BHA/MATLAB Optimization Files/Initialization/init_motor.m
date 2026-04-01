function init_motor()

J = 0.01;
b = 1e-6;
K = 0.1;
R = 2.18;
L = 2.3e-3;

step_amp = 50;

num = K;
den = [(J*L) ((J*R)+(L*b)) ((b*R)+K^2)];

assignin('base','num',num);
assignin('base','den',den);
assignin('base','step_amp',step_amp);

end