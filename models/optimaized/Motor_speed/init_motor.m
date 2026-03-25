function init_motor()

J = 0.01;
b = 0.1;
K = 0.01;
R = 1;
L = 0.5;

step_amp = 1;

num = K;
den = [(J*L) ((J*R)+(L*b)) ((b*R)+K^2)];

assignin('base','num',num);
assignin('base','den',den);
assignin('base','step_amp',step_amp);

end