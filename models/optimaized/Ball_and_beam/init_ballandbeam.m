function init_ballandbeam()

% Parameters
m = 0.11;
R = 0.015;
g = 9.81;
L = 1;
d = 0.03;

ref = 0.25;

step_amp = 1;

num = [g];
den = [1 0 0];

assignin('base','m',m);
assignin('base','R',R);
assignin('base','g',g);
assignin('base','L',L);
assignin('base','d',d);

assignin('base','ref',ref);
assignin('base','step_amp',step_amp);
assignin('base','num',num);
assignin('base','den',den);

end