function init_cruise()

m = 1000;
b = 50;

step_amp = 10;

num = 1;
den = [m b];

assignin('base','step_amp',step_amp);
assignin('base','num',num);
assignin('base','den',den);

end