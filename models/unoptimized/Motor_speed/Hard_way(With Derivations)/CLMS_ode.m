clc
clear
close all

%% System parameters
m = 1000;
b = 50;

%% Controller parameters
Vref = 120;
Kp = 250;
Ki = 10;
Kd = 0;   % Derivative gain

%% Initial conditions
% x1 = position
% x2 = velocity
% x3 = integral of error
x0 = [0;0;0];

%% Simulation
tspan = [0 120];

[t,x] = ode23(@(t,x) OCC_PID(t,x,m,b,Kp,Ki,Kd,Vref), tspan, x0);

y = x(:,2);   % velocity (system output)

%% Plot
figure
plot(t,y,'LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('Velocity')
title('Closed Loop Response (PID Control)')

%% Step response specifications
info = stepinfo(y,t,Vref);

fprintf('\n=== Closed Loop Specifications ===\n')
fprintf('Rise Time      = %.4f s\n',info.RiseTime)
fprintf('Settling Time  = %.4f s\n',info.SettlingTime)
fprintf('Overshoot      = %.2f %%\n',info.Overshoot)
fprintf('Peak           = %.4f\n',info.Peak)
fprintf('Peak Time      = %.4f s\n',info.PeakTime)

%% System dynamics function
function dx = OCC_PID(~,x,m,b,Kp,Ki,Kd,Vref)

err = Vref - x(2);

% PID control law with derivative on error
% edot = -x2_dot, and x2_dot depends on u
% Solve algebraically to avoid implicit loop
u = (Kp*err + Ki*x(3) + Kd*(b/m)*x(2)) / (1 + Kd/m);

dx = zeros(3,1);

dx(1) = x(2);
dx(2) = -b/m*x(2) + u/m;
dx(3) = err;

end