%% Blink built-in LED on Arduino from MATLAB
% This script controls the built-in LED (usually on pin D13)

clear; clc;

% 1) Connect to Arduino
a = arduino();

% If MATLAB cannot detect the board automatically, specify the port and board:
% Example:
% a = arduino("COM3","Nano3");

% 2) Built-in LED pin (usually D13)
ledPin = "D13";

% 3) Blink settings
blinkCount = 3;   % Number of blinks
onTime  = 1;      % LED ON time (seconds)
offTime = 1;      % LED OFF time (seconds)

% 4) Blink loop
for k = 1:blinkCount
    writeDigitalPin(a, ledPin, 1);   % Turn LED ON
    pause(onTime);

    writeDigitalPin(a, ledPin, 0);   % Turn LED OFF
    pause(offTime);
end

% 5) Release the Arduino connection (recommended)
clear a;
