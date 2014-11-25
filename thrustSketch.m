
throttlePercent = [%-100
%-90
-80
-70
-60
-50
-40
-30
-20
-10
0
10
20
30
40
50
60];

thrust = [%0
%0
0
25
56
91
122
160
196
230
278
335
375
420
467
528
560]; % in grams

command_us = [%1102
  %1142 % this one is made up
  1181
  1221
  1263
  1303
  1344
  1382
  1422
  1461
  1504
  1542
  1584
  1622
  1661
  1704
  1744];




thrust_N = thrust * .0098;

figure(1)
clf
plot(throttlePercent, thrust,'x-');

figure(2)
clf
plot(throttlePercent, command_us, 'rx-');

% Linear model Poly1:
%      f(x) = p1*x + p2
% Coefficients (with 95% confidence bounds):
%        p1 =     0.01004  (0.009485, 0.01059)
%        p2 =      -12.17  (-12.98, -11.36)
% 
% Goodness of fit:
%   SSE: 0.3829
%   R-square: 0.9916
%   Adjusted R-square: 0.991
%   RMSE: 0.1716

% linear fit: 0.01004 * command_us - 12.17, with min = 1212, max = 1744

usToThrust = 0.01004 * command_us - 12.17;

figure(3)
clf
plot(command_us, thrust_N, 'x-');
hold on
plot(command_us, usToThrust, 'rx-');
xlabel('Command us');
ylabel('Thrust (N)');
legend('Data', 'Fit');


scaledThrottlePercent = (throttlePercent + 80) /1.4



% >> cftool
% Linear model Poly1:
%      f(x) = p1*x + p2
% Coefficients (with 95% confidence bounds):
%        p1 =     0.05159  (0.047, 0.05619)
%        p2 =    0.004179  (-0.2502, 0.2585)
% 
% Goodness of fit:
%   SSE: 1.453
%   R-square: 0.9745
%   Adjusted R-square: 0.9728
%   RMSE: 0.3112
% 
