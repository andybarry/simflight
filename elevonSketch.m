
clear

command_us_left = [
  1509
  1559
  1609
  1659
  1709
  1759
  1809
  1859
  1909
  1459
  1409
  1359
  1309
  1259
  1209
  1159
  %1109
  ];


degrees_left = [
  6.9
  12.5
  18.5
  26.5
  32.0
  40.0
  47.7
  57.0
  64.5
  2.0
  -2.5
  -9.0
  -13.5
  -19.5
  -22.9
  -27.0
  %-27.0
  ];

radians_left = deg2rad(degrees_left);


figure(1)
clf
plot(command_us_left, radians_left, '*');
% 
% Linear model Poly1:
%      f(x) = p1*x + p2
% Coefficients (with 95% confidence bounds):
%        p1 =    0.002114  (0.001982, 0.002246)
%        p2 =      -3.011  (-3.216, -2.806)
% 
% Goodness of fit:
%   SSE: 0.0451
%   R-square: 0.9883
%   Adjusted R-square: 0.9874
%   RMSE: 0.05676
% 

fit_us = 1100:1950;
fit_left = 0.002114 * fit_us -3.011; % min = 1150, max = 1909

hold on
plot(fit_us, fit_left, 'r-')
legend('Data', 'Fit','Location','NorthWest');
xlabel('Command (us)');
ylabel('Deflection (rad)');

command_us_right = [
  1609
  1559
  1509
  1459
  1409
  1359
  1309
  1259
  1209
  1659
  1709
  1759
  1809
  %1859
  %1909
  ];


degrees_right = [
  0
  4.5
  9.5
  16.6
  21.5
  33.0
  39.1
  48.2
  54.3
  -7.4
  -13.2
  -20.0
  -24.5
  %-25.5
  %-25.3
  ];

radians_right = deg2rad(degrees_right);

figure(2)
clf
plot(command_us_right, radians_right, '*');

% 
% Linear model Poly1:
%      f(x) = p1*x + p2
% Coefficients (with 95% confidence bounds):
%        p1 =     -0.0023  (-0.002428, -0.002172)
%        p2 =       3.688  (3.494, 3.882)
% 
% Goodness of fit:
%   SSE: 0.01688
%   R-square: 0.993
%   Adjusted R-square: 0.9924
%   RMSE: 0.03918



fit_right = -0.0023 * fit_us + 3.688; % min = 1209, max = 1809

hold on
plot(fit_us, fit_right, 'r-');
legend('Data', 'Fit');

xlabel('Command (us)');
ylabel('Deflection (rad)');
