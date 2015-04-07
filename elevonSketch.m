
clear

%{
%% aircraft #1

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

%}

%% aircraft #2

command_us_left2 = [
  1377
  1327
  1276
  1226
  1176
  1125
  1428
  1478
  1528
  1579
  1629
  1673
  1724
  ];


degrees_left2 = [
  0
  -6.4
  -13.9
  -19.0
  -26.0
  -27.6
  6.0
  11.0
  18.4
  27.5
  30.5
  39.7
  44.7
  ];


radians_left2 = deg2rad(degrees_left2);

figure(1)
clf

plot(command_us_left2, radians_left2, '*');
xlabel('Command (us)');
ylabel('Deflection (rad)');
title('Airplane #2, Elevon Left');

grid on

% -- cftool --
% Linear model Poly1:
%      f(x) = p1*x + p2
% Coefficients (with 95% confidence bounds):
%        p1 =    0.002195  (0.002095, 0.002295)
%        p2 =      -3.017  (-3.161, -2.873)
% 
% Goodness of fit:
%   SSE: 0.01038
%   R-square: 0.9953
%   Adjusted R-square: 0.9949
%   RMSE: 0.03073

% 


fit_us2 = 1100:1950;
fit_left2 = 0.002195 * fit_us2 - 3.017;
hold on
plot(fit_us2, fit_left2, 'r-');
legend('Data', 'Linear Fit', 'Location', 'NorthWest');

















command_us_right2 = [
  
  1470
  1520
  1569
  1619
  1669
  1718
  1420
  1371
  1321
  1272
  1222
  1169
  ];


degrees_right2 = [
  0
  -2.9
  -10.6
  -16.9
  -19.5
  -24.6
  5.9
  11.5
  15.3
  22.6
  27.6
  36.3
  ];

radians_right2 = deg2rad(degrees_right2);

figure(2)
clf

plot(command_us_right2, radians_right2, '*');
xlabel('Command (us)');
ylabel('Deflection (rad)');
title('Airplane #2, Elevon Right');

grid on

% -- cftool --
% Linear model Poly1:
%      f(x) = p1*x + p2
% Coefficients (with 95% confidence bounds):
%        p1 =     -0.0019  (-0.001995, -0.001806)
%        p2 =       2.811  (2.674, 2.948)
% 
% Goodness of fit:
%   SSE: 0.006342
%   R-square: 0.9951
%   Adjusted R-square: 0.9946
%   RMSE: 0.02518

fit_us2 = 1100:1950;
fit_right2 = -0.0019 * fit_us2 + 2.811; % min = 1209, max = 1809
hold on
plot(fit_us2, fit_right2, 'r-');
%legend('Data', 'Linear Fit');

