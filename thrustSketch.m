
throttlePercent = [-100
-90
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

thrust = [0
0
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

thrust_N = thrust * .0098;

plot(throttlePercent, thrust,'x-');

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
