
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

thurst = [0
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
560];

plot(throttlePercent, thurst,'x-');

scaledThrottlePercent = (throttlePercent + 80) /1.4


% >> cftool
% Linear model Poly1:
%      f(x) = p1*x + p2
% Coefficients (with 95% confidence bounds):
%        p1 =       5.265  (4.796, 5.734)
%        p2 =      0.4265  (-25.53, 26.38)
% 
% Goodness of fit:
%   SSE: 1.513e+04
%   R-square: 0.9745
%   Adjusted R-square: 0.9728
%   RMSE: 31.75
% 
