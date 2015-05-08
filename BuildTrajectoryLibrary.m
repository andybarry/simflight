
clear
%%


% bounds = [ 2
%   .01
%   .01
%   .5
%   .5
%   .5
%     .5
%     3
%     3
%   1
%   1
%   1];
%{
bounds = [ 10
  .5
  1
  .1
  .01
  .5
    3
    5
    5
  deg2rad(70)
  deg2rad(70)
  deg2rad(70)];

u0 = [0, 0, 5.33/2];


tf_straight = 1;

xf_straight = [15 * tf_straight
         0
   1
         0
    -0.1857
         0
   15.0805
         0
   0
         0
    0
         0];
       
% constrain the plane to be knife-edged
bounds_roll_lower= -Inf*ones(12,1);
bounds_roll_upper = Inf*ones(12,1);


bounds_roll_lower(4) = pi/2 - .1;
bounds_roll_upper(4) = pi/2 + .1;
c_knife = BoundingBoxConstraint(bounds_roll_lower, bounds_roll_upper);

cons = struct();
cons.c = c_knife;
cons.N_fac = 0.5;



[utraj_knife, xtraj_knife] = runDircol(xf_straight, tf_straight, bounds, u0, cons);
%}
%% alieron roll

bounds = [ 100
  20
  20
  .1
  .01
  .5
    3
    5
    5
  deg2rad(70)
  deg2rad(70)
  deg2rad(70)];

u0 = [0, 0, 5.33/2];


tf_straight = 4;

xf_straight = [14 * tf_straight
         0
   1
         0
    -0.1857
         0
   15.0805
         0
   0
         0
    0
         0];
       
% constrain the plane to be knife-edged
bounds_roll_lower = -Inf*ones(12,1);
bounds_roll_upper = Inf*ones(12,1);


bounds_roll_lower(4) = rad2deg(160);
bounds_roll_upper(4) = rad2deg(170)
c_knife = BoundingBoxConstraint(bounds_roll_lower, bounds_roll_upper);


bounds_roll_lower = -Inf*ones(12,1);
bounds_roll_upper = Inf*ones(12,1);

bounds_roll_lower(4) = rad2deg(190);
bounds_roll_upper(4) = rad2deg(200);
c_knife2 = BoundingBoxConstraint(bounds_roll_lower, bounds_roll_upper);

cons = struct();
cons.c = c_knife;
cons.N_fac = 0.45;

cons.c(2) = c_knife2;
cons.N_fac(2) = 0.55;



[utraj_knife, xtraj_knife] = runDircol(xf_straight, tf_straight, bounds, u0, cons);
return;

%%


bounds = [ 10
  .5
  1
  .5
  .5
  .5
    3
    5
    5
  deg2rad(70)
  deg2rad(70)
  deg2rad(70)];
%{


xf_left = [10
         3
   2
         0
    0.1857
         0
   15.0805
         2
   0
         deg2rad(30)
    0
         0];
       
tf_left = 0.5;

[left_utrag, left_xtraj] = runDircol(xf_left, tf_left, bounds, u0);

%}

xf_right = [5
         -2.5
   2
         0
    0.1857
         0
   15.0805
         -2
   0
         -deg2rad(30)
    0
         0];
       
tf_right = 0.5;

[right_utraj, right_xtraj] = runDircol(xf_right, tf_right, bounds, u0);
