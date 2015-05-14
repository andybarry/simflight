
megaclear

%% trim conditions

parameters = {0.904, 0.000, -0.134, -0.049, 0 };
gains = GetDefaultGains();

p = DeltawingPlant(parameters);

[x0, u0, lib] = FindTrimDrake(p);


%% knife-edge


bounds = [ 
  10          % x
  0.5         % y
  1.0         % z
  deg2rad(5)  % roll
  deg2rad(10) % pitch
  deg2rad(28) % yaw
  3           % x-dot
  5           % y-dot
  5           % z-dot
  deg2rad(70) % roll-dot
  deg2rad(70) % pitch-dot
  deg2rad(70) % yaw-dot
  ];


tf_knife_edge = 1.75;

xf_knife_edge = x0;
xf_knife_edge(1) = x0(7)*tf_knife_edge;
       
% constrain the plane to be knife-edged
bounds_roll_lower= -Inf*ones(12,1);
bounds_roll_upper = Inf*ones(12,1);


bounds_roll_lower(4) = pi/2 - .1;
bounds_roll_upper(4) = pi/2 + .1;
c_knife = BoundingBoxConstraint(bounds_roll_lower, bounds_roll_upper);

cons = struct();
cons.c = c_knife;
cons.N_fac = 0.5;



[utraj_knife1, xtraj_knife1] = runDircol(parameters, x0, xf_knife_edge, tf_knife_edge, bounds, u0, cons, 11);
[utraj_knife2, xtraj_knife2] = runDircol(parameters, x0, xf_knife_edge, xtraj_knife1.tspan(2), bounds, u0, cons, 31, utraj_knife1, xtraj_knife1);

% stabilize the trajectory with TVLQR

lib = AddLqrControllersToLib('knife-edge', lib, p, xtraj_knife2, utraj_knife2, gains);


%% alieron roll

bounds = [ 
  100         % x
  20          % y
  20          % z
  deg2rad(5)  % roll
  deg2rad(50) % pitch
  0.5         % yaw
  3           % x-dot
  5           % y-dot
  100         % z-dot
  deg2rad(70) % roll-dot
  deg2rad(70) % pitch-dot
  deg2rad(70) % yaw-dot
  ];


tf_roll = 4;
xf_roll = x0;
xf_roll(1) = x0(7)*tf_roll;


[utraj_roll1, xtraj_roll1] = runDircol(parameters, x0, xf_roll, tf_roll, bounds, u0, [], 11);
[utraj_roll2, xtraj_roll2] = runDircol(parameters, x0, xf_roll, xtraj_roll1.tspan(2), bounds, u0, [], 31, utraj_roll1, xtraj_roll1);

lib = AddLqrControllersToLib('knife-edge', lib, p, xtraj_roll2, utraj_roll2, gains);
