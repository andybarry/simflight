
megaclear

%% trim conditions


[parameters, gains] = GetDefaultGains();

deltawing_plant = DeltawingPlant(parameters);
p = DeltawingPlantStateEstFrame(deltawing_plant);


[x0, u0, lib] = FindTrimDrake(p);

%% more trim conditions

% climb

max_climb = 1.0; % m/s
[~, ~, lib] = FindClimbTrimDrake(p, max_climb, lib);

% aggressive left turn
[~, ~, lib] = FindLeftTrimDrake(p, deg2rad(-55), 0, 12, lib, 'TI-aggressive-left-turn2');

% agressive right turn
[~, ~, lib] = FindLeftTrimDrake(p, deg2rad(55), 0, 12, lib, 'TI-aggressive-right-turn2');


%% left turn 45 deg
bounds = [ 
  100         % x
  100         % y
  50          % z
  deg2rad(10)  % roll
  deg2rad(20) % pitch
  deg2rad(10) % yaw
  30           % x-dot
  30           % y-dot
  5           % z-dot
  deg2rad(70) % roll-dot
  deg2rad(70) % pitch-dot
  deg2rad(70) % yaw-dot
  ];


tf_turn = 2;

xf_turn = x0;
xf_turn(1) = x0(7)*.5*tf_turn;
xf_turn(2) = x0(7)*.5*tf_turn;
xf_turn(6) = deg2rad(45);

% compute final velocities

% input: velocity, body-x, final rpy
% output: velocity global x, y, and z
rpy_model = ConvertToModelFrameFromDrakeWorldFrame([zeros(3,1); xf_turn(4:6); zeros(6,1)]);

model_xf = [zeros(3,1); rpy_model(4:6); x0(7); 0; 0; zeros(3,1)];
xf_drake = ConvertToDrakeFrameFromModelFrame(model_xf);

xf_turn(7:9) = xf_drake(7:9);

cons = struct();
cons.c = [];
cons.N_fac = [];
cons.final_cost_on_time_gain = 4;

checkDependency('lcmgl');
lcmgl_f = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'deltawing-dircol-final-condition');
lcmgl_f.glColor4f(1,0,0,.5);
lcmgl_f.box(xf_turn(1:3), 2*bounds(1:3));

lcmgl_f.switchBuffers();

% draw the attempted trajectory
xtraj_draw = [x0 xf_turn];

[utraj_turn1, xtraj_turn1] = runDircol(parameters, x0, xf_turn, tf_turn, bounds, u0, cons, 15);
[utraj_turn2, xtraj_turn2] = runDircol(parameters, x0, xf_turn, xtraj_turn1.tspan(2), bounds, u0, cons, 31, utraj_turn1, xtraj_turn1);

% stabilize the trajectory with TVLQR

lib = AddLqrControllersToLib('left-turn-45', lib, xtraj_turn2, utraj_turn2, gains);

%% right turn 45 deg
bounds = [ 
  100         % x
  100         % y
  50          % z
  deg2rad(10)  % roll
  deg2rad(20) % pitch
  deg2rad(10) % yaw
  30           % x-dot
  30           % y-dot
  5           % z-dot
  deg2rad(70) % roll-dot
  deg2rad(70) % pitch-dot
  deg2rad(70) % yaw-dot
  ];


tf_turn = 2;

xf_turn = x0;
xf_turn(1) = x0(7)*.5*tf_turn;
xf_turn(2) = -x0(7)*.5*tf_turn;
xf_turn(6) = deg2rad(-45);

% compute final velocities

% input: velocity, body-x, final rpy
% output: velocity global x, y, and z
rpy_model = ConvertToModelFrameFromDrakeWorldFrame([zeros(3,1); xf_turn(4:6); zeros(6,1)]);

model_xf = [zeros(3,1); rpy_model(4:6); x0(7); 0; 0; zeros(3,1)];
xf_drake = ConvertToDrakeFrameFromModelFrame(model_xf);

xf_turn(7:9) = xf_drake(7:9);

cons = struct();
cons.c = [];
cons.N_fac = [];
cons.final_cost_on_time_gain = 4;

checkDependency('lcmgl');
lcmgl_f = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'deltawing-dircol-final-condition');
lcmgl_f.glColor4f(1,0,0,.5);
lcmgl_f.box(xf_turn(1:3), 2*bounds(1:3));

lcmgl_f.switchBuffers();

% draw the attempted trajectory
xtraj_draw = [x0 xf_turn];

[utraj_turn1, xtraj_turn1] = runDircol(parameters, x0, xf_turn, tf_turn, bounds, u0, cons, 15);
[utraj_turn2, xtraj_turn2] = runDircol(parameters, x0, xf_turn, xtraj_turn1.tspan(2), bounds, u0, cons, 31, utraj_turn1, xtraj_turn1);

% stabilize the trajectory with TVLQR

lib = AddLqrControllersToLib('right-turn-45', lib, xtraj_turn2, utraj_turn2, gains);


%% super aggressive left turn
bounds = [ 
  100         % x
  100         % y
  50          % z
  deg2rad(10)  % roll
  deg2rad(20) % pitch
  deg2rad(10) % yaw
  30           % x-dot
  30           % y-dot
  5           % z-dot
  deg2rad(70) % roll-dot
  deg2rad(70) % pitch-dot
  deg2rad(70) % yaw-dot
  ];


tf_turn = 3;

xf_turn = x0;
xf_turn(1) = x0(7)*.5*tf_turn;
xf_turn(2) = -x0(7)*.5*tf_turn;
xf_turn(6) = deg2rad(90);

% compute final velocities

% input: velocity, body-x, final rpy
% output: velocity global x, y, and z
rpy_model = ConvertToModelFrameFromDrakeWorldFrame([zeros(3,1); xf_turn(4:6); zeros(6,1)]);

model_xf = [zeros(3,1); rpy_model(4:6); x0(7); 0; 0; zeros(3,1)];
xf_drake = ConvertToDrakeFrameFromModelFrame(model_xf);

xf_turn(7:9) = xf_drake(7:9);

cons = [];

checkDependency('lcmgl');
lcmgl_f = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'deltawing-dircol-final-condition');
lcmgl_f.glColor4f(1,0,0,.5);
lcmgl_f.box(xf_turn(1:3), 2*bounds(1:3));

lcmgl_f.switchBuffers();

% draw the attempted trajectory
xtraj_draw = [x0 xf_turn];

[utraj_turn1, xtraj_turn1] = runDircol(parameters, x0, xf_turn, tf_turn, bounds, u0, cons, 15);
[utraj_turn2, xtraj_turn2] = runDircol(parameters, x0, xf_turn, xtraj_turn1.tspan(2), bounds, u0, cons, 31, utraj_turn1, xtraj_turn1);

% stabilize the trajectory with TVLQR

lib = AddLqrControllersToLib('super-aggressive-left-turn', lib, xtraj_turn2, utraj_turn2, gains);

%% super aggressive right turn
bounds = [ 
  100         % x
  100         % y
  50          % z
  deg2rad(10)  % roll
  deg2rad(20) % pitch
  deg2rad(10) % yaw
  30           % x-dot
  30           % y-dot
  5           % z-dot
  deg2rad(70) % roll-dot
  deg2rad(70) % pitch-dot
  deg2rad(70) % yaw-dot
  ];


tf_turn = 3;

xf_turn = x0;
xf_turn(1) = x0(7)*.5*tf_turn;
xf_turn(2) = -x0(7)*.5*tf_turn;
xf_turn(6) = deg2rad(-90);

% compute final velocities

% input: velocity, body-x, final rpy
% output: velocity global x, y, and z
rpy_model = ConvertToModelFrameFromDrakeWorldFrame([zeros(3,1); xf_turn(4:6); zeros(6,1)]);

model_xf = [zeros(3,1); rpy_model(4:6); x0(7); 0; 0; zeros(3,1)];
xf_drake = ConvertToDrakeFrameFromModelFrame(model_xf);

xf_turn(7:9) = xf_drake(7:9);

cons = [];

checkDependency('lcmgl');
lcmgl_f = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'deltawing-dircol-final-condition');
lcmgl_f.glColor4f(1,0,0,.5);
lcmgl_f.box(xf_turn(1:3), 2*bounds(1:3));

lcmgl_f.switchBuffers();

% draw the attempted trajectory
xtraj_draw = [x0 xf_turn];

[utraj_turn1, xtraj_turn1] = runDircol(parameters, x0, xf_turn, tf_turn, bounds, u0, cons, 15);
[utraj_turn2, xtraj_turn2] = runDircol(parameters, x0, xf_turn, xtraj_turn1.tspan(2), bounds, u0, cons, 31, utraj_turn1, xtraj_turn1);

% stabilize the trajectory with TVLQR

lib = AddLqrControllersToLib('super-aggressive-right-turn', lib, xtraj_turn2, utraj_turn2, gains);




%% super aggressive climb

bounds = [ 
  100         % x
  100         % y
  1          % z
  deg2rad(10)  % roll
  deg2rad(10) % pitch
  deg2rad(10) % yaw
  30           % x-dot
  30           % y-dot
  5           % z-dot
  deg2rad(70) % roll-dot
  deg2rad(70) % pitch-dot
  deg2rad(70) % yaw-dot
  ];


tf_turn = 3;

xf_turn = x0;
xf_turn(1) =  x0(7)*tf_turn;
xf_turn(3) = 10;

cons = [];

checkDependency('lcmgl');
lcmgl_f = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'deltawing-dircol-final-condition');
lcmgl_f.glColor4f(1,0,0,.5);
lcmgl_f.box(xf_turn(1:3), 2*bounds(1:3));

lcmgl_f.switchBuffers();

% draw the attempted trajectory
xtraj_draw = [x0 xf_turn];

[utraj_turn1, xtraj_turn1] = runDircol(parameters, x0, xf_turn, tf_turn, bounds, u0, cons, 15);
[utraj_turn2, xtraj_turn2] = runDircol(parameters, x0, xf_turn, xtraj_turn1.tspan(2), bounds, u0, cons, 31, utraj_turn1, xtraj_turn1);

% stabilize the trajectory with TVLQR

lib = AddLqrControllersToLib('super-aggressive-climb', lib, xtraj_turn2, utraj_turn2, gains);



%% super aggressive dive

bounds = [ 
  100         % x
  100         % y
  2          % z
  deg2rad(10)  % roll
  deg2rad(10) % pitch
  deg2rad(10) % yaw
  30           % x-dot
  30           % y-dot
  5           % z-dot
  deg2rad(70) % roll-dot
  deg2rad(70) % pitch-dot
  deg2rad(70) % yaw-dot
  ];


tf_turn = 4;

xf_turn = x0;
xf_turn(1) =  x0(7)*tf_turn;
xf_turn(3) = -10;
xf_turn(7) = x0(7) - 5;

utraj_init = ConstantTrajectory([0; 0; 0]);

cons = [];

checkDependency('lcmgl');
lcmgl_f = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'deltawing-dircol-final-condition');
lcmgl_f.glColor4f(1,0,0,.5);
lcmgl_f.box(xf_turn(1:3), 2*bounds(1:3));

lcmgl_f.switchBuffers();

% draw the attempted trajectory
xtraj_draw = [x0 xf_turn];

[utraj_turn1, xtraj_turn1] = runDircol(parameters, x0, xf_turn, tf_turn, bounds, u0, cons, 15, utraj_init);
[utraj_turn2, xtraj_turn2] = runDircol(parameters, x0, xf_turn, xtraj_turn1.tspan(2), bounds, u0, cons, 31, utraj_turn1, xtraj_turn1);

% stabilize the trajectory with TVLQR

lib = AddLqrControllersToLib('super-aggressive-dive', lib, xtraj_turn2, utraj_turn2, gains);


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
cons.final_cost_on_time_gain = 1;



[utraj_knife1, xtraj_knife1] = runDircol(parameters, x0, xf_knife_edge, tf_knife_edge, bounds, u0, cons, 11);
[utraj_knife2, xtraj_knife2] = runDircol(parameters, x0, xf_knife_edge, xtraj_knife1.tspan(2), bounds, u0, cons, 31, utraj_knife1, xtraj_knife1);

% stabilize the trajectory with TVLQR

lib = AddLqrControllersToLib('knife-edge', lib, xtraj_knife1, utraj_knife1, gains);
%lib = AddLqrControllersToLib('knife-edge', lib, xtraj_knife2, utraj_knife2, gains);
return;

%% alieron roll

bounds = [ 
  100         % x
  20          % y
  20          % z
  deg2rad(5)  % roll
  deg2rad(20) % pitch
  0.5         % yaw
  3           % x-dot
  5           % y-dot
  100         % z-dot
  deg2rad(70) % roll-dot
  deg2rad(70) % pitch-dot
  deg2rad(70) % yaw-dot
  ];

tf_roll = 2;
xf_roll = x0;
xf_roll(1) = x0(7)*tf_roll;
xf_roll(4) = deg2rad(360);


[utraj_roll1, xtraj_roll1] = runDircol(parameters, x0, xf_roll, tf_roll, bounds, u0, [], 11);
[utraj_roll2, xtraj_roll2] = runDircol(parameters, x0, xf_roll, xtraj_roll1.tspan(2), bounds, u0, [], 31, utraj_roll1, xtraj_roll1);

lib = AddLqrControllersToLib('knife-edge', lib, xtraj_roll2, utraj_roll2, gains);
