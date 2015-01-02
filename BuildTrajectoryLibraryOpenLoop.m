
clear

lib = TrajectoryLibrary();

parameters = { 1.92, 1.84, 2.41, 0.48, 0.57 };

p = DeltawingPlant(parameters);

throttle_max = 5.33976;

%% straight ahead

tf = 0.5;

u0 = [0.2, 0.2, 0.9 * throttle_max]';

utraj = ConstantTrajectory(u0);
utraj = utraj.setOutputFrame(p.getInputFrame());

xtraj = runInputTape(p, utraj, tf);

lib = lib.AddTrajectory(xtraj, utraj);



%% turn left

u0 = [0.6 -0.2 0.9 * throttle_max]';

utraj = ConstantTrajectory(u0);
utraj = utraj.setOutputFrame(p.getInputFrame());

xtraj = runInputTape(p, utraj, tf);

lib = lib.AddTrajectory(xtraj, utraj);

%% go up

% first pull up, then fly straight

t_mark = [0, .2, .3, .5];
elevs = [0.6, 0.6, 0, 0];
throttle = [throttle_max, throttle_max, throttle_max, throttle_max];

utraj = PPTrajectory(foh(t_mark, [elevs; elevs; throttle]));

utraj = utraj.setOutputFrame(p.getInputFrame());

[xtraj, sys] = runInputTape(p, utraj, tf);

Q = eye(12);
R = eye(3);
Qf = eye(12);

ltvsys = tvlqr(p, xtraj, utraj, Q, R, Qf)

lib = lib.AddTrajectory(xtraj, utraj);

%% draw

lib.DrawTrajectories();