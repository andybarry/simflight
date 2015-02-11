
clear

HudBotVisualizer.InitJava();

lib = TrajectoryLibrary();

parameters = { 1.92, 1.84, 2.41, 0.48, 0.57 };

p = DeltawingPlant(parameters);

throttle_max = 5.33976;

tf = 0.5;

%Q = diag([10 10 10 1 1 1 .1 .1 .1 .1 .1 .1]);
Q = diag([0 0 0 1 1 1 0 .1 .1 .1 .1 .1]);
R = diag([100 100 10]);
Qf = eye(12);

%% straight ahead

t_mark = [0, tf];
elevL = [0.2, 0.2];
elevR = [0.2, 0.2];
throttle = [0.9 * throttle_max, 0.9 * throttle_max];

utraj = PPTrajectory(foh(t_mark, [elevL; elevR; throttle]));
utraj = utraj.setOutputFrame(p.getInputFrame());

xtraj = runInputTape(p, utraj, tf);

disp('Computing TVLQR controller...');
lqr_controller = tvlqr(p, xtraj, utraj, Q, R, Qf)
disp('done.');


lib = lib.AddTrajectory(p, xtraj, utraj, lqr_controller);



%% turn left

t_mark = [0, tf];
elevL = [0.6, 0.6];
elevR = [-0.2, -0.2];
throttle = [0.9 * throttle_max, 0.9 * throttle_max];

utraj = PPTrajectory(foh(t_mark, [elevL; elevR; throttle]));
utraj = utraj.setOutputFrame(p.getInputFrame());

xtraj = runInputTape(p, utraj, tf);


disp('Computing TVLQR controller...');
lqr_controller = tvlqr(p, xtraj, utraj, Q, R, Qf)
disp('done.');


lib = lib.AddTrajectory(p, xtraj, utraj, lqr_controller);

%% turn right

t_mark = [0, tf];
elevL = [-0.2, -0.2];
elevR = [0.6, 0.6];
throttle = [0.9 * throttle_max, 0.9 * throttle_max];

utraj = PPTrajectory(foh(t_mark, [elevL; elevR; throttle]));
utraj = utraj.setOutputFrame(p.getInputFrame());

xtraj = runInputTape(p, utraj, tf);


disp('Computing TVLQR controller...');
lqr_controller = tvlqr(p, xtraj, utraj, Q, R, Qf)
disp('done.');


lib = lib.AddTrajectory(p, xtraj, utraj, lqr_controller);

%% go up

% first pull up, then fly straight

t_mark = [0, .2, .3, .5];
elevs = [0.6, 0.6, 0, 0];
throttle = [throttle_max, throttle_max, throttle_max, throttle_max];

utraj = PPTrajectory(foh(t_mark, [elevs; elevs; throttle]));

utraj = utraj.setOutputFrame(p.getInputFrame());

[xtraj, sys] = runInputTape(p, utraj, tf);


disp('Computing TVLQR controller...');
lqr_controller = tvlqr(p, xtraj, utraj, Q, R, Qf)
disp('done.');

lib = lib.AddTrajectory(p, xtraj, utraj, lqr_controller);


%% go down

t_mark = [0, .1, .2, .5];
elevs = [-0.6, -0.6, 0, 0];
throttle = [throttle_max, throttle_max, throttle_max, throttle_max];

utraj = PPTrajectory(foh(t_mark, [elevs; elevs; throttle]));

utraj = utraj.setOutputFrame(p.getInputFrame());

[xtraj, sys] = runInputTape(p, utraj, tf);

disp('Computing TVLQR controller...');
lqr_controller = tvlqr(p, xtraj, utraj, Q, R, Qf)
disp('done.');

lib = lib.AddTrajectory(p, xtraj, utraj, lqr_controller);

%% draw

lib.DrawTrajectories();