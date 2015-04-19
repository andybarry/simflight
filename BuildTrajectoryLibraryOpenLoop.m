
clear

HudBotVisualizer.InitJava();

lib = TrajectoryLibrary();

%parameters = {0.820; 2.499; 2.171; 0.697; 0.374; 0.028};
parameters = { 1.427, 0.321, 2.618, 0.611, 0.000 };

p = DeltawingPlant(parameters);

throttle_max = 5.33976;

tf = 0.5;

%Q = diag([10 10 10 1 1 1 .1 .1 .1 .1 .1 .1]);
Q = diag([0 0 0 10 50 .25 0.1 .0001 0.0001 .1 .01 .1]);
Q(1,1) = 1e-10; % ignore x-position
Q(2,2) = 1e-10; % ignore y-position
Q(3,3) = 1e-10; % ignore z-position


R = diag([35 35 35]);

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

comments = sprintf('%s\n\n%s', 'Straight ahead trajectory', [prettymat('Parameters', cell2mat(parameters), 3) ...
  prettymat('Q', Q, 5) prettymat('R', R)]);
lib = lib.AddTrajectory(p, xtraj, utraj, lqr_controller, comments);



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


lib = lib.AddTrajectory(p, xtraj, utraj, lqr_controller, comments);

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


lib = lib.AddTrajectory(p, xtraj, utraj, lqr_controller, comments);

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

lib = lib.AddTrajectory(p, xtraj, utraj, lqr_controller, comments);


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

lib = lib.AddTrajectory(p, xtraj, utraj, lqr_controller, comments);

%% draw

lib.DrawTrajectories();