
clear

HudBotVisualizer.InitJava();

lib = TrajectoryLibrary();

% add stabilization trajectories

%parameters = {0.820; 2.499; 2.171; 0.697; 0.374; 0.028};
[parameters, gains] = GetDefaultGains();

FindTrimDrake



p = DeltawingPlant(parameters);

throttle_max = 5.33976;

tf = 0.5;

%Q = diag([10 10 10 1 1 1 .1 .1 .1 .1 .1 .1]);


%% straight ahead

t_mark = [0, tf];
elevL = [0.2, 0.2];
elevR = [0.2, 0.2];
throttle = [0.9 * throttle_max, 0.9 * throttle_max];

utraj = PPTrajectory(foh(t_mark, [elevL; elevR; throttle]));
utraj = utraj.setOutputFrame(p.getInputFrame());

xtraj = runInputTape(p, utraj, tf);


lib = AddLqrControllersToLib('straight', lib, xtraj, utraj, parameters, gains);



%% turn left
% 
% t_mark = [0, tf];
% elevL = [0.6, 0.6];
% elevR = [-0.2, -0.2];
% throttle = [0.9 * throttle_max, 0.9 * throttle_max];
% 
% utraj = PPTrajectory(foh(t_mark, [elevL; elevR; throttle]));
% utraj = utraj.setOutputFrame(p.getInputFrame());
% 
% xtraj = runInputTape(p, utraj, tf);
% 
% 
% disp('Computing TVLQR controller...');
% lqr_controller = tvlqr(p, xtraj, utraj, Q, R, Qf)
% disp('done.');
% 
% comments = sprintf('%s\n\n%s', 'Turn left', [prettymat('Parameters', cell2mat(parameters), 3) ...
%   prettymat('Q', Q, 5) prettymat('R', R)]);
% lib = lib.AddTrajectory(xtraj, utraj, lqr_controller, comments);

%% turn right

t_mark = [0, .3];
elevL = [-0.1, -0.1];
elevR = [0.2, 0.2];
throttle = [0.9 * throttle_max, 0.9 * throttle_max];

utraj = PPTrajectory(foh(t_mark, [elevL; elevR; throttle]));
utraj = utraj.setOutputFrame(p.getInputFrame());

xtraj = runInputTape(p, utraj, 0.3);



lib = AddLqrControllersToLib('turn-right', lib, xtraj, utraj, parameters, gains);


%% go up

% first pull up, then fly straight

t_mark = [0, .2, .3, 1.5];
elevs = [0.2, 0.2, 0, 0];
throttle = [throttle_max, throttle_max, throttle_max, throttle_max];

utraj = PPTrajectory(foh(t_mark, [elevs; elevs; throttle]));

utraj = utraj.setOutputFrame(p.getInputFrame());

[xtraj, sys] = runInputTape(p, utraj, tf);



lib = AddLqrControllersToLib('climb', lib, xtraj, utraj, parameters, gains);



%% go down

% t_mark = [0, .1, .2, .5];
% elevs = [-0.6, -0.6, 0, 0];
% throttle = [throttle_max, throttle_max, throttle_max, throttle_max];
% 
% utraj = PPTrajectory(foh(t_mark, [elevs; elevs; throttle]));
% 
% utraj = utraj.setOutputFrame(p.getInputFrame());
% 
% [xtraj, sys] = runInputTape(p, utraj, tf);
% 
% disp('Computing TVLQR controller...');
% lqr_controller = tvlqr(p, xtraj, utraj, Q, R, Qf)
% disp('done.');
% 
% comments = sprintf('%s\n\n%s', 'Dive', [prettymat('Parameters', cell2mat(parameters), 3) ...
%   prettymat('Q', Q, 5) prettymat('R', R)]);
% lib = lib.AddTrajectory(xtraj, utraj, lqr_controller, comments);

%% draw

lib.DrawTrajectories();