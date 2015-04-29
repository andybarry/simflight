
clear

HudBotVisualizer.InitJava();

lib = TrajectoryLibrary();

%parameters = {0.820; 2.499; 2.171; 0.697; 0.374; 0.028};
parameters = {0.904, 0.000, -0.134, -0.049, 0 }

p = DeltawingPlant(parameters);

throttle_max = 5.33976;

tf = 0.5;

%Q = diag([10 10 10 1 1 1 .1 .1 .1 .1 .1 .1]);
Q = diag([0 0 0 10 50 .25 0.1 .0001 0.0001 .1 .01 .1]);
Q(1,1) = 1e-10; % ignore x-position
Q(2,2) = 1e-10; % ignore y-position
Q(3,3) = 1e-10; % ignore z-position

R_values = [35 50 25];

Qf = eye(12);


K_pd = zeros(3,12);

% roll P
K_pd(1,4) = -0.4;
K_pd(2,4) = 0.4;

% roll D
K_pd(1,10) = -0.02;
K_pd(2,10) = 0.02;

% pitch P
K_pd(1,5) = -0.4;
K_pd(2,5) = -0.4;

% pitch D
K_pd(1,11) = -0.02;
K_pd(2,11) = -0.02;

K_pd_yaw = K_pd;
K_pd_aggressive_yaw = K_pd;

K_pd_yaw(1,6) = 0.25;
K_pd_yaw(2,6) = -0.25;

K_pd_aggressive_yaw(1,6) = 0.5;
K_pd_aggressive_yaw(2,6) = -0.5;

gains.Q = Q;
gains.Qf = Qf;
gains.R_values = R_values;
gains.K_pd = K_pd;
gains.K_pd_yaw = K_pd_yaw;
gains.K_pd_aggressive_yaw = K_pd_aggressive_yaw;


%% straight ahead

t_mark = [0, tf];
elevL = [0.2, 0.2];
elevR = [0.2, 0.2];
throttle = [0.9 * throttle_max, 0.9 * throttle_max];

utraj = PPTrajectory(foh(t_mark, [elevL; elevR; throttle]));
utraj = utraj.setOutputFrame(p.getInputFrame());

xtraj = runInputTape(p, utraj, tf);


lib = AddLqrControllersToLib('straight', lib, p, xtraj, utraj, parameters, gains);



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
% lib = lib.AddTrajectory(p, xtraj, utraj, lqr_controller, comments);

%% turn right

t_mark = [0, .3];
elevL = [-0.1, -0.1];
elevR = [0.2, 0.2];
throttle = [0.9 * throttle_max, 0.9 * throttle_max];

utraj = PPTrajectory(foh(t_mark, [elevL; elevR; throttle]));
utraj = utraj.setOutputFrame(p.getInputFrame());

xtraj = runInputTape(p, utraj, 0.3);



lib = AddLqrControllersToLib('turn-right', lib, p, xtraj, utraj, parameters, gains);


%% go up

% first pull up, then fly straight

t_mark = [0, .2, .3, 1.5];
elevs = [0.2, 0.2, 0, 0];
throttle = [throttle_max, throttle_max, throttle_max, throttle_max];

utraj = PPTrajectory(foh(t_mark, [elevs; elevs; throttle]));

utraj = utraj.setOutputFrame(p.getInputFrame());

[xtraj, sys] = runInputTape(p, utraj, tf);



lib = AddLqrControllersToLib('climb', lib, p, xtraj, utraj, parameters, gains);



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
% lib = lib.AddTrajectory(p, xtraj, utraj, lqr_controller, comments);

%% draw

lib.DrawTrajectories();