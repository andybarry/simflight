%% find fixed point

%clear

p = NonlinearProgram(5);
%p = p.setSolver('fmincon');

%parameters = { 1.92, 1.84, 2.41, 0.48, 0.57 };
parameters_old_before_3_31_2015 = {0.820; 2.499; 2.171; 0.697; 0.374; 0.028};
%parameters = { 0.254; 0.171; 4.048; 0.266; 0.0001; 0.036};
%parameters = { 0.4518, 0.3149, 4.9991, 0.4983, 0.0000};

% these were pretty good: parameters = { 0.687, 0.198, 3.803, 0.360, 0.000};

%parameters = { 0.764, 0.257, 1, 0.467, 0.000 };

%parameters = { 0.957, 0.110, 3.906, 0.199, 0.000 };

%parameters = { 0.968, 0.219, 4.995, 0.393, 0.000 };

%parameters = { 1.427, 0.321, 2.618, 0.611, 0.000 };

%parameters = {0.904, 0.000, -0.134, -0.049, -0.128 };


%%%% parameters = {0.904, 0.000, -0.134, -0.049, 0 };

func = @(in) tbsc_model_less_vars(in(1:2), in(3:5), parameters);


% min_xdot = 5;
% max_xdot = 30;
% 
% min_pitch = -1;
% max_pitch = 1;



c = FunctionHandleConstraint( zeros(6,1), zeros(6,1), 5, func);
c.grad_method = 'numerical';
p = p.addConstraint(c);

%c2 = BoundingBoxConstraint( [ 0.1; 10; -.5; -.5; 0 ], [1; 30; .5; .5; 4] );

%p = p.addConstraint(c2);


[x, objval, exitflag] = p.solve( [0; 18; 0; 0; 3.5] );




p = DeltawingPlant(parameters);

full_state = zeros(12,1);

full_state(5) = x(1);
full_state(7) = x(2);

%p.dynamics(0, full_state, x(3:5));

x0 = zeros(12, 1);
x0(5) = x(1);
x0(7) = x(2);

u0 = zeros(3,1);

u0(1) = x(3);
u0(2) = x(4);
u0(3) = x(5);

disp('x0:')
disp(x0);
 
disp('u0:')
disp(u0);


%% build lqr controller based on that trim


% I'd like to get Q and R tuned to give something close to APM's nominal
% PID values (omitting I since LQR can't do that)
%
% Roll:
%   P: 0.4
%   I: 0.04
%   D: 0.02
%
% Pitch:
%   P: 0.4
%   I: 0.04
%   D: 0.02
%
% Yaw:
%   P: 1.0
%   I: 0
%   D: 0




Q = diag([0 0 0 10 30 .25 0.1 .0001 0.0001 .001 .001 .1]);
Q(1,1) = 1e-10; % ignore x-position
Q(2,2) = 1e-10; % ignore y-position
Q(3,3) = 1e-10; % ignore z-position


%R = diag([35 35 35]);
R_values = [35 50 25];

[A, B, C, D, xdot0, y0] = p.linearize(0, x0, u0);
%% check linearization

%(A*(x0-x0) + B*(u0-u0) + xdot0) - p.dynamics(0, x0, u0)

%(A*.1*ones(12,1) + B*.1*ones(3,1) + xdot0) - p.dynamics(0, x0+.1*ones(12,1), u0+.1*ones(3,1))

%% compte difference to PID gains K

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

%K
K_pd

K_pd_yaw = K_pd;
K_pd_aggressive_yaw = K_pd;

K_pd_yaw(1,6) = 0.25;
K_pd_yaw(2,6) = -0.25;

K_pd_aggressive_yaw(1,6) = 0.5;
K_pd_aggressive_yaw(2,6) = -0.5;


%% add a bunch of controllers

xtraj = ConstantTrajectory(x0);
utraj = ConstantTrajectory(u0);

gains.Q = Q;
gains.Qf = Q;
gains.R = diag([35 35 35]);
gains.K_pd = K_pd;
gains.K_pd_yaw = K_pd_yaw;
gains.K_pd_aggressive_yaw = K_pd_aggressive_yaw;


lib = WriteTiqrControllers(lib, 'tilqr', p, A, B, x0, u0, parameters, gains);

return;

%% compute lqr controller






% add in yaw
%K_pd(1,6) = 1;
%K_pd(2,6) = -1;
% 
%K = K_pd;


% add in roll
%K(1,10) = -0.08;
%K(2,10) = 0.08;

% increased pitch
%K(1,11) = -0.03;
%K(2,11) = 0.03;



% kill everything that isn't pitch or roll
%K(:, 1:3) = 0; % kill xyz
%K(:, 6) = 0; % yaw
%K(:, 7) = 0; % airspeed
%K(:, 8:9) = 0; % ydot zdot
%K(:, 12) = 0; % yawdot
K


%% build a Trajectory so that we can use all of the TrajectoryLibrary tools

xtraj = ConstantTrajectory(x0);
utraj = ConstantTrajectory(u0);

ktraj = ConstantTrajectory(-K);
affine_traj = ConstantTrajectory(zeros(3,1));

lqrsys = struct();
lqrsys.D = ktraj;
lqrsys.y0 = affine_traj;



comments = sprintf('%s\n\n%s', ['TILQR from model, full gains', prettymat('Parameters', cell2mat(parameters), 3) ...
  prettymat('Q', Q, 5) prettymat('R', R)]);
%comments = sprintf('%s', [prettymat('Parameters', cell2mat(parameters), 3) ...
%  'K_pd from APM WITH aggressive YAW']);

traj = TrajectoryInLibrary(xtraj, utraj, lqrsys, p.getStateFrame(), 'tilqr-pd', comments);

traj.WriteToFile('trajlib/tilqr-full-10005', .01, true);
