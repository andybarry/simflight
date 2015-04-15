%% find fixed point

clear

p = NonlinearProgram(5);
%p = p.setSolver('fmincon');

%parameters = { 1.92, 1.84, 2.41, 0.48, 0.57 };
parameters_old_before_3_31_2015 = {0.820; 2.499; 2.171; 0.697; 0.374; 0.028};
%parameters = { 0.254; 0.171; 4.048; 0.266; 0.0001; 0.036};
parameters = { 0.421; 3.766; 3.310; 0.593; 0.888; 0.030};

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

% disp('x0:')
% disp(x0);
% 
% disp('u0:')
% disp(u0);


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




Q = diag([0 0 0 10 50 .25 0.1 .0001 .0001 .1 .01 .1]);
Q(1,1) = 1e-10; % ignore x-position
Q(2,2) = 1e-10; % ignore y-position
Q(3,3) = 1e-10; % ignore z-position


R = diag([25 25 25]);

[A, B, C, D, xdot0, y0] = p.linearize(0, x0, u0);
%% check linearization

%(A*(x0-x0) + B*(u0-u0) + xdot0) - p.dynamics(0, x0, u0)

%(A*.1*ones(12,1) + B*.1*ones(3,1) + xdot0) - p.dynamics(0, x0+.1*ones(12,1), u0+.1*ones(3,1))

%% compute lqr controller

K = lqr(full(A), full(B), Q, R);

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

K
K_pd


% add in yaw
%K_pd(1,6) = 1;
%K_pd(2,6) = -1;

%K = K_pd;


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

comments = sprintf('%s', [prettymat('Parameters', cell2mat(parameters), 3) ...
  prettymat('Q', Q, 5) prettymat('R', R)]);
%comments = sprintf('%s', [prettymat('Parameters', cell2mat(parameters), 3) ...
%  'K_pd from APM WITH aggressive YAW']);

traj = TrajectoryInLibrary(xtraj, utraj, lqrsys, p.getStateFrame(), comments);

traj.WriteToFile('trajlib/lqr-trim-10005', .01, true);
