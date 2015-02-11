%% find fixed point

clear

p = NonlinearProgram(5);
%p = p.setSolver('fmincon');


func = @(in) tbsc_model_less_vars(in(1:2), in(3:5));


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


[x, objval, exitflag] = p.solve( [0; 20; 0; 0; 3.5] )


parameters = { 1.92, 1.84, 2.41, 0.48, 0.57 };

p = DeltawingPlant(parameters);

full_state = zeros(12,1);

full_state(5) = x(1);
full_state(7) = x(2);

p.dynamics(0, full_state, x(3:5))


%% build lqr controller based on that trim

x0 = zeros(12, 1);
x0(5) = x(1);
x0(7) = x(2);

u0 = zeros(3,1);

u0(1) = x(3);
u0(2) = x(4);
u0(3) = x(5);

Q = eye(12);
R = 10*eye(3);


[A, B, C, D, xdot0, y0] = p.linearize(0, x0, u0);

%% check linearization

(A*(x0-x0) + B*(u0-u0) + xdot0) - p.dynamics(0, x0, u0)

(A*.1*ones(12,1) + B*.1*ones(3,1) + xdot0) - p.dynamics(0, x0+.1*ones(12,1), u0+.1*ones(3,1))

%% compute lqr controller

K = lqr(full(A), full(B), Q, R)