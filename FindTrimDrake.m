% find fixed point

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