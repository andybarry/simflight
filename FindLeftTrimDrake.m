function [x0, u0, lib] = FindLeftTrimDrake(p, desired_roll, desired_climb_rate, nominal_xdot, lib, trajname, gains)
    %% find fixed point

    if nargin < 4
      nominal_xdot = 12;
    end
    
    if nargin < 5
        lib = TrajectoryLibrary(p);
    end
    
    if nargin < 6
      trajname = 'TI-trim';
    end
    
    %desired_roll = deg2rad(50);
    
    initial_guess = zeros(12,1);
    initial_guess(4) = desired_roll;
    initial_guess(7) = nominal_xdot;
    initial_guess(13:14) = [0; 0];
    initial_guess(15) = p.umax(3)-.5;
    
    %initial_guess = [desired_roll; 0; 12; 0; 0; 0; p.umax(3)-.5];
    num_decision_vars = length(initial_guess);
    
    
    disp('Searching for fixed point...');
    
    prog = NonlinearProgram(num_decision_vars);

    func = @(in) tbsc_model_for_turn(in(1:12), in(13:15), p.parameters);


    % min_xdot = 5;
    % max_xdot = 30;
    % 
    min_pitch = -1.4;
    max_pitch = 1.4;

    % constraint on:
    % 3 z-ddot_body
    % 4 roll-ddot
    % 5 pitch-ddot
    
    num_constraints = 6;
    lb = zeros(num_constraints, 1);
    ub = zeros(num_constraints, 1);
    
    c = FunctionHandleConstraint( lb, ub, num_decision_vars, func);
    c.grad_method = 'numerical';
    prog = prog.addConstraint(c);
    
    %CostFunc = @(in) abs(in(1)-desired_roll);
    %cost = FunctionHandleConstraint( -Inf, Inf, num_decision_vars, CostFunc);
    %cost.grad_method = 'numerical';
    %prog = prog.addCost(cost);
    
    
    input_limits_lower = -Inf*ones(15,1);
    input_limits_upper = Inf*ones(15,1);
    
    input_limits_lower(2) = 0;
    input_limits_upper(2) = 0;
    
    input_limits_lower(4) = desired_roll - deg2rad(20);
    input_limits_upper(4) = desired_roll + deg2rad(20);
    
    input_limits_lower(8) = 0;
    input_limits_upper(8) = 0;
    
    if desired_climb_rate ~= 0
      input_limits_lower(9) = desired_climb_rate - .5;
      input_limits_upper(9) = desired_climb_rate + .5;
    else
      input_limits_lower(9) = 0;
      input_limits_upper(9) = 0;
    end
    
    % turning climb
    %input_limits_lower(9) = 1.5;
    %input_limits_upper(9) = 10;
    
    input_limits_lower(10) = 0;
    input_limits_upper(10) = 0;
    
    input_limits_lower(11) = 0;
    input_limits_upper(11) = 0;
    

    
    input_limits_lower(13:15) = p.umin;
    input_limits_upper(13:15) = p.umax;
    
    %c_input_limits = BoundingBoxConstraint([deg2rad(-50); min_pitch; -Inf; -Inf; p.umin], [deg2rad(-5); max_pitch; Inf; Inf; p.umax]);
    c_input_limits = BoundingBoxConstraint(input_limits_lower, input_limits_upper);
    
    prog = prog.addConstraint(c_input_limits);
    
    
    

    %c2 = BoundingBoxConstraint( [ 0.1; 10; -.5; -.5; 0 ], [1; 30; .5; .5; 4] );

    %p = p.addConstraint(c2);


    [x, objval, exitflag] = prog.solve( initial_guess );



    
    assert(exitflag == 1, ['Solver error: ' num2str(exitflag)]);

    x0_drake = x(1:12);
    u0 = x(13:15);

    %full_state = zeros(12,1);

    %full_state(5) = x(1);
    %full_state(7) = x(2);

    %p.dynamics(0, full_state, x(3:5));

%     x0 = zeros(12, 1);
%     x0(4) = x(1);
%     x0(5) = x(2);
%     x0(7) = x(3);
%     x0_drake = x0;
%     
     x0 = ConvertDrakeFrameToEstimatorFrame(x0_drake);
% 
%     u0 = zeros(3,1);
% 
%     u0(1) = x(4);
%     u0(2) = x(5);
%     u0(3) = x(6);
    
    disp('Fixed point found:');

    disp('x0 (drake frame):')
    disp(x0_drake');

    disp('u0:')
    disp(u0');
    
    disp('xdot (drake frame):');
    xdot_temp = p.p.dynamics(0, x0_drake, u0);
    disp(xdot_temp');
    
    disp('xdot (est frame):');
    xdot_temp = p.dynamics(0, x0, u0);
    disp(xdot_temp');
    
    disp(['xdot: ' num2str(x0(7)), ' m/s']);
    disp(['roll angle: ' num2str(rad2deg(x0(4))) ' deg']);
    disp(['climb rate: ' num2str(x0_drake(9)) ' m/s']);


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

    if abs(x0(12)) > 0.1
      % we are expecting some yaw-dot, so no gain on yaw
      gains.Q(6,6) = 1e-10; % ignore yaw
      gains.Q(2,2) = 1e-10; % ignore Y
      disp(['ignoring yaw on ' trajname]);
    end

    [A, B, C, D, xdot0, y0] = p.linearize(0, x0, u0);
    %% check linearization

    %(A*(x0-x0) + B*(u0-u0) + xdot0) - p.dynamics(0, x0, u0)

    %(A*.1*ones(12,1) + B*.1*ones(3,1) + xdot0) - p.dynamics(0, x0+.1*ones(12,1), u0+.1*ones(3,1))




    %% add a bunch of controllers


    lib = AddTiqrControllers(lib, trajname, A, B, x0, u0, gains);
end
