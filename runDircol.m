function [utraj, xtraj, prog, r] = runDircol(parameters, x0, xf, final_speed, tf0, bounds_delta, u0, additional_constraints, N, utraj_guess, xtraj_guess)

  num_args_req = 7;

  if nargin < num_args_req + 1 || isempty(additional_constraints)
    additional_constraints = struct();
    additional_constraints.c = [];
    additional_constraints.N_fac = [];
  end
  
  
  if nargin < num_args_req + 2
    %N = 11; % number of knot points
    N = round(tf0 * 10) + 1;
  end

  
  if nargin < num_args_req + 3
     traj_init.x = PPTrajectory(foh([0, tf0], [x0, xf]));
  else
    traj_init.x = xtraj_guess;
  end
  
  if nargin < num_args_req + 4
    traj_init.u = ConstantTrajectory(u0);
  else
    traj_init.u = utraj_guess;
    assert(traj_init.x.tspan(2) == traj_init.u.tspan(2), 'Init trajectories have different tspan');
  end
  
  if tf0 ~= traj_init.x.tspan(2)
    tf0 = traj_init.x.tspan(2);
    warning('tf0 ~= traj_init.x.tspan(2), ignoring tf0 parameter.');
  end
  
  
  % run trajectory optimization
  
  checkDependency('lcmgl');
  lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'deltawing-dircol');
  
  lcmgl_f = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'deltawing-dircol-final-condition');
  
  !echo "0" > abort.txt


  %% build drake objects

  p = DeltawingPlant(parameters);


  options.floating = true;
  r = RigidBodyManipulator('TBSC_visualizer.urdf', options);

  v2 = HudBotVisualizer(r);
  %v2 = r.constructVisualizer();
  
  %% draw final state
  
  lcmgl_f.glColor4f(1,0,0,.5);
  lcmgl_f.box(xf(1:3), 2*bounds_delta(1:3));
  
  lcmgl_f.switchBuffers();

  %% run trajectory optimization


  %N = 21; % number of knot points
  minimum_duration = max(0, tf0 - tf0/2);
  maximum_duration = tf0 + tf0/2;

  prog = DircolTrajectoryOptimization(p, N, [minimum_duration, maximum_duration]);
  
  prog = prog.addStateConstraint(ConstantConstraint(x0), 1);
  prog = prog.addInputConstraint(ConstantConstraint(u0), 1);
  
  
  for i = 1 : length(additional_constraints.c)
    prog = prog.addStateConstraint(additional_constraints.c(i), round(additional_constraints.N_fac(i) * N));
  end

  prog = prog.addStateConstraint(BoundingBoxConstraint(xf-bounds_delta, xf+bounds_delta), N);
  prog = prog.addInputConstraint(ConstantConstraint(u0), N);

  prog = prog.addRunningCost(@cost);
  
  FinalCostWrapper = @(x) FinalCostOnState(x, xf, final_speed);
  
  final_cost = FunctionHandleConstraint(-inf, inf, 12, FinalCostWrapper, 0);
  final_cost.grad_method = 'numerical';
  prog = prog.addCost(final_cost, prog.x_inds(:,N));
  
  % also have a cost on final time
  
  prog = prog.addFinalCost(@FinalCostOnTime);
  
  %prog = prog.addFinalCost(@finalCost);
  
  prog = prog.addTrajectoryDisplayFunction(@plotDircolTraj);
  
  prog = prog.setSolverOptions('snopt','print','print.out');


  info = 0;

  disp(['Starting trajectory optimization (N = ' num2str(N) ')...']);
  %while (info~=1)
    tic
    [xtraj, utraj, z, F, info] = prog.solveTraj(tf0, traj_init);
    toc
    %keyboard;
  %end
  
%   for i=1:100000
%     disp(i)
%     pause(1)
%   end


  %% visualize

  % combine the simulated trajectory with the inputs

  traj_and_u = [xtraj; utraj];

  fr = traj_and_u.getOutputFrame();

  transform_func = @(t, x, x_and_u) [ x_and_u(1:6); x_and_u(15); x_and_u(13:14); x_and_u(7:12); zeros(3,1)];

  trans = FunctionHandleCoordinateTransform(17, 0, traj_and_u.getOutputFrame(), v2.getInputFrame(), true, true, transform_func, transform_func, transform_func);

  fr.addTransform(trans);


  playback(v2, traj_and_u, struct('slider', true));
  
  assert(info == 1, 'Trajectory optimization failed to find a solution.')
  
  DrawTrajectoryLcmGl(xtraj);
  

  function [g,dg] = cost(dt,x,u)

    R = 0.1*eye(3);
    R(3,3) = 0; % low cost on throttle action
    g = u'*R*u;
    %g = sum((R*u).*u,1);
    dg = [zeros(1,1+size(x,1)),2*u'*R];
    %dg = zeros(1, 1 + size(x,1) + size(u,1));

  end

  function [h] = FinalCostOnState(x, xf0, final_speed)

    x2 = [x(4:6); x(9:12)];
    xf0_2 = [xf0(4:6); xf0(9:12)];
    
    x_body = ConvertToModelFrameFromDrakeWorldFrame(x);
    
    h = norm(x2 - xf0_2) + abs(x_body(7) - final_speed);
    
    %h = 0;
    
    %h = 20 * sum((xf(1:3) - x(1:3)).^2);
    
    %h = h + 1 * sum((xf(4:9) - x(4:9)).^2);
    
    %dh = [1,zeros(1,size(x,1))];

  end

  function [h, dh] = FinalCostOnTime(T, xf)
    h = T;
    dh = zeros(1,13);
    dh(1) = 1;
  end
  


  function plotDircolTraj(t,x,u)
    options_draw.color = [0 0 1];
    DrawTrajectoryLcmGl(x, 'dircol-running', options_draw);

    assert(fscanf(fopen('abort.txt', 'r'), '%d') == 0, 'Abort from file.')
    
  end

end

