function [utraj, xtraj, prog, r] = runDircol
  % run trajectory optimization
  
  javaaddpath('/home/abarry/realtime/LCM/LCMtypes.jar');
  javaaddpath('/home/abarry/Fixie/build/share/java/lcmtypes_mav-lcmtypes.jar');

  %% setup

  parameters = { 1.92, 1.84, 2.41, 0.48, 0.57 };

  x = 0;
  y = 0;
  z = 0;
  roll = 0;
  pitch = 0;
  yaw = 0;
  xdot = 15;
  ydot = 0;
  zdot = 0;
  rolldot = 0;
  pitchdot = 0;
  yawdot = 0;

  x0_drake = [ x; y; z; roll; pitch; yaw; xdot; ydot; zdot; rolldot; pitchdot; yawdot ]


  %% build drake objects

  p = DeltawingPlant(parameters);


  options.floating = true;
  r = RigidBodyManipulator('TBSC_visualizer.urdf', options);

  v2 = HudBotVisualizer(r);
  %v2 = r.constructVisualizer();


  %% run trajectory optimization

  N = 11; % number of knot points
  minimum_duration = 0.1;
  maximum_duration = 2.0;

  prog = DircolTrajectoryOptimization(p, N, [minimum_duration, maximum_duration]);

  u0 = [0, 0, 0];

  prog = prog.addStateConstraint(ConstantConstraint(x0_drake), 1);
  prog = prog.addInputConstraint(ConstantConstraint(u0), 1);

%   xf = x0_drake;
% 
%   xf(1) = 15;
%   xf(2) = 0;
%   xf(3) = -3.68;
%   
%   xf(4) = 0;
%   xf(5) = .51;
%   xf(6) = 0;
%   
%   xf(7) = 14.95;
%   xf(8) = 0;
%   xf(9) = -8.42;
%   
%   xf(10) = 0;
%   xf(11) = 0.48;
%   xf(12) = 0;
  
  
   xf = [14.9949
         0
   -1
         0
    0.5169
         0
   14.9585
         0
   -8.4280
         0
    0.2872
         0];

  prog = prog.addStateConstraint(ConstantConstraint(xf), N);
  prog = prog.addInputConstraint(ConstantConstraint(u0), N);

  prog = prog.addRunningCost(@cost);
  prog = prog.addFinalCost(@finalCost);

  tf0 = 1;
  traj_init.x = PPTrajectory(foh([0, tf0], [x0_drake, xf]));
  traj_init.u = ConstantTrajectory(u0);

  info = 0;

  disp('Starting trajectory optimization...');
  %while (info~=1)
    tic
    [xtraj, utraj, z, F, info] = prog.solveTraj(tf0, traj_init);
    toc
  %end


  %% visualize

  % combine the simulated trajectory with the inputs

  traj_and_u = [xtraj; utraj];

  fr = traj_and_u.getOutputFrame();

  transform_func = @(t, x, x_and_u) [ x_and_u(1:6); x_and_u(15); x_and_u(13:14); x_and_u(7:12); zeros(3,1)];

  trans = FunctionHandleCoordinateTransform(17, 0, traj_and_u.getOutputFrame(), v2.getInputFrame(), true, true, transform_func, transform_func, transform_func);

  fr.addTransform(trans);


  playback(v2, traj_and_u, struct('slider', true));
  
  assert(info == 1, 'Trajectory optimization failed to find a solution.')
  
end

function [g,dg] = cost(dt,x,u)

  R = eye(3);
  g = u'*R*u;
  %g = sum((R*u).*u,1);
  dg = [zeros(1,1+size(x,1)),2*u'*R];
  %dg = zeros(1, 1 + size(x,1) + size(u,1));

end

function [h,dh] = finalCost(t,x)

  h = t;
  dh = [1,zeros(1,size(x,1))];

end


