function xtraj = runUtraj(utraj, x0)
  % run trajectory optimization
  
  javaaddpath('/home/abarry/realtime/LCM/LCMtypes.jar');
  javaaddpath('/home/abarry/Fixie/build/share/java/lcmtypes_mav-lcmtypes.jar');

  %% setup

  parameters = { 1.92, 1.84, 2.41, 0.48, 0.57 };
% 
%   x = 0;
%   y = 0;
%   z = 0;
%   roll = 0;
%   pitch = 0;
%   yaw = 0;
%   xdot = 15;
%   ydot = 0;
%   zdot = 0;
%   rolldot = 0;
%   pitchdot = 0;
%   yawdot = 0;

  x0_drake = x0;%[ x; y; z; roll; pitch; yaw; xdot; ydot; zdot; rolldot; pitchdot; yawdot ]


  %% build drake objects

  p = DeltawingPlant(parameters);


  options.floating = true;
  r = RigidBodyManipulator('TBSC_visualizer.urdf', options);

  v2 = HudBotVisualizer(r);
  %v2 = r.constructVisualizer();


  %% simulate
  
  utraj = utraj.setOutputFrame(p.getInputFrame());
  
  sys = cascade(utraj, p);
  
  xtraj = simulate(sys, utraj.tspan, x0);
  
  
  
  %% visualize

  % combine the simulated trajectory with the inputs

  traj_and_u = [xtraj; utraj];

  fr = traj_and_u.getOutputFrame();

  transform_func = @(t, x, x_and_u) [ x_and_u(1:6); x_and_u(15); x_and_u(13:14); x_and_u(7:12); zeros(3,1)];

  trans = FunctionHandleCoordinateTransform(17, 0, traj_and_u.getOutputFrame(), v2.getInputFrame(), true, true, transform_func, transform_func, transform_func);

  fr.addTransform(trans);


  playback(v2, traj_and_u, struct('slider', true));
  
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


