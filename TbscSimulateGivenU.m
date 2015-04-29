function xtraj = TbscSimulateGivenU(x0, u, parameters)

  % build a PPTrajectory from u data
  
  u_pp = foh(u.logtime, [u.rad.elevonL u.rad.elevonR u.rad.throttle]');
  
  utraj = PPTrajectory(u_pp);

  p = DeltawingPlant(parameters);

  utraj = utraj.setOutputFrame(p.getInputFrame());

  options.floating = true;
  
  %{
  r = RigidBodyManipulator('TBSC_visualizer.urdf', options);

  v2 = HudBotVisualizer(r);
  %v2 = r.constructVisualizer();
  %}


  feedback_system = cascade(utraj, p);

  xtraj = simulate(feedback_system, utraj.tspan, x0);




end