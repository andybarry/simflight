function xtrajsim = SimulateTrajectoryAgainstData(u, est, parameters, t_start, t_end)

  
  u = TrimU(t_start, t_end, u);
  est = TrimEst(t_start, t_end, est);


  disp(['Simulating, t = ' num2str(t_start) '...']);
  xtrajsim = TbscSimulateGivenU(est.drake_frame(1,:)', u, parameters);
  disp('Simulation complete.');


end