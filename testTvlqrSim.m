
trajnum = 4;

traj = lib.GetTrajectoryByNumber(trajnum);


xf = traj.xtraj.eval(traj.xtraj.tspan(end));


[ytraj, xtraj, utraj] = lib.SimulateTrajectory(trajnum);

xf_sim = xtraj.eval(traj.xtraj.tspan(end));

