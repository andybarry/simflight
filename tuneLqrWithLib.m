traj = lib.GetTrajectoryByNumber(4);

[parameters, gains] = GetDefaultGains();

xyzgain = 1;

gains.Q(1,1) = xyzgain;
gains.Q(2,2) = xyzgain;
gains.Q(3,3) = xyzgain;
gains.R_values = 150;

lib = AddLqrControllersToLib('testing', lib, traj.xtraj, traj.utraj, gains);


lib.SimulateTrajectory(40, 1, traj.xtraj.eval(0));