traj = lib.GetTrajectoryByNumber(4);

[parameters, gains] = GetDefaultGains();

xyzgain = 0.01;

gains.Q(1,1) = xyzgain;
gains.Q(2,2) = xyzgain;
gains.Q(3,3) = xyzgain;
gains.R_values = 150;

lib = AddLqrControllersToLib('testing', lib, traj.xtraj, traj.utraj, gains);

trajnum = length(lib.trajectories) - 1;


lib.GetTrajectoryByNumber(trajnum).draw();

lib.SimulateTrajectory(trajnum, 1);