%% compute controller

traj = lib.GetTrajectoryByNumber(4);

[parameters, gains] = GetDefaultGains();

xyzgain = 10;

gains.Q(1,1) = xyzgain;
gains.Q(2,2) = xyzgain;
gains.Q(3,3) = xyzgain;
gains.R_values = 150;



lib = AddLqrControllersToLib('testing', lib, traj.xtraj, traj.utraj, gains, true);

trajnum = length(lib.trajectories) - 1;


lib.GetTrajectoryByNumber(trajnum).draw();

%% simulate

x0 = ConvertStateEstimatorToDrakeFrame(traj.xtraj.eval(0));

x0(2) = x0(2) - 2;
x0(3) = x0(3) + 2;

x0(4) = x0(4) + deg2rad(15);

lib.SimulateTrajectory(trajnum, traj.xtraj.tspan(2), x0);