%% compute controller

traj = lib.GetTrajectoryByNumber(4);

[parameters, gains] = GetDefaultGains();

% xyzgain = 10;
% 
% gains.Q(1,1) = xyzgain;
% gains.Q(2,2) = xyzgain;
% gains.Q(3,3) = xyzgain;
% gains.R_values = 150;



lib = AddLqrControllersToLib('testing', lib, traj.xtraj, traj.utraj, gains, true);

trajnum = length(lib.trajectories) - 1;


lib.GetTrajectoryByNumber(trajnum).draw();

%% simulate

x0 = ConvertStateEstimatorToDrakeFrame(traj.xtraj.eval(0));

x0(1:3) = x0(1:3) + rand(3,1)*1;
x0(4:6) = x0(4:6) + rand(3,1)*.1;
x0(7:9) = x0(7:9) + rand(3,1)*1;
x0(10:12) = x0(10:12) + rand(3,1)*.1;

x0(11) = x0(11) - 10

[ytrajsim, xtrajsim, utrajsim] = lib.SimulateTrajectory(trajnum, traj.xtraj.tspan(2), x0);