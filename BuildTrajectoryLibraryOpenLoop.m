
clear

lib = TrajectoryLibrary();

parameters = { 1.92, 1.84, 2.41, 0.48, 0.57 };

p = DeltawingPlant(parameters);

throttle_max = 5.33976;

%% straight ahead

tf = 0.5;

u0 = [0.2, 0.2, 0.9 * throttle_max]';

utraj = ConstantTrajectory(u0);
utraj = utraj.setOutputFrame(p.getInputFrame());

xtraj = runInputTape(p, utraj, tf);

lib = lib.AddTrajectory(xtraj, utraj);



%% turn left

u0 = [0.6 -0.2 0.9 * throttle_max]';

utraj = ConstantTrajectory(u0);
utraj = utraj.setOutputFrame(p.getInputFrame());

xtraj = runInputTape(p, utraj, tf);

lib = lib.AddTrajectory(xtraj, utraj);

%% go up

u0 = [0.3 0.3 1 * throttle_max]';

utraj = ConstantTrajectory(u0);
utraj = utraj.setOutputFrame(p.getInputFrame());

xtraj = runInputTape(p, utraj, tf);

lib = lib.AddTrajectory(xtraj, utraj);

%% draw

lib.DrawTrajectories();