% Tune gains

traj_num = 1;
traj = lib3.GetTrajectoryByNumber(traj_num);

Q = diag([10 10 10 10 50 .25 0.1 .0001 0.0001 .1 .05 .1]);

R_values = [150];
    
[~, gains] = GetDefaultGains();


gains.Q = Q;
gains.R_values = R_values;

lib_gains = TrajectoryLibrary.RebuildTvControllers(lib2, gains);


x0 = traj.xtraj.eval(0);
tf = traj.xtraj.tspan(end);
xf = traj.xtraj.eval(tf);
% 
% disturbances = zeros(12,3);
% 
% 
% disturbances(1:3,:) = 10*rand(3,3) - 5;
% disturbances(4:6,:) = 1*rand(3,3)-.5;
% 
% disturbances(7:9,:) = 5*rand(3,3)-2.5;
% disturbances(10:12,:) = 10*rand(3,3)-5;
% 
disturbances = [
         0
         0
         0
    0.1318
   -0.3735
         0
   -1.4122
   -1.2448
    1.9646
    2.0672
    0.5779
   -1.8657];

disp(disturbances);


init_cond = [ x0+disturbances(:,1)];%, x0+disturbances(:,2), x0+disturbances(:,3) ];


for i = 1 : 1%size(init_cond,2)
tic  
  xtraj = lib_gains.SimulateTrajectory(traj_num, tf, init_cond(:,i));
  
  error(i) = norm(xf-xtraj.eval(tf))
toc
end

