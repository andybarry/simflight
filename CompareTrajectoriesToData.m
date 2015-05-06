clear

parameters = {0.904, 0.000, -0.134, -0.049, 0 };

date = '2015-05-01';
name = 'field-test';
log_number = '04';

trajectory_library = 'trajlib/may-1.mat';



dir = [date '-' name '/gps-logs/'];
filename = ['lcmlog_' strrep(date, '-', '_') '_' log_number '.mat'];



dir_prefix = '/home/abarry/rlg/logs/';
dir = [ dir_prefix dir ];

addpath('/home/abarry/realtime/scripts/logs');
loadDeltawing


load(trajectory_library);


[t_starts, t_ends] = FindActiveTimes(u.logtime, u.is_autonomous, 0.5);

%%
num = 9;

t_start = t_starts(num);
t_end = t_ends(num);

[~, idx] = min(abs(tvlqr_out.logtime - t_start));
this_traj_num = tvlqr_out.trajectory_number(idx);

disp(['Trajectory #: ' num2str(this_traj_num)]);



traj = lib.GetTrajectoryByNumber(this_traj_num);

%%
u = TrimU(t_start, t_end, u);
est = TrimEst(t_start, t_end, est);

%% simulate

disp('Simulating...');
xtrajsim = TbscSimulateGivenU(est.drake_frame(1,:)', u, parameters);
disp('Simulation complete.');


%%
dt = 1/140;
t = 0:dt:traj.xtraj.tspan(2);
trajx = traj.xtraj.eval(t);
traju = traj.utraj.eval(t);
trajsim = xtrajsim.eval(t+est.logtime(1));

%% plot roll

figure(1)
clf

plot(est.logtime, rad2deg(est.orientation.roll))
hold on

traj_roll = trajx(4,:);
plot(t+est.logtime(1), rad2deg(traj_roll), 'r-')

plot(t+est.logtime(1), rad2deg(trajsim(4,:)), 'k');

xlim([t(1)+est.logtime(1) t(end)+est.logtime(1)]);

grid on
xlabel('Time (s)');
ylabel('Roll (deg)');
legend('Actual', 'Planned','Simulated with new model','Location','NorthWest')

%% plot pitch

figure(2)
clf

plot(est.logtime, rad2deg(est.orientation.pitch))
hold on

plot(t+est.logtime(1), rad2deg(trajx(5,:)), 'r-')
plot(t+est.logtime(1), rad2deg(trajsim(5,:)), 'k');

xlim([t(1)+est.logtime(1) t(end)+est.logtime(1)]);
set(gca,'YDir','reverse');
grid on
xlabel('Time (s)');
ylabel('Pitch (deg)');
legend('Actual', 'Planned','Simulated with new model','Location','NorthWest')

%% plot z

figure(3)
clf

plot(est.logtime, est.pos.z);
hold on

plot(t+est.logtime(1), trajx(3,:) + est.pos.z(1), 'r-');
plot(t+est.logtime(1), trajsim(3,:), 'k');

xlim([t(1)+est.logtime(1) t(end)+est.logtime(1)]);


grid on
xlabel('Time (s)');
ylabel('Altitude (m)');
legend('Actual', 'Planned','Simulated with new model','Location','NorthWest')


%% plot u

figure(4)
clf
plot(u.logtime, u.rad.elevonL);
hold on
plot(t+u.logtime(1), traju(1,:),'b--')



plot(u.logtime, u.rad.elevonR, 'm-');
plot(t+u.logtime(1), traju(2,:),'m--')
%plot(u.logtime, u.throttle, 'k-');
legend('elevonL', 'elevonL-plan', 'elevonR', 'elevonR-plan');


xlim([t(1)+est.logtime(1) t(end)+est.logtime(1)]);

