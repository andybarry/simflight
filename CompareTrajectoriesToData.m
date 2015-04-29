clear


date = '2015-04-21';
name = 'field-test';
log_number = '03';

trajectory_library = 'trajlib/april-23.mat';



dir = [date '-' name '/gps-logs/'];
filename = ['lcmlog_' strrep(date, '-', '_') '_' log_number '.mat'];



dir_prefix = '/home/abarry/rlg/logs/';
dir = [ dir_prefix dir ];

addpath('/home/abarry/realtime/scripts/logs');
loadDeltawing


load(trajectory_library);


[t_starts, t_ends] = FindActiveTimes(u.logtime, u.is_autonomous, 0.5);

%%
t_start = t_starts(1);
t_end = t_ends(1);
traj = lib.trajectories{10};

%%
u = TrimU(t_start, t_end, u);
est = TrimEst(t_start, t_end, est);

%%
dt = 1/140;
t = 0:dt:traj.xtraj.tspan(2);
trajx = traj.xtraj.eval(t);

%% plot roll

figure(1)
clf

plot(est.logtime, rad2deg(est.orientation.roll))
hold on

traj_roll = trajx(4,:);
plot(t+est.logtime(1), rad2deg(traj_roll), 'r-')

xlim([t(1)+est.logtime(1) t(end)+est.logtime(1)]);

grid on
xlabel('Time (s)');
ylabel('Roll (deg)');
legend('Actual', 'Planned','Location','NorthWest')

%% plot pitch

figure(2)
clf

plot(est.logtime, rad2deg(est.orientation.pitch))
hold on

plot(t+est.logtime(1), rad2deg(trajx(5,:)), 'r-')

xlim([t(1)+est.logtime(1) t(end)+est.logtime(1)]);
set(gca,'YDir','reverse');
grid on
xlabel('Time (s)');
ylabel('Pitch (deg)');
%legend('Actual', 'Planned','Location','NorthWest')

%% plot z

figure(3)
clf

plot(est.logtime, est.pos.z);
hold on

plot(t+est.logtime(1), trajx(3,:) + est.pos.z(1), 'r-');

xlim([t(1)+est.logtime(1) t(end)+est.logtime(1)]);

grid on
xlabel('Time (s)');
ylabel('Altitude (m)');
%legend('Actual', 'Planned','Location','NorthWest')




