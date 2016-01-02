%% bar graph of trajectories executed / flight

% for i = 1 : length(logs)
%   disp(i)
%   [t_start, t_end] = FindAutonomousFlight(logs(i))
% end
% 
% 

counter = zeros(1,7);

tvlqr_many = [];
% figure out which trajectories were used in each flight
clf
for i = 1 : length(logs)
  
  tvlqr = TrimTvlqr(logs(i).t_start, logs(i).t_end, logs(i).tvlqr_out);
  tvlqr_many = [tvlqr_many; tvlqr.trajectory_number];
  %disp([logs(i).date '.' logs(i).log_number]);
  
  %plot(tvlqr.logtime - logs(i).t_start, tvlqr.trajectory_number,'-')
  %hold on
  
  for j = 1 : length(tvlqr.trajectory_number)
    counter(tvlqr.trajectory_number(j)+1) = counter(tvlqr.trajectory_number(j)+1) + 1;
  end
  
end

bar(counter/length(logs))
grid on

%hist(tvlqr_many+1, 25)
%set(gca, 'XTickLabelMode', 'manual');
set(gca, 'XTick', [1 2 3 4 5 6 7]);

set(gca, 'XTickLabel', {'Level flight', 'Climb', 'Glide', 'Gentle left', 'Gentle right', 'Left jog', 'Right jog'});

%[n,x] = hist(tvlqr_many+1, 7);

%barstrings = num2str(n');
%text(x,n,barstrings,'horizontalalignment','center','verticalalignment','bottom')
xlim([.5 7.5])
xlabel('Trajectory');
ylabel('Number of Times Executed per Flight');

%% sum up all trajectories
disp(['Total number of trajectories executed: ' num2str(sum(counter))]);

total_stereo = 0;
for i = 1 : length(logs)
  this_stereo = TrimStereo(logs(i).t_start, logs(i).t_end, logs(i).stereo);
  
  total_stereo = total_stereo + sum(this_stereo.number_of_points);
  
end

disp(['Total number of stereo points seen: ' num2str(total_stereo)]);


% total distance flown
total_dist = 0;
for i = 1 : length(logs)
  total_dist = total_dist + ComputeTotalDistance(logs(i).t_start, logs(i).t_end, logs(i).est);
end

disp(['Total distance flown: ' num2str(total_dist) 'm']);


% average speed
speed_count = 0;
speed_val = 0;

for i = 1 : length(logs)
  
  trim_est = TrimEst(logs(i).t_start, logs(i).t_end, logs(i).est);
  
  for j = 1 : length(trim_est.logtime)
    speed_count = speed_count + 1;
    speed_val = speed_val + sqrt(trim_est.vel.x(j)*trim_est.vel.x(j) + trim_est.vel.y(j)*trim_est.vel.y(j) + trim_est.vel.z(j)*trim_est.vel.z(j));
  end
  
end

mean_speed = speed_val / speed_count;

disp(['Mean speed: ' num2str(mean_speed) 'm/s']);

% total time in the air
total_t = 0;
for i = 1 : length(logs)
  total_t = total_t + logs(i).t_end - logs(i).t_start;
end

disp(['Time in autonomous mode: ' num2str(total_t) 's']);

% max G
for i = 1 : length(logs)
  trim_est = TrimEst(logs(i).t_start, logs(i).t_end, logs(i).est);
  
  max_x = max(abs(trim_est.accel.x));
  max_y = max(abs(trim_est.accel.y));
  max_z = max(abs(trim_est.accel.z));
end

disp(['Max G: ' num2str(max_x/9.81) ' (x), ' num2str(max_y/9.81) ' (y), ' num2str(max_z/9.81)]);