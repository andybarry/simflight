% Generates test cases for many trajectories and many points
% for testing C++ code

% load a visualizer
parameters = GetDefaultGains();
p = DeltawingPlant(parameters);
p.constructVisualizer;

disp('Loading trajectory library...');
load('TrajectoryTesting/june13.mat');
disp('done.');



%%
disp('----------------');

%points = 30*rand(3, 30) - 15;
%points(1,:) = points(1,:) + 20;

%save('TrajectoryTesting/points.mat', 'points');
load TrajectoryTesting/points.mat

%points = points(:,1:1)

% plot points
lcmgl = LCMGLClient('random-points');
lcmgl.glColor3f(0,0,1);
for i = 1 : size(points,2)
  lcmgl.sphere(points(:, i), 0.25, 20, 20);
end

lcmgl.switchBuffers();



threshold = 50;

% run nearest neighbor on each trajectory we want to use

trajs_to_use = [0, 6, 18, 24];

max_dist = -1;
best_traj = -1;

for i = 1 : length(trajs_to_use)
  
  disp(['Checking trajectory ' num2str(trajs_to_use(i)) '...']);
  
  traj = lib.GetTrajectoryByNumber(trajs_to_use(i));
  
  dist = traj.NearestNeighborLinear(points)
  
  if (max_dist < 0 || dist > max_dist)
    max_dist = dist;
    best_traj = trajs_to_use(i);
    
    if ( max_dist > threshold )
      disp('exiting because we found a good enough trajectory');
      break;
    end
    
  end
  
  
end
  
max_dist
best_traj

traj = lib.GetTrajectoryByNumber(best_traj);

options = struct();
options.color = [0 1 0];
traj.draw(options);

lib2 = TrajectoryLibrary(lib.p);


for i = 1 : length(trajs_to_use)

  traj = lib.GetTrajectoryByNumber(trajs_to_use(i));
  
  if i == 4
    traj.name = 'climb';
  end
  
  
  lib2 = lib2.AddExistingTrajectory(traj);
  
  
end

lib2.DrawTrajectories();