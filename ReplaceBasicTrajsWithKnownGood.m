function lib2 = ReplaceBasicTrajsWithKnownGood(lib2)

  trajectory_library = 'traj-archive/sept16.mat';
  disp(['Loading ' trajectory_library '...']);
  load(trajectory_library);

  lib2 = lib2.RemoveTrajectroy(0:2);
  lib2 = lib2.InsertExistingTrajectory(lib.GetTrajectoryByNumber(0), 0);
  lib2 = lib2.InsertExistingTrajectory(lib.GetTrajectoryByNumber(1), 1);
  lib2 = lib2.InsertExistingTrajectory(lib.GetTrajectoryByNumber(2), 2);

end