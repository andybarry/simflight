function x_drake_frame = ConvertStateEstimatorToDrakeFrame(x_est_frame, Mz)
  % Converts the 12-dimensional vector for the aircraft's state from
  % the state estimator into the Drake frame.
  %
  % Output state: x, y, z (global frame), roll, pitch, yaw, xdot, ydot,
  % zdot (global frame), rolldot, pitchdot, yawdot
  %
  % @param x_est_frame input state
  % @param Mz (optional) rotation around the Z axis (a yaw) to apply
  %   @default eye(3)
  %
  % @retval x_drake_frame output state

  if nargin < 2
    Mz = eye(3);
  end
  
  x_drake_frame = x_est_frame;
  x_drake_frame(1:3) = Mz * x_est_frame(1:3);
  
  rot_mat = rpy2rotmat(x_est_frame(4:6));
  rpy_out = rotmat2rpy(Mz * rot_mat);
  
  x_drake_frame(4:6) = rpy_out;

  rpy = x_est_frame(4:6);
  UVW = x_est_frame(7:9);

  R_body_to_world = rpy2rotmat(rpy);

  vel_world = Mz * R_body_to_world  * UVW;

  x_drake_frame(7:9) = vel_world;

  PQR = x_est_frame(10:12); % in body frame

  pqr = Mz * R_body_to_world * PQR; % in world frame

  rpydot = angularvel2rpydot(rpy, pqr);

  x_drake_frame(10:12) = rpydot;


end