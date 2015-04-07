function x_drake_frame = ConvertStateEstimatorToDrakeFrame(x_est_frame)
  % Converts the 12-dimensional vector for the aircraft's state from
  % the state estimator into the Drake frame.
  %
  % Output state: x, y, z (global frame), roll, pitch, yaw, xdot, ydot,
  % zdot (global frame), rolldot, pitchdot, yawdot
  %
  % @param x_est_frame input state
  %
  % @retval x_drake_frame output state

  x_drake_frame(1:6, :) = x_est_frame(1:6);

  rpy = x_est_frame(4:6);
  UVW = x_est_frame(7:9);

  R_body_to_world = rpy2rotmat(rpy);

  vel_world = R_body_to_world  * UVW;

  x_drake_frame(7:9) = vel_world;

  PQR = x_est_frame(10:12); % in body frame

  pqr = R_body_to_world * PQR; % in world frame

  rpydot = angularvel2rpydot(rpy, pqr);

  x_drake_frame(10:12) = rpydot;


end