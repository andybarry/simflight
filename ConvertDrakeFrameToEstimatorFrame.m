function x_est_frame = ConvertDrakeFrameToEstimatorFrame(x_drake_frame)
  % Converts the 12-dimensional vector for the aircraft's state into
  % one that works for the state esimator and should be exported and
  % used for online control
  %
  % Output state: x, y, z (global frame), roll, pitch, yaw, xdot, ydot,
  % zdot (body frame), angular velocity (3 numbers)
  %
  % @param x_drake_frame input state
  %
  % @retval x_est_frame output state

  % global position stays in the global frame

  % Drake frame:
  % x(1:3): x,y,z, in global frame (ENU coordinates)
  % x(4:6): rpy
  % x(7:9): xdot, ydot, zdot, in global frame (ENU coordinates)
  % x(10:12): rdot, pdot, ydot

  % State estimator frame:
  %
  % double pos[3];              // position x,y,z in meters in local frame (ENU coordinates)
  % double vel[3];              // velocity in m/s, expressed in body frame
  % 
  % double orientation[4];      // rotate vector in body coordinate frame 
  %                             // (X-forward, Z-down) by this to get that vector
  %                             // in local frame
  % 
  % double rotation_rate[3];    // angular velocity vector of the vehicle
  %                             // in rad/s.  This is expressed in the body
  %                             // frame.

  % get the rotation matrix for this rpy

  % Compute U,V,W from xdot,ydot,zdot

  x_est_frame(1:6,:) = x_drake_frame(1:6);

  rpy = x_drake_frame(4:6);
  xdot = x_drake_frame(7);
  ydot = x_drake_frame(8);
  zdot = x_drake_frame(9);

  rolldot = x_drake_frame(10);
  pitchdot = x_drake_frame(11);
  yawdot = x_drake_frame(12);

  R_body_to_world = rpy2rotmat(rpy);
  R_world_to_body = R_body_to_world';
  UVW = R_world_to_body*[xdot;ydot;zdot];

  % Compute P,Q,R (angular velocity components)
  pqr = rpydot2angularvel(rpy,[rolldot;pitchdot;yawdot]); % in world frame
  PQR = R_world_to_body*pqr; % body coordinate frame

  x_est_frame(7:9) = UVW;

  x_est_frame(10:12) = PQR;

end
