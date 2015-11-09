function state = PoseToStateEstimatorVector(est, index, substract_positions, Mz)
  % This mirrors RealtimeUtils.cpp's PoseMsgToStateEstimatorVector
  
  state = zeros(12,1);
  
  pos(1) = est.pos.x(index) - substract_positions(1);
  pos(2) = est.pos.y(index) - substract_positions(2);
  pos(3) = est.pos.z(index) - substract_positions(3);

  % // x, y, and z are always in global frame (add in any custom rotation)
  pos_global = Mz * pos';

  state(1) = pos_global(1);
  state(2) = pos_global(2);
  state(3) = pos_global(3);

  % // roll, pitch, and yaw come from the quats in the message
  
  q(1) = est.orientation.q0(index);
  q(2) = est.orientation.q1(index);
  q(3) = est.orientation.q2(index);
  q(4) = est.orientation.q3(index);

  rot_mat = quat2rotmat(q');
  
  rpy = rotmat2rpy(Mz * rot_mat);

  state(4) = rpy(1);
  state(5) = rpy(2);
  state(6) = rpy(3);

  % // velocities are in body frame
  state(7) = est.vel.x(index);
  state(8) = est.vel.y(index);
  state(9) = est.vel.z(index);

  % // rotation rates are given in body frame
  %  // as angular velocities

  state(10) = est.rotation_rate.x(index);
  state(11) = est.rotation_rate.y(index);
  state(12) = est.rotation_rate.z(index);

end