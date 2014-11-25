function x_drake = ConvertToDrakeFrameFromModelFrame(x_model)
  % Converts deltawing model frame to drake frame
  
  rotm(1:3,1:3) = [ 1,  0,  0;
                    0, -1,  0;
                    0,  0, -1];

  rotm_full = blkdiag(rotm, rotm, rotm, rotm);
  
  x_model_rotated = rotm_full * x_model;
  
  x_drake(1:6,:) = x_model_rotated(1:6);
  
  rpy = [ x_model_rotated(4); x_model_rotated(5); x_model_rotated(6)];
  R_body_to_world = rpy2rotmat(rpy);
  
  x_drake(7:9,:) = R_body_to_world * x_model_rotated(7:9);
  
  
  rpydot = angularvel2rpydot(rpy, R_body_to_world*x_model_rotated(10:12));
  
  x_drake(10:12,:) = rpydot;
  
end