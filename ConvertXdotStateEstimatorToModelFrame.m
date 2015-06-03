function xdot_model_frame = ConvertXdotStateEstimatorToModelFrame(xdot_state_est_frame)

  % in this case, the only difference between the two frames is the 180
  % degree rotation about the x axis
  
  rotm(1:3,1:3) = [ 1,  0,  0;
                    0, -1,  0;
                    0,  0, -1];

  rotm_full = blkdiag(rotm, rotm, rotm, rotm);
  xdot_model_frame = rotm_full * xdot_state_est_frame;

end