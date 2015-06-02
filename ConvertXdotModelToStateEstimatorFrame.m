function xdot_state_est = ConvertXdotModelToStateEstimatorFrame(x_model_frame, xdot_model_frame)

  % in this case, the only difference between the two frames is the 180
  % degree rotation about the x axis
  
  
  % TODO: CHECKME
  rotm(1:3,1:3) = [ 1,  0,  0;
                    0, -1,  0;
                    0,  0, -1];

  rotm_full = blkdiag(rotm, rotm, rotm, rotm);

  xdot_state_est = rotm_full * xdot_model_frame;

end