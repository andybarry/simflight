function xdot_small = tbsc_model_for_turn(small_state, u, parameters)


  full_state = zeros(12,1);
  
  % small_state:
  %   1: roll
  %   2: pitch
  %   3: xdot
  %   4: ydot
  
  % output:
  %   1: z ddot
  %   2: roll ddot
  %   3: pitch ddot
  %   4: yaw ddot
  
  
  
  full_state(4) = small_state(1);
  full_state(5) = small_state(2);
  full_state(7) = small_state(3);
  full_state(8) = small_state(4);
  
  full_state = ConvertToModelFrameFromDrakeWorldFrame(full_state);
  
  
  xdot_model_frame = tbsc_model(0, full_state, u, parameters{:});

  
  %xdot_full_est = ConvertXdotModelToStateEstimatorFrame(xdot_model_frame);
  xdot_full_drake = ConvertXdotModelToDrake(full_state, xdot_model_frame);
  

  xdot_small = xdot_full_drake(9:12);
  
end