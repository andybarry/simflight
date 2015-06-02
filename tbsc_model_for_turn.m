function xdot_model_frame_small = tbsc_model_for_turn(small_state, u, parameters)


  full_state = zeros(12,1);
  
  % small_state:
  %   1: roll
  %   2: pitch
  %   3: xdot
  
  % output:
  %   1: z ddot
  %   2: roll ddot
  %   3: pitch ddot
  
  full_state(4) = small_state(1);
  full_state(5) = small_state(2);
  full_state(7) = small_state(3);
  
  
  full_state = ConvertToModelFrameFromDrakeWorldFrame(full_state);
      
  xdot_model_frame = tbsc_model(0, full_state, u, parameters{:});

  xdot_full = ConvertXdotModelToDrake(full_state, xdot_model_frame);
  

  xdot_model_frame_small = xdot_full(9:11);
  
end