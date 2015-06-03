function xtraj_state_est = ConvertXtrajFromDrakeFrameToStateEstFrame(xtraj_drake_frame)

  

  % this loops for each state
  for i = 1 : length(xtraj_drake_frame.pp.breaks)
    
    knot_drake = xtraj_drake_frame.eval(xtraj_drake_frame.pp.breaks(i));
    knots(:,i) = ConvertDrakeFrameToEstimatorFrame(knot_drake);

  end
  
  xtraj_state_est = PPTrajectory(spline(xtraj_drake_frame.pp.breaks, knots));

end