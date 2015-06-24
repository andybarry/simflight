function x_body = ConvertToModelFrameFromDrakeWorldFrame(x_drake)
  
  x_body = x_drake;
  
  x_body(4:6) = x_drake(4:6);
  

  rotm(1:3,1:3) = [ 1,  0,  0;
                    0, -1,  0;
                    0,  0, -1];

  rotm_full = blkdiag(rotm, rotm, rotm, rotm);

  x0_drake_rotated = rotm_full * x_drake;

  x_body(1:6) = x0_drake_rotated(1:6);
  
  % body position is always at origin in body frame
  
  %x_body(1) = 0;
  %x_body(2) = 0;
  %x_body(3) = 0;

  rpy = [ x0_drake_rotated(4); x0_drake_rotated(5); x0_drake_rotated(6)];
  rpydot = x0_drake_rotated(10:12);
  R_body_to_world = rpy2rotmat(rpy);
  R_world_to_body = R_body_to_world';
  R = R_body_to_world;

  x_body(7:9) = R_world_to_body*x0_drake_rotated(7:9);

  phi = rpy(1);
  theta = rpy(2);
  psi = rpy(3);

  phidot = rpydot(1);
  thetadot = rpydot(2);
  psidot = rpydot(3);

  pqr = rpydot2angularvel([phi;theta;psi],[phidot;thetadot;psidot]);
  pqr = R'*pqr;

  x_body(10:12) = pqr;

end

