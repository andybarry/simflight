function xdot_drake = ConvertXdotModelToDrake(x_model, xdot_model)

  rpy = x_model(4:6);
  
  U = x_model(7);
  V = x_model(8);
  W = x_model(9);
  P = x_model(10);
  Q = x_model(11);
  R = x_model(12);
  R_body_to_world = rpy2rotmat(rpy);
  
  xdot_world_ani = ConvertToWorldCoords(xdot_model, R_body_to_world, rpy, [U;V;W], [P;Q;R]);

  rotm(1:3,1:3) = [ 1,  0,  0;
                    0, -1,  0;
                    0,  0, -1];

  rotm_full = blkdiag(rotm, rotm, rotm, rotm);

  xdot_drake = rotm_full * xdot_world_ani;

  
end


function xdot_world = ConvertToWorldCoords(xdot, R_body_to_world, rpy, uvw, pqr)
  
  xyzdot = xdot(1:3);
  rpydot = xdot(4:6);
  UVW_dot = xdot(7:9);
  pqr_dot = xdot(10:12);

  phi = rpy(1);
  theta = rpy(2);
  psi = rpy(3);
  
  phidot = rpydot(1);
  thetadot = rpydot(2);
  psidot = rpydot(3);
  
  R = R_body_to_world;
  
  
  % Now, convert pqr_dot to rpy_ddot
  [Phi, dPhi] = angularvel2rpydotMatrix([phi;theta;psi]);

  Rdot =  [ 0, sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta),   cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta); ...
            0, cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi), - cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta); ...
            0,                              cos(phi)*cos(theta),                               -cos(theta)*sin(phi)]*phidot + ...
            [ -cos(psi)*sin(theta), cos(psi)*cos(theta)*sin(phi), cos(phi)*cos(psi)*cos(theta); ...
              -sin(psi)*sin(theta), cos(theta)*sin(phi)*sin(psi), cos(phi)*cos(theta)*sin(psi); ...
                      -cos(theta),         -sin(phi)*sin(theta),         -cos(phi)*sin(theta)]*thetadot + ...
                     [ -cos(theta)*sin(psi), - cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta), cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta); ...
                       cos(psi)*cos(theta),   cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta); ...
                                         0,                                                  0,                                                0]*psidot;

  rpy_ddot = Phi*R*pqr_dot + reshape((dPhi*[phidot;thetadot;psidot]),3,3)*R*pqr + ...
             Phi*Rdot*pqr;

  xyz_ddot_world = R_body_to_world*UVW_dot + Rdot * uvw;
           
  xdot_world = [xyzdot; rpydot; xyz_ddot_world; rpy_ddot];

                     
end
