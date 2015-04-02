function [xdot, y] = tbsc_model(t,x,u, Jx_fac, Jy_fac, Jz_fac, elev_lift_fac, elev_drag_fac, body_x_drag_fac, varargin)
% Model derived from Ani's SBach model

% Set output (first six states)
y = x(1:7);


% @param t time
% @param x state: x =
%  Plane's X coordinate faces forward, Y to the right, and Z down.
%  x(1):x    (Forward Position, Earth frame)
%  x(2):y    (East or y position, Earth frame)
%  x(3):z     (z position (down), Earth frame)
%  x(4):phi   (roll angle)
%  x(5):theta (pitch angle)
%  x(6):psi   (yaw angle)
%  x(7):U     (X-velocity, body frame)
%  x(8):V     (Y-velocity, body frame)
%  x(9):W     (Z-velocity, body frame)
%  x(10):P    (Angular velocity in X direction, body frame)
%  x(11):Q    (Angular velocity in Y direction, body frame)
%  x(12):R    (Angular velocity in Z direction, body frame)
%
%  u(1):elevL   (Elevon left Command, radians)
%  u(2):elevR   (Elevon right Command, radians)
%  u(3):thr   (Throttle command, newtons)

% TODO: ROLL DAMPENING
% TODO: MOMENT COEFFICIENT

%% Parameters fit from data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Rotational inertias
%{
 Jx_fac = 0.75;
 Jy_fac = 3.91;
 Jz_fac = 1.50;

% Elevator
elev_lift_fac = 3.83;
elev_drag_fac = 1.09;
%}

elevL_lift_fac = elev_lift_fac;
elevR_lift_fac = elev_lift_fac;

elevL_drag_fac = elev_drag_fac;
elevR_drag_fac = elev_drag_fac;
% 
%  elevL_lift_fac = elev_lift_fac;
%  elevR_lift_fac = elev_lift_fac;
% % 
%  elevL_drag_fac = elev_drag_fac;
%  elevR_drag_fac = elev_drag_fac;

% Throttle/propwash
thr_fac = 1;

% Body drag
% body_x_drag_fac = 0;
body_y_drag_fac = 0;
body_z_drag_fac = 0;

% Rate dependent force
F_Q_fac_x = 0;
F_Q_fac_z = 0;

% Throttle aerodynamic drag
thr_drag_fac = 0; % TODO: roll from throttle

% Rate dependent moments
M_P_fac = 0;
M_Q_fac = 0;
M_R_fac = 0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Rotational inertias
Jx = Jx_fac*0.014894; % The numbers are from the solidworks model
Jy = Jy_fac*0.005580;
Jz = Jz_fac*0.019316; % TODO: include cross terms from solidworks?

J = diag([Jx,Jy,Jz]);
invJ = diag([1/Jx,1/Jy,1/Jz]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Measured parameters (MKS units):
%wing_area = 0.18109692;% m^2
wing_span = 0.8636; % m
wing_chord = 0.2097;
wing_area = wing_span * wing_chord;
%out_dist = 132.8/1000; % Moment arm of outer wing section
%in_dist = 46.4/1000; % Moment arm of inner wing section
%ail_out_dist_x = 41/1000; % These are behind the COM 
%ail_in_dist_x = 65/1000; % These are behind the COM
elevL_area = 0.01250823; % m^2
elevR_area = 0.01250823;

elevL_x_arm = -0.12495; % position of the elevon in meters in the x-axis (how far forward or back it is)
elevR_x_arm = -0.12495;

elevL_y_arm = -0.276225; % position of the elevon's center in meters on the y-axis (how far left/right it is)
elevR_y_arm = 0.276225;

elevL_chord = 0.0402;
elevR_chord = 0.0402;

wingletL_area = 0.01944076;
wingletR_area = 0.01944076;

wingletL_body_pos = [ -0.08; -0.4318; -0.081954 ];
wingletR_body_pos = [ -0.08;  0.4318; -0.081954 ];

%rudder_area = (3.80152*1000)/(1000*1000);
%rudder_arm = 258.36/1000; % m
%stab_area = 923.175/(1000*1000); % m^2
%stab_arm = 222/1000; % m
m =  0.648412; % kg (with battery in)
g = 9.81; % m/s^2
thr_to_thrust = 1; % throttle comes in in newtons now ... was: 0.05159; % kg per unit throttle command

elevL_comm_to_rad = 1;
elevR_comm_to_rad = 1;

thr_min = 0; % if throttle is below this value, it is zero (below threshold to run prop)

throttle_trim = 0; %  in newtons
elevL_trim = 0; % in radians
elevR_trim = 0; % in radians

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get states
rpy = x(4:6);
% phi = rpy(1);
% theta = rpy(2);
% psi = rpy(3);
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
U = x(7);
V = x(8);
W = x(9);
P = x(10);
Q = x(11);
R = x(12);

R_body_to_world = rpy2rotmat(rpy);
R_world_to_body = R_body_to_world';

% COM velocity in world coordinates
xdots_world = R_body_to_world*[U;V;W];

% Angular velocity in world coordinate frame
omega_world = R_body_to_world*[P;Q;R];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get control inputs

%Throttle
thr = u(3) - throttle_trim;
if thr < (thr_min - throttle_trim)
    thr = 0;
end
% thr = thr_fac*max(thr,0); % Scale it and don't let it go negative

% positive AilL is negative lift (front of aileron tips downwards)
elevL = (u(1)-elevL_trim)*elevL_comm_to_rad; % input in radians of deflection
elevR = (u(2)-elevR_trim)*elevR_comm_to_rad; % input in radians of deflection

%positive elevator is deflection up (negative lift - makes plane
%pitch up)
%elev = (u(3)-elev_trim)*elev_comm_to_rad; % input in radians of deflection

%positive rudder is deflection to the right
%rud = (u(4)-rud_trim)*rud_comm_to_rad;% input in radians of deflection
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Now, the hard stuff

% Do translataional stuff first

% Speed of plane (does not includ V^2 since that is sideslip which is not
% used in the flatplate model
vel_uw = sqrt(U^2 + W^2);


% Angle of attack
alpha = atan2(W,U);


%%Forces due to wings and elevons%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Lift force generated by wing components. (flat plate)
%lift = dynamic pressure * area * Coefficient of lift.
wing_lift = pressure(vel_uw) * wing_area * Cl_fp(alpha);



%Drag force generated by wing components.
wing_drag = pressure(vel_uw) * wing_area * Cd_fp(alpha);

% Collect these forces and represent them in correct frame
F_wing = rotAlpha([wing_drag; 0; wing_lift], alpha);

% Roll drag force

%M_wing_roll_drag = -sign(P) * 1/6 * 1.204 * Cd_fp(pi/2) * wing_chord * wing_span^3 * P^2

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Forces due to elevons

% Propwash over rudder/elevator (0.1444 was fit from anemometer
% experiments)
%upe = sqrt((vel^2)/4 + thr_to_sp_elev*0.1444*thr) - vel/2;

%Lift force generated by elevons. (flat plate)
%lift = dynamic pressure * area * Coefficient of lift.
% Velocity of elevator in world coordinate frame

elevL_body_pos = [elevL_x_arm + (elevL_chord/2 - elevL_chord/2*cos(elevL)); elevL_y_arm; -elevL_chord/2*sin(elevL)];
elevR_body_pos = [elevR_x_arm + (elevR_chord/2 - elevR_chord/2*cos(elevR)); elevR_y_arm; -elevR_chord/2*sin(elevR)];


%xdot_elevL = xdots_world + cross(omega_world, R_body_to_world*elevL_body_pos);
%xdot_elevL_body = R_world_to_body*xdot_elevL;
xdot_elevL_body = [U;V;W] + cross([P;Q;R], elevL_body_pos);
xdot_elevL_body_no_sideslip = [xdot_elevL_body(1); 0; xdot_elevL_body(3)];
vel_elevL = norm(xdot_elevL_body_no_sideslip);

%xdot_elevR = xdots_world + cross(omega_world, R_body_to_world*elevR_body_pos);
%xdot_elevR_body = R_world_to_body*xdot_elevR;
xdot_elevR_body = [U;V;W] + cross([P;Q;R], elevR_body_pos);
xdot_elevR_body_no_sideslip = [xdot_elevR_body(1); 0; xdot_elevR_body(3)];
vel_elevR = norm(xdot_elevR_body_no_sideslip);

alpha_elevL = atan2(xdot_elevL_body_no_sideslip(3), xdot_elevL_body_no_sideslip(1));
alpha_elevR = atan2(xdot_elevR_body_no_sideslip(3), xdot_elevR_body_no_sideslip(1));

%include lift terms from flat plate theory of elevator
elevL_lift = elevL_lift_fac*pressure(vel_elevL) * elevL_area * ... likely a small term
    Cl_fp(alpha_elevL-elevL); %angle of deflection of elevon
  
elevR_lift = elevR_lift_fac*pressure(vel_elevR) * elevR_area * ... likely a small term
    Cl_fp(alpha_elevR-elevR); %angle of deflection of elevon

%Compute drag on elevator using flat plate theory
elevL_drag = elevL_drag_fac * pressure(vel_elevL) * elevL_area * Cd_fp(alpha_elevL-elevL);
elevR_drag = elevR_drag_fac * pressure(vel_elevR) * elevR_area * Cd_fp(alpha_elevR-elevR);

% Rotate to correct frame
F_elevL = rotAlpha([elevL_drag; 0; elevL_lift], alpha_elevL); % elevon left
F_elevR = rotAlpha([elevR_drag; 0; elevR_lift], alpha_elevR); % elevon right
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Rudder and stabilizer

%Stabilizing force from aircraft yawing (this is a crude approximation)
%F_stabilizer = [0;-stab_force_fac*sign(R) * pressure(R*stab_arm) * (stab_area+rudder_area);0];

% Sideslip angle for rudder
%beta_rud = atan2(V,sqrt(vel^2-V^2) + upe); % Assuming propwash over rudder is same as elevator

% winglet forces (flat plates)



%xdot_wingletL = xdots_world + cross(omega_world, R_body_to_world * wingletL_body_pos);
%xdot_wingletL_body = R_world_to_body*xdot_wingletL;
xdot_wingletL_body = [U;V;W] + cross([P;Q;R], wingletL_body_pos);
xdot_wingletL_body_no_sideslip = [xdot_wingletL_body(1); xdot_wingletL_body(2); 0];
vel_wingletL = norm(xdot_wingletL_body_no_sideslip);

%xdot_wingletR = xdots_world + cross(omega_world, R_body_to_world * wingletR_body_pos);
%xdot_wingletR_body = R_world_to_body*xdot_wingletR;
xdot_wingletR_body = [U;V;W] + cross([P;Q;R], wingletR_body_pos);
xdot_wingletR_body_no_sideslip = [xdot_wingletR_body(1); xdot_wingletR_body(2); 0];
vel_wingletR = norm(xdot_wingletR_body_no_sideslip);

beta_wingletL = atan2(xdot_wingletL_body_no_sideslip(2), xdot_wingletL_body_no_sideslip(1));
beta_wingletR = atan2(xdot_wingletR_body_no_sideslip(2), xdot_wingletR_body_no_sideslip(1));

wingletL_lift = pressure(vel_wingletL) * wingletL_area * Cl_fp(beta_wingletL);
wingletL_drag = pressure(vel_wingletL) * wingletL_area * Cd_fp(beta_wingletL);

wingletR_lift = pressure(vel_wingletR) * wingletR_area * Cl_fp(beta_wingletR);
wingletR_drag = pressure(vel_wingletR) * wingletR_area * Cd_fp(beta_wingletR);

wingletL_force = [wingletL_drag; wingletL_lift; 0];
wingletR_force = [wingletR_drag; wingletR_lift; 0];

F_wingletL = [-cos(beta_wingletL), sin(beta_wingletL), 0; -sin(beta_wingletL), -cos(beta_wingletL), 0;0 0 0]*wingletL_force;
F_wingletR = [-cos(beta_wingletR), sin(beta_wingletR), 0; -sin(beta_wingletR), -cos(beta_wingletR), 0;0 0 0]*wingletR_force;

%rudder_force = [rudder_force_d;rudder_force_l;0];

% Rotate to correct frame
%F_rudder = [-cos(beta_rud), sin(beta_rud), 0; -sin(beta_rud), -cos(beta_rud), 0;0 0 0]*rudder_force;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gravity, thrust, and body drag
F_gravity = R_world_to_body*[0;0;m*g];

F_thrust = [thr_fac*thr_to_thrust*thr; 0; 0]; % thr_to_thrust was estimated from digital scale experiments

% Drag due to body (in x-direction)
body_drag_x =  body_x_drag_fac*pressure(U);

% Drag due to body in y-direction
body_drag_y = body_y_drag_fac * pressure(V) * wing_area; % Just a rough initial estimate

% Drag due to wing in z-direction
body_drag_z = body_z_drag_fac * pressure(W) * wing_area;

F_body_drag = [-sign(U)*body_drag_x;-sign(V)*body_drag_y;-sign(W)*body_drag_z];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Additional angular rate dependent forces
F_rate_dependent = [F_Q_fac_x*Q;0;F_Q_fac_z*Q]; % These should be small effects hopefully

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Now moments/rotational stuff

% Moment from wings and ailerons
d_wing = [0; 0; 0];

M_wing = cross(d_wing, F_wing);

% Moment from elevon

M_elevL = cross(elevL_body_pos, F_elevL);
M_elevR = cross(elevR_body_pos, F_elevR);

% Moment from stabilizer
%d_stabilizer = [-stab_arm;0;-35/1000]; % -35 mm in z approximately
%M_stabilizer = cross(d_stabilizer,F_stabilizer);

% Moment from rudder
%d_rudder = [-rudder_arm;0;-9/1000]; % -9 mm in z approximately
M_wingletL = cross(wingletL_body_pos, F_wingletL);
M_wingletR = cross(wingletR_body_pos, F_wingletR);

% Moment from throttle aerodynamic drag
M_throttle = [thr_drag_fac*thr; 0; 0];

% Additional rate dependent moments
M_rate_dependent = [M_P_fac*P; M_Q_fac*Q; M_R_fac*R];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Put equations together

% Kinematics
xyzdot = R_body_to_world*[U;V;W];
Phi = angularvel2rpydotMatrix(rpy);
rpydot = Phi*omega_world;

% Dynamics
% Translational equations
F_total = F_wing + ... % wing
          F_elevL + F_elevR + ...
          F_wingletL + F_wingletR + ...
          F_gravity + F_thrust + F_body_drag + F_rate_dependent; % gravity, thrust, body drag, rate dependent force

Sw = [0 -R Q; R 0 -P;-Q P 0];
UVW_dot = -Sw*[U;V;W] + F_total/m;

% Rotational stuff
M_total = M_wing + ...
          M_elevL + M_elevR + ... %M_stabilizer + M_rudder + 
          M_wingletL + M_wingletR + ...
          M_throttle + M_rate_dependent;
        
PQR_dot = invJ*(M_total - cross([P;Q;R],J*[P;Q;R]));


xdot = [xyzdot;rpydot;UVW_dot;PQR_dot];

% xdot_world_ani = ConvertToWorldCoords(xdot, R_body_to_world, rpy, [U;V;W], [P;Q;R]);
% 
% rotm(1:3,1:3) = [ 1,  0,  0;
%                   0, -1,  0;
%                   0,  0, -1];
% 
% rotm_full = blkdiag(rotm, rotm, rotm, rotm);
% 
% xdot_world_drake = rotm_full * xdot_world_ani;
% 
% valuecheck(ConvertXdotModelToDrake(x, xdot), xdot_world_drake);
% 
% disp('Correct...!');

end

function cl = Cl_fp(a) % Flat plate model
% if a > pi/2
%     a = pi/2;
% elseif a<-pi/2
%     a = -pi/2;
% end

cl = 2*sin(a)*cos(a);

end

function cd = Cd_fp(a) % Flat plate
% if a > pi/2
%     a = pi/2;
% elseif a<-pi/2
%     a = -pi/2;
% end

cd = 2*(sin(a)^2);

end

function cl = Cl_fp_fit(a) % Flat plate model with correction terms
if a > pi/2
    a = pi/2;
elseif a<-pi/2
    a = -pi/2;
end

% These numbers were fit from no-throttle experiments
cl = 2*sin(a)*cos(a) + 0.5774*sin(3.0540*a);

end

function cd = Cd_fp_fit(a) % Flat plate with correction terms
if a > pi/2
    a = pi/2;
elseif a<-pi/2
    a = -pi/2;
end

% These numbers were fit from no-throttle experiments
cd = 2*(sin(a)^2) - 0.1027*(sin(a)^2) + 0.1716;

end

function f = rotAlpha(f,a)
% Rotation matrix
Ra = [-cos(a) 0  sin(a); ...
       0      0  0     ; ...
      -sin(a) 0 -cos(a)];

% Rotate f using Ra
f = Ra*f;  
end


% function cm = Cm(obj,a) % xfoil
% if a > pi/2
%     a = pi/2;
% elseif a<-pi/2
%     a = -pi/2;
% end
% 
% cm = ppval(obj.Cmpp, a*180/pi);
% end

function pre = pressure(vel) %Dynamic Pressure = .5 rho * v^2
  % rho = 1.1839; % kg/m^3 (density of air)
pre = .5 * 1.204 * vel^2; % N/m^2
end
% 
% 
% 
% function xdot_world = ConvertToWorldCoords(xdot, R_body_to_world, rpy, uvw, pqr)
%   
%   xyzdot = xdot(1:3);
%   rpydot = xdot(4:6);
%   UVW_dot = xdot(7:9);
%   pqr_dot = xdot(10:12);
% 
%   phi = rpy(1);
%   theta = rpy(2);
%   psi = rpy(3);
%   
%   phidot = rpydot(1);
%   thetadot = rpydot(2);
%   psidot = rpydot(3);
%   
%   R = R_body_to_world;
%   
%   
%   % Now, convert pqr_dot to rpy_ddot
%   [Phi, dPhi] = angularvel2rpydotMatrix([phi;theta;psi]);
% 
%   Rdot =  [ 0, sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta),   cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta); ...
%             0, cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi), - cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta); ...
%             0,                              cos(phi)*cos(theta),                               -cos(theta)*sin(phi)]*phidot + ...
%             [ -cos(psi)*sin(theta), cos(psi)*cos(theta)*sin(phi), cos(phi)*cos(psi)*cos(theta); ...
%               -sin(psi)*sin(theta), cos(theta)*sin(phi)*sin(psi), cos(phi)*cos(theta)*sin(psi); ...
%                       -cos(theta),         -sin(phi)*sin(theta),         -cos(phi)*sin(theta)]*thetadot + ...
%                      [ -cos(theta)*sin(psi), - cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta), cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta); ...
%                        cos(psi)*cos(theta),   cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta); ...
%                                          0,                                                  0,                                                0]*psidot;
% 
%   rpy_ddot = Phi*R*pqr_dot + reshape((dPhi*[phidot;thetadot;psidot]),3,3)*R*pqr + ...
%              Phi*Rdot*pqr;
% 
%   xyz_ddot_world = R_body_to_world*UVW_dot + Rdot * uvw;
%            
%   xdot_world = [xyzdot; rpydot; xyz_ddot_world; rpy_ddot];
% 
%                      
% end
% 
