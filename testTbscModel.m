% tests TBSC model against URDF model when u = 0

clear

%% Construct from URDF


disp('Constructing from URDF...');

options.floating = true;
r = RigidBodyManipulator('urdf/robots/TBSC_assembly_for_testing.URDF', options);
v = r.constructVisualizer();

x = 0;
y = 0;
z = 0;
roll = 1;
pitch = 0;
yaw = 0;
xdot = 15;
ydot = 0;
zdot = 0;
rolldot = 1;
pitchdot = 0;
yawdot = 0;

%x0_drake = [0; 0; 0; 0; 0; 0; 15; 0; 0; 1; 0; 0];

x0_drake = [ x; y; z; roll; pitch; yaw; xdot; ydot; zdot; rolldot; pitchdot; yawdot ];

x0_hand(1:6) = x0_drake(1:6);

rotm(1:3,1:3) = [ 1,  0,  0;
                  0, -1,  0;
                  0,  0, -1];

rotm_full = blkdiag(rotm, rotm, rotm, rotm);

x0_drake_rotated = rotm_full * x0_drake;

x0_hand(1:6) = x0_drake_rotated(1:6);

rpy = [ x0_drake_rotated(4); x0_drake_rotated(5); x0_drake_rotated(6)];
rpydot = x0_drake_rotated(10:12);
R_body_to_world = rpy2rotmat(rpy);
R_world_to_body = R_body_to_world';
R = R_body_to_world;

x0_hand(7:9) = R_world_to_body*x0_drake_rotated(7:9);

phi = rpy(1);
theta = rpy(2);
psi = rpy(3);

phidot = rpydot(1);
thetadot = rpydot(2);
psidot = rpydot(3);

pqr = rpydot2angularvel([phi;theta;psi],[phidot;thetadot;psidot]);
pqr = R'*pqr;

x0_hand(10:12) = pqr;



u0 = [0; 0; 0];

xdot_urdf = r.dynamics(0, x0_drake, u0(3))

% convert to Ani's model coordinates

% rotate 180 degrees about the x-axis

% rotm = [ 1, 0, 0;
%          0, cos(pi), -sin(pi);
%          0 sin(pi), cos(pi)];
       



       
%xdot_urdf(1:12,:) = rotm_full*xdot_urdf_before_rot


%% construct from hand-written model

xdot_hand = tbsc_model(0, x0_hand, u0)

valuecheck(xdot_hand, xdot_urdf);

disp('Correct.');