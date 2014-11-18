% tests TBSC model against URDF model when u = 0

clear

%% Construct from URDF


disp('Constructing from URDF...');

options.floating = true;
r = RigidBodyManipulator('urdf/robots/TBSC_assembly_urdf2_params.URDF', options);
v = r.constructVisualizer();


x0 = [0; 0; 0; 0; 0; 0; 15; 1; 0; 0; 0; 0];

u0 = [0; 0; 0];

xdot_urdf_before_rot = r.dynamics(0, x0, u0);

% convert to Ani's model coordinates

% rotate 180 degrees about the x-axis

% rotm = [ 1, 0, 0;
%          0, cos(pi), -sin(pi);
%          0 sin(pi), cos(pi)];
       
rotm(1:3,1:3) = [ 1,  0,  0;
                  0, -1,  0;
                  0,  0, -1];

rotm_full = blkdiag(rotm, rotm, rotm, rotm);
       
xdot_urdf(1:12,:) = rotm_full*xdot_urdf_before_rot;


%% construct from hand-written model

xdot_hand = tbsc_model(0, rotm_full*x0, u0)

valuecheck(xdot_hand, xdot_urdf);

disp('Correct.');