disp('Constructing from URDF...');

options.floating = true;
r = RigidBodyManipulator('urdf/robots/TBSC_assembly_urdf2_params.URDF', options);
%r = RigidBodyManipulator('/home/abarry/drake/drake/examples/Wingeron/Plane.URDF', options);



%r = r.setParamFrame(CoordinateFrame('TbscParams',2,'p',...
%        { 'drag_fuselage_xy','drag_fuselage_z' }));

params = r.getParams();

params.drag_fuselage_xy = 1;
params.drag_fuselage_z = 100000;

r = r.setParams(params);


v = r.constructVisualizer();

v.inspector(zeros(12,1), 1:12)

%%

%    [x; y; z; roll; pitch; yaw; xdot; ydot; zdot; rolldot; pitchdot; yawdot]
x0 = [0; 0; 0; 0; 0; 0; .1; 0; 0; 0; 0; 0];

constant_traj = ConstantTrajectory([0.8 0.8 100]);
constant_traj = constant_traj.setOutputFrame(r.getInputFrame());

feedback_system = cascade(constant_traj, r);


disp('Simulating...');


%feedback_visualize_system = cascade(feedback_system, v);
%feedback_visualize_system.simulate([0 1], x0);

[ytraj, xtraj] = feedback_system.simulate([0 1], x0);

options_v.slider = true;
v.playback(xtraj, options_v)