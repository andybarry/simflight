

p = DeltawingPlant();

options.floating = true;
r = RigidBodyManipulator('urdf/robots/TBSC_assembly_for_drawing.URDF', options);
v2 = r.constructVisualizer();


x = 0;
y = 0;
z = 0;
roll = .1;
pitch = -.5;
yaw = 0;
xdot = 15;
ydot = 0;
zdot = 5;
rolldot = 0;
pitchdot = 0;
yawdot = 0;

x0_drake = [ x; y; z; roll; pitch; yaw; xdot; ydot; zdot; rolldot; pitchdot; yawdot ]

constant_traj = ConstantTrajectory([0.8 -0.8 5.33976]);
constant_traj = constant_traj.setOutputFrame(p.getInputFrame());

feedback_system = cascade(constant_traj, p);

traj = simulate(feedback_system, [0 .5], x0_drake)


v2 = v2.setInputFrame(p.getOutputFrame());

playback(v2, traj, struct('slider', true));
