
options.floating = true;
r = RigidBodyManipulator('urdf/robots/TBSC_assembly_urdf2.URDF', options);
v = r.constructVisualizer;

%%

x0 = [0; 0; 0; 0; 0; 0; 10; 0; 0; 0; 0; 0];

constant_traj = ConstantTrajectory([-pi/2 -pi/2 0]);
constant_traj = constant_traj.setOutputFrame(r.getInputFrame());

feedback_system = cascade(constant_traj, r);

[ytraj, xtraj] = feedback_system.simulate([0 1], x0);

v.playback(xtraj)