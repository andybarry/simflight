disp('Constructing from URDF...');

options.floating = true;
r = RigidBodyManipulator('urdf/robots/TBSC_assembly_urdf2.URDF', options);
v = r.constructVisualizer();

%%

disp('Simulating...');

x0 = [0; 0; 0; 0; 0; 0; 10; 0; 0; 0; 0; 0];

constant_traj = ConstantTrajectory([0.1 0.1 100]);
constant_traj = constant_traj.setOutputFrame(r.getInputFrame());

feedback_system = cascade(constant_traj, r);

%feedback_visualize_system = cascade(feedback_system, v);
%feedback_visualize_system.simulate([0 1], x0);

[ytraj, xtraj] = feedback_system.simulate([0 1], x0);

v.playback(xtraj)