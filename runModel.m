
options.floating = true;
r = RigidBodyManipulator('urdf/robots/TBSC_assembly_urdf2.URDF', options);
v = r.constructVisualizer;

x0 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 10; 0; 0; 0; 0; 0; 0; 0; 0];
[ytraj, xtraj] = r.simulate([0 1], x0);

v.playback(xtraj)