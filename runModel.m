disp('Constructing from URDF...');

options.floating = true;
r = RigidBodyManipulator('urdf/robots/TBSC_assembly_urdf2.URDF', options);
%r = RigidBodyManipulator('/home/abarry/drake/drake/examples/Wingeron/Plane.URDF', options);
v = r.constructVisualizer();

q = zeros(r.getNumContStates(),1);
qd = zeros(r.getNumContStates(),1);

clf
for i = 1 : length(r.force)
  plot3(0, 0, 0, '*');
  if isa(r.force{i}, 'RigidBodyWing')
    r.force{i}.drawWing(r, q, qd, [rand() rand() rand()]);
  end
end

%%
%{
disp('Simulating...');

x0 = [0; 0; 0; 0; 0; 0; 15; 0; 0; 0; 0; 0];

constant_traj = ConstantTrajectory([0.8 0.8 100]);
constant_traj = constant_traj.setOutputFrame(r.getInputFrame());

feedback_system = cascade(constant_traj, r);

%feedback_visualize_system = cascade(feedback_system, v);
%feedback_visualize_system.simulate([0 1], x0);

[ytraj, xtraj] = feedback_system.simulate([0 5], x0);

v.playback(xtraj)

%}