%function runPassive

p = DeltawingPlant();

options.floating = true;
r = RigidBodyManipulator('urdf/robots/TBSC_assembly_for_drawing.URDF', options);
v2 = r.constructVisualizer();

v = SBachVisualizer(p);
v.xlims = [0 10];
v.ylims = [-5 5];


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


traj = simulate(p,[0 .5], x0_drake)


v2 = v2.setInputFrame(p.getOutputFrame());

playback(v2, traj, struct('slider', true));
