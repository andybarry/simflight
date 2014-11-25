function runPassive

p = DeltawingPlant();

%gv = GliderVisualizer(gp);

traj = simulate(p,[0 .5])
%playback(gv,traj);
