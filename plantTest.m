[parameters, gains] = GetDefaultGains();
p_world_coords = DeltawingPlant(parameters);
p = DeltawingPlantStateEstFrame(p_world_coords);

x0 = [0.8147
    0.9058
    0.1270
    0.9134
    0.6324
    0.0975
    0.2785
    0.5469
    0.9575
    0.9649
    0.1576
    0.9706];
  
u0 = [0.9572
    0.4854
    0.8003];
xdot1 = p.dynamics(0, x0, u0);

xdot_old = [    0.8637
   -0.3413
    0.6565
    1.4909
   -0.6720
    0.8900
    7.3946
   -5.6273
   -5.6419
   -5.9900
   -2.9629
    0.3268];
  
  
assert(sum(abs(xdot1 - xdot_old)) < .001, 'mismatch');

disp('ok');