megaclear

parameters = GetDefaultGains();

p = DeltawingPlant(parameters);

p_state_est = DeltawingPlantStateEstFrame(p);

p_tester = DeltawingPlantStateEstFrameTester(p_state_est);

x0 = zeros(12,1);
u0 = zeros(3,1);

t = 0;

p.dynamics(t, x0, u0) - p_state_est.dynamics(t, x0, u0)


x0 = rand(12,1);
u0 = rand(3,1);

x0(4:6) = zeros(3,1);
x0(10:12) = zeros(3,1);

p.dynamics(t, x0, u0) - p_state_est.dynamics(t, x0, u0)




%%

%x0 = zeros(12,1);
%u0 = zeros(3,1);

x0 = rand(12,1);
u0 = rand(12,1);


[x1, dx1] = p.dynamics(t, x0, u0);

[x2, dx2] = p_tester.dynamics(t, x0, u0);

x1 - x2

dx1 - dx2