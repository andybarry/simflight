
x = 5*rand(1);
y = 5*rand(1);
z = 5*rand(1);
roll = 3*rand(1);
pitch = 3*rand(1);
yaw = 3*rand(1);
xdot = 5*rand(1);
ydot = 5*rand(1);
zdot = 5*rand(1);
rolldot = 5*rand(1);
pitchdot = 5*rand(1);
yawdot = 5*rand(1);



% x = 0;
% y = 0;
% z = 0;
% roll = 0;
% pitch = 1;
% yaw = 0;
% xdot = 0;
% ydot = 0;
% zdot = 0;
% rolldot = 0;
% pitchdot = 0;
% yawdot = 1;


u0 = [0; 0; 0];

x0_drake = [ x; y; z; roll; pitch; yaw; xdot; ydot; zdot; rolldot; pitchdot; yawdot ]

x0_body = ConvertToModelFrameFromDrakeWorldFrame(x0_drake)


x0_drake2 = ConvertToDrakeFrameFromModelFrame(x0_body)

valuecheck(x0_drake2, x0_drake);

disp('Correct.');