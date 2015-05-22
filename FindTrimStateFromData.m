% from:
% date = '2015-05-14';
% name = 'field-test-gps2';
% log_number = '01';
tstart = 320;
tend = 324;

[~, idx_start] = min(abs(est.logtime - tstart));
[~, idx_end] = min(abs(est.logtime - tend));

xvel = mean(est.vel.x(idx_start:idx_end));
zvel = mean(est.vel.z(idx_start:idx_end));

roll = mean(est.orientation.roll(idx_start:idx_end));
pitch = mean(est.orientation.pitch(idx_start:idx_end));

x_fixed = zeros(12,1);

x_fixed(5) = pitch;
x_fixed(7) = xvel;
x_fixed(9) = zvel;

[~, idx_u_start] = min(abs(u.logtime - tstart));
[~, idx_u_end] = min(abs(u.logtime - tend));

elevL = mean(u.rad.elevonL(idx_u_start:idx_u_end));
elevR = mean(u.rad.elevonR(idx_u_start:idx_u_end));
throttle = mean(u.rad.throttle(idx_u_start:idx_u_end));

u_fixed = [elevL; elevR; throttle];

x_fixed_model = ConvertToModelFrameFromDrakeWorldFrame(x_fixed);

% ok now do optimization on model parameters
%p.dynamics(0, x_fixed, u_fixed)


tbsc_model(0, x_fixed_model, u_fixed, parameters{:}, 0.03)


myfunc = @(x_drag) tbsc_model_for_fixed_point(x_fixed_model, u_fixed, x_drag);

x_body_dragx  = lsqnonlin(myfunc, [0.03])

myfunc(x_body_dragx)

xdot = tbsc_model(0, x_fixed_model, u_fixed, parameters{:}, x_body_dragx)