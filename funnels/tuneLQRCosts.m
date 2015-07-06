% p = SBachPlantDummy();

%load initialConditionSet.mat

%Vtv = V;
%rho0 = 1;
%V0 = Vtv.getPoly(0)/rho0; % Get inlet of funnel

x = Vtv.getFrame.getPoly;

G0 = Vtv.S.eval(0);

V0 = x'*(G0)*x;
opts.num_samples = 100;
xinit = getLevelSet(x,V0,opts); % zeros(12,1));


tvOrig = tv;
% p.getStateFrame.addTransform(AffineTransform(p.getStateFrame,tv.getInputFrame,eye(length(x)),-xtraj));
% tv.getOutputFrame.addTransform(AffineTransform(tv.getOutputFrame,p.getInputFrame,eye(4),utraj));
sysCl = feedback(p,tvOrig);

% Get time samples
ts = Vtv.S.getBreaks(); tsend = ts(end);
% ts = ts(ts >= 4/70);
ts = ts(1:ceil(ts(end)/12*(1/mean(diff(ts)))):length(ts));
%%

figure(1)
clf

Vmaxs = -Inf*ones(1,length(ts));
for k = 1:length(xinit)
    x0 = 0.95*xinit(:,k) + xtraj.eval(0);
    xtrajSim = sysCl.simulate([0 ts(end)], x0);
    
    for j = 1:length(ts)
        Vsj = Vtv.eval(ts(j),xtrajSim.eval(ts(j)) - xtraj.eval(ts(j)));
        Vmaxs(j) = max(Vmaxs(j),double(Vsj));
    end
    
    for coord = 2 : 12
      subplot(4,3,coord);
      
      fnplt(xtrajSim,[1 coord])
      hold on
      drawnow;
    end
end



%% talyor approx check

global_max = 0;

Vmaxs = -Inf*ones(1,length(ts));

f = polyOrig.getPolyDynamics(0);

u = utraj.getOutputFrame.getPoly();

for k = 1:length(xinit)
    x0 = 0.95*xinit(:,k) + xtraj.eval(0);
    
    
    talyor_xdot = subs(f, [x;u], [x0;utraj.eval(0)]);

    real_xdot = p.dynamics(0, x0, utraj.eval(0));
    
    this_max = max(abs(double(talyor_xdot - real_xdot)))
    
    if this_max > global_max
      global_max = this_max;
    end
    
    
end

global_max