% p = SBachPlantDummy();

load initialConditionSet.mat

Vtv = V;
%rho0 = 1;
%V0 = Vtv.getPoly(0)/rho0; % Get inlet of funnel
x = V.getFrame.poly;
V0 = x'*(G0)*x;
opts.num_samples = 100;
xinit = getLevelSet(decomp(V0),V0,opts); % zeros(12,1));

x = p.getStateFrame.poly;
tvOrig = tv;
% p.getStateFrame.addTransform(AffineTransform(p.getStateFrame,tv.getInputFrame,eye(length(x)),-xtraj));
% tv.getOutputFrame.addTransform(AffineTransform(tv.getOutputFrame,p.getInputFrame,eye(4),utraj));
sysCl = feedback(p,tvOrig);

% Get time samples
ts = Vtv.S.getBreaks(); tsend = ts(end);
% ts = ts(ts >= 4/70);
ts = ts(1:ceil(ts(end)/12*(1/mean(diff(ts)))):length(ts));


figure(1)
figure(2)

Vmaxs = -Inf*ones(1,length(ts));
for k = 1:length(xinit)
    x0 = 0.95*xinit(:,k) + xtraj.eval(0);
    xtrajSim = sysCl.simulate([0 ts(end)], x0);
    
    for j = 1:length(ts)
        Vsj = Vtv.eval(ts(j),xtrajSim.eval(ts(j)) - xtraj.eval(ts(j)));
        Vmaxs(j) = max(Vmaxs(j),double(Vsj));
    end
    
    figure(1)
    hold on
    fnplt(xtrajSim,[1 2])
    drawnow;
    
    figure(2)
    hold on
    fnplt(xtrajSim,[1 3])
    drawnow;
end