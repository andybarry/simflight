% find trim condition for the aircraft using fmincon



Ceq = @(in) tbsc_model_less_vars(in(1:2), in(3:5));


x  = lsqnonlin(Ceq, [0; 20; 0; 0; 3.5])

% 
% A = [];
% b = [];
% Aeq = [];
% beq = [];
% lb = [];
% ub = [];
% 
% 
% sol = fmincon([], [0; 20; 0; 0; 3.5], A, b, Aeq, beq, lb, ub, @fmincon_nonlincon);








parameters = { 1.92, 1.84, 2.41, 0.48, 0.57 };
