% optimization with constraints
clc;clear
x0 = [2; 4; -3; 4];

xL = [0; 4; -5; 1];
xU = [4; 7; -1; 4];

optfun = @(x) gripperFitnessLinear(x(1), x(2), x(3), x(4));

% opts = optimoptions('fmincon', 'Algorithm', 'sqp');
% opts = optimoptions('fmincon', 'Algorithm', 'active-set');
opts = optimoptions('fmincon', 'Algorithm', 'interior-point');

[x, fval, exitflag, output] = fmincon(optfun, x0, [], [], [], [], xL, xU, [], opts);