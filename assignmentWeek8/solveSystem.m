%% define variables and ode settings
clc; clear;

param = struct('m', 2, 'k', 134, 'c', 12, 'L', 0.6, 'g', 9.81);

odeFun = @(~, y) systemOdeFun(y, param);
odeJac = @(~, ~) systemOdeJac(param);

y0 = [0; 0; 1; 0.2]; % initial conditions

dt = 1e-3;
tk = 1;

[t, y] = odeBE(odeFun, [dt, tk], y0, odeJac);

%% plot results to see if they make sense
plot(t, y(:, 2)) 
% point 4 done, but are results accurate?

%% check with shorter time step
divFactor = 4;
[t2, y2] = odeBE(odeFun, [dt/divFactor, tk], y0, odeJac);

disp('Compare odeBE with two different dt')
compareResults( t, y, t2, y2 )
%% solve using ode15s
disp('ode15s default options')
opts = odeset('AbsTol', 1e-9, 'RelTol', 1e-6);
tic
[t15s, y15s] = ode15s(odeFun, t, y0, opts);
toc
compareResults( t, y, t15s, y15s )

% 
disp('ode15s with Jacobian')
opts = odeset(opts, 'Jacobian', odeJac);
tic
[t15sJ, y15sJ] = ode15s(odeFun, t, y0, opts);
toc
compareResults( t, y, t15sJ, y15sJ )

disp('Compare between ode15s-es')
compareResults( t15s, y15s, t15sJ, y15sJ )