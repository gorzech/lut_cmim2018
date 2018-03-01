%% NR test
fun = @(x) x.^2 - 9;
dfun = @(x) 2.*x;
x0 = 1000;
% NewtonRaphson(fun, dfun, x0)
[xr, flag] = NewtonRaphson(fun, dfun, x0)

%% odeBE test
% u' = r*u - solution u(t) = u0 exp(r*t)
r = 0.5;
u0 = 2;
ofun = @(t, x) r.*x;
odfun = @(t, x) r;
[t, y] = odeBE(ofun, [1e-5, 1], u0, odfun);
osol = u0 .* exp(r .* t);

plot(t, y, t, osol, '--', 'LineWidth', 1.5)
disp(rmse(y, osol'))

%% second ode test
% x'' + 2x' + 5x = 3, x0=0, x'0 = 0
% solution
osol2 = @(t) 3/5 - 3/10*exp(-t) .* sin(2.*t) - 3/5.*exp(-t).*cos(2.*t);

% order reduction - y = [x'; x];
ofun2 = @(t, y) [3 - 2*y(1) - 5*y(2); y(1)];
odfun2 = @(t, y) [-2, -5; 1, 0];
[t2, y2] = odeBE(ofun2, [1e-5, 1], [0;0], odfun2);
% [t2, y2] = ode45(ofun2, 0:1e-4:1, [0;0]);

osol2val = osol2(t2);
plot(t2, y2(:, 2), t2, osol2val, '--', 'LineWidth', 1.5)
disp(rmse(y2(:, 2), osol2val))
