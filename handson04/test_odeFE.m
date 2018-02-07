%% test if function works

% test function  y'=r*y
r = 0.5;

test_fun = @(t, y) r*y;

% solution params
dt = 1e-6;
tk = 1;

% initial conditions
y0 = 1;

% solution of ODE
[T, Y] = odeFE(test_fun, [dt, tk], y0);

%% verify the solution

% exact solution
Yexact = y0*exp(r*T);

rmse = @(x1, x2) sqrt(sum((x1-x2).^2) / length(x1));

% plot(T, Y, T, Yexact)
plot(T, Y-Yexact)

disp(['Time step: ',num2str(dt), ' norm(err) = ', num2str(rmse(Y,Yexact))])
