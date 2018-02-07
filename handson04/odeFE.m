function [ T, Y ] = odeFE( fun, tspan, y0 )
%ODEFE Simple integration using Forward Euler
%   fun - function handle with interface fun(t, y)
%   tspan - two element vector with dt and tend
%   y0 - initial conditions

% time step
dt = tspan(1);

T = 0:dt:tspan(2);

n = length(T);

Y = zeros(length(y0), n);

% populate initial conditions
Y(:, 1) = y0(:);

% compute the solution
for i = 2:n
    Y(:, i) = Y(:, i-1) + dt * fun(T(i-1), Y(:, i-1));
end

end

