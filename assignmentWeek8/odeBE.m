function [ t, y ] = odeBE( fun, tspan, y0, dfun )
%ODEBE Simple implementation of the Backward Euler method
%   fun - function to integrate
%   tspan = [dt, tk] - time step and finial time
%   y0 - initial conditions
%   dfun - Jacobian of fun
%   Output: t, y - time points and solution values

dt = tspan(1);
tk = tspan(2);
t = (0:dt:tk)';

n = length(t);
m = length(y0);

y = zeros( n, m );

I = eye(m);
% at time = 0
y(1, :) = y0(:)';
yi = y0(:);

for i = 2:n
    gf = @(x) x - yi - dt*fun(t(i), x);
    dgf = @(x) I - dt*dfun(t(i), x);
    [yi, iflag] = NewtonRaphson(gf, dgf, yi + dt*fun(t(i-1), yi));
    if iflag
        y(i, :) = yi';
    else
        t = t(1:i-1);
        y = y(1:i-1, :);
        break;
    end
end


end

