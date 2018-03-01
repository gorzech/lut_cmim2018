function [ xout, flag ] = NewtonRaphson( fun, dfun, x0, maxiter, tol )
%NEWTONRAPHSON Simple implementation of the NR method
%   fun - vector function f(x)
%   dfun - df/dx - Jacobian matrix
%   x0 - initial point
%   maxiter - optional maximum number of iterations
%   tol - optional tollerance
%   Outputs:
%   xout - converged value
%   flag - indicate if computations are successful or not (bool)

if nargin < 4
    maxiter = 30;
end
if nargin < 5
    tol = eps(max(abs(x0)))^0.8;
end

iter = 0;
xc = x0;
fval = fun(xc);
solnorm = norm(fval);

while solnorm > tol && iter < maxiter
    dfval = dfun(xc);
    xc = xc - dfval\fval;
    fval = fun(xc);
    solnorm = norm(fval);
    
    iter = iter + 1;
end

% if solution looks correct, assign solution to output
if solnorm <= tol && ~isnan(solnorm)
    xout = xc; 
    flag = true;
else
    flag = false;
    xout = x0;
end

end

