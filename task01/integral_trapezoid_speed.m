function result = integral_trapezoid_speed( fun, low_limit, up_limit, no_splits )
%INTEGRAL_TRAPEZOID Calculate the integral of the 1D function.
%   Function that calculate the integral of 1D continous function using
%   well-known trapezoidal rule. Vectorized approach.
%   fun - handle of a function to integrate,
%   low_limit - lower limit of an integral,
%   up_limit - upper limit of an integral,
%   no_splits - number of trapezoids.

% integration step
h = (up_limit - low_limit) / no_splits;

% result of the function in all points
F = fun(low_limit:h:up_limit);

% result = (0.5.*h) .* sum(F(1:end-1) + F(2:end));
result = h .* ( ( F(1) + F(end) ).*0.5 + sum( F(2:end-1) ) );

end

