function [ M ] = MassMatrix( m, L )

Ic = 1/12 * m * L * L;

M = diag([m, m, Ic, m, m, Ic]);

end

