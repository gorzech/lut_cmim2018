function [ xpp ] = systemAcc( x, xp, m, k, c, L, g )
%SYSTEMACC Function return assignment system accelerations
%   Inputs:
%   x, xp - position and velocity
%   m, k, c - mass, stiffness, damping
%   L - length of the beam
%   g - gravity acc if included

M = [3*m, m*L; m*L, 2*m*L*L/3];
C = [2*c, c*L; c*L, c*L*L];
K = [3*k, 2*k*L; 2*k*L, 2*k*L*L + m*g*L];
xpp = -M\(C*xp + K*x);


end

