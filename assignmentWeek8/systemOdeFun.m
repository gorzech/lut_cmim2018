function [ yp ] = systemOdeFun( y, param )
%SYSTEMODEFUN function input for ode solver based on system in assignment

yp = [y(3:4); systemAcc(y(1:2), y(3:4), param.m, param.k, param.c, param.L, param.g )];

end

