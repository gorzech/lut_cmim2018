%% define vars and compute ode function
syms m k c L g real
x = sym('x', [2, 1], 'real');
xp = sym('xp', [2, 1], 'real');

y = [x; xp];

param = struct('m', m, 'k', k, 'c', c, 'L', L, 'g', g);

Fun = systemOdeFun(y, param);

%% compute jacobian and generate matlab function out of it

FunDy = jacobian(Fun, y);

matlabFunction(FunDy, 'File', 'systemOdeJacOrig', 'Vars', {struct2array(param)});
% then make some simple edit and rename to systemOdeJac