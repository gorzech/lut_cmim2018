%% Symbolic toolbox tutorial - in appliction to SDOF systems
% Open help for symbolic toolbox.

doc symbolic
%% Define symbolic variables
%%
syms m k c real
assumeAlso(m > 0);
assumeAlso(k > 0);
assumeAlso(c > 0);

syms x(t)
%% Analyze free vibrations
% $$m\ddot{x}+kx=0$$
%%
xp = diff(x, t);
xpp = diff(xp, t);

Eq = m*xpp + k*x;

solution_free = dsolve(Eq == 0);

disp(solution_free)
%% rewrite
%%
oms = sqrt(k/m);
syms omega real

solution_free = subs(solution_free, oms, omega);

disp(solution_free)
%% Solve with initial conditions
% $$\left\{ \begin{array}{c}m\ddot{x}+kx=0\\x\left(0\right)=A,\,\dot{x}\left(0\right)=0\end{array}\right.$$
%%
syms A real
solution_free_ic = dsolve(Eq == 0, [xp(0) == 0, x(0) == A] );
solution_free_ic = subs(solution_free_ic, oms, omega);
disp(solution_free_ic)
% display equation variables
symvar(solution_free_ic)
%% Plot solution
%%
figure
fplot(subs(solution_free_ic, [A, omega], [-1, 2]), [0, 10])
grid on
%% Get numerical solution
%%
tsol = linspace(0, 15, 15*20);
xsol = subs(solution_free_ic, [A, omega, t], {-0.5, sqrt(10/1), tsol});
vsol = subs(diff(solution_free_ic, t), [A, omega, t], {-0.5, sqrt(10/1), tsol});
%% Animate solution
%%
animate_sdof(tsol, double(xsol), double(vsol))
%% Now add damping to the equations
% $$\left\{ \begin{array}{c}m\ddot{x}+c\dot{x}+kx=0\\x\left(0\right)=A,\,\dot{x}\left(0\right)=0\end{array}\right.$$
%%
Eq = m*xpp + c*xp + k*x;

solution_damped = dsolve(Eq == 0, [xp(0) == 0, x(0) == A] );

disp(solution_damped)
%% Substitutions
% Knowing that $c_c=2 \sqrt(km)$ is critical damping and damping ratio is $\zeta=c/c_c$:
%%
syms cc zeta real

solution_damped_rewrite = simplify(expand(subs(subs(simplify(subs(subs(solution_damped, 4*k*m, cc*cc), c, zeta*cc)),cc,2*sqrt(k*m)),sqrt(k/m),omega)));
disp(solution_damped_rewrite)
%% Plot solution
%%
figure
fplot(subs(solution_damped_rewrite, [A, omega, zeta], [-1, 2, 0.99]), [0, 10])
grid on
%% Get numerical solution
%%
zeta_num = 1.1;
tsol = linspace(0, 15, 15*20);
xsol = subs(solution_damped_rewrite, [A, omega, t, zeta], {-0.5, 2*pi, tsol, zeta_num});
vsol = subs(diff(solution_damped_rewrite, t), [A, omega, t, zeta], {-0.5, 2*pi, tsol, zeta_num});
%% Animate solution
%%
animate_sdof(tsol, double(xsol), double(vsol))