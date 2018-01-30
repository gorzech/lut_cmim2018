%% compute model 

k = 10; 
m = 1;
c = 0;

x0 = [-0.5; 0];

difffun = @(~,x) [x(2); (-k*x(1)-c*x(2))/m];

tnum = linspace(0, 15, 15*20);

[T, Y] = ode15s(difffun, tnum, x0);

% %% test plots
% 
% plot(T, Y(:, 2))

%% animate

animate_sdof(T, Y(:, 1), Y(:, 2))