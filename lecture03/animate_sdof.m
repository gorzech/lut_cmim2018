function animate_sdof(t, x, v)
% Animate single DOF system.
% Modified by G. Orzechowski, 2018

%Animation function for a horizontal spring/mass/damper
%Written by T. Nordenholz, Fall 05
%To use, type free_sim(t,x) where t and x are the time (sec) and
%displacement (m) arrays
%Geometrical and plotting parameters can be set within this program

% set geometric parameters
W = 0.05; %width of mass
H = 0.1; %height of mass
L0 = ceil(( max(abs(x)) + W )*10)/10;
% L0 = 0.2;%unstretched spring length
Wsd = 0.5*H; %spring width
xrect = [-W/2,-W/2,W/2,W/2,-W/2]; % plotting coordinates of mass
yrect = [0,H,H,0,0];

%set up and initialize plots
%x vs t plot
figure('Units','normalized','Position',[.1,.1,.8,.8]);
set(gcf,'Visible','on') % for live script to work
subplot(2, 2, 1);
plot(t, x);
grid on
xlabel('t (sec)'); ylabel('x (m)')
hold on
H_plot_position = plot(t(1), x(1), 'o', 'MarkerFaceColor', 'red');
hold off
title('Position of the mass');

subplot(2, 2, 2);
plot(t, v);
grid on
xlabel('t (sec)'); ylabel('v (m/s)')
hold on
H_plot_velocity = plot(t(1), v(1), 'o', 'MarkerFaceColor', 'red');
hold off
title('Velocity of the mass');

% animation plot
subplot(2,2,[3,4]);
% create mass
Hp_f2rect = fill(xrect+x(1), yrect, 'b');
axis([-L0,L0,-H,2*H])
grid on
Hl_cm = line(x(1),H/2,'Marker','O','MarkerSize',8,'MarkerFaceColor', 'w');
% create spring/damper
Hgt_springdamp=hgtransform;
Hl_Lend=line([0,.1],[0,0],'Color','k','Parent',Hgt_springdamp);
Hl_Rend=line([.9,1],[0,0],'Color','k','Parent',Hgt_springdamp);
Hl_Lbar=line([.1,.1],Wsd*[-1,1],'Color','k','Parent',Hgt_springdamp);
Hl_Rbar=line([.9,.9],Wsd*[-1,1],'Color','k','Parent',Hgt_springdamp);
Hl_spring=line(linspace(.1,.9,9),Wsd*[1,2,1,0,1,2,1,0,1],'Color','k','Parent',Hgt_springdamp);
Hl_dampL=line([.1,.4],Wsd*[-1,-1],'Color','k','Parent',Hgt_springdamp);
Hl_dampLpist=line([.4,.4],Wsd*[-1.3,-.7],'Color','k','Parent',Hgt_springdamp);
Hl_dampR=line([.6,.9],Wsd*[-1,-1],'Color','k','Parent',Hgt_springdamp);
Hl_dampRcyl=line([.35,.6,.6,.35],Wsd*[-.5,-.5,-1.5,-1.5],'Color','k','Parent',Hgt_springdamp);
% set initial length
L=L0+x(1)-W/2;
set(Hgt_springdamp,'Matrix',[L,0,0,-L0;0,1,0,H/2;0,0,1,0;0,0,0,1]);
text(0,1.5*H,'|-> x');
% draw and hold for 1 second
drawnow
tic;while toc<t(1),end
tic

% Animate by looping through time and x arrays
% and redrawing at each value
for idx = 1:length(t)
    L = L0 + x(idx) - W/2;
    H_plot_position.XData = t(idx);
    H_plot_position.YData = x(idx);
    H_plot_velocity.XData = t(idx);
    H_plot_velocity.YData = v(idx);
    %    set(Hls_f2plot1,'XData',t(1:n),'YData',x(1:n));
    set(Hp_f2rect,'XData',xrect+x(idx));
    set(Hl_cm,'XData',x(idx));
    set(Hgt_springdamp,'Matrix',[L,0,0,-L0;0,1,0,H/2;0,0,1,0;0,0,0,1]);
    while toc<t(idx),end
    drawnow;
end