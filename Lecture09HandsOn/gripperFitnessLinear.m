function [ fitnessValue ] = gripperFitnessLinear(BUx, BUy, CUx, CUy)

% BUx = 2; % range 0-4?
% BUy = 5; % 4-5
% CUx = -3; 
% CUy = 4;

% model points
A = [0; 0];
KU = [9; 6];
BU = [BUx; BUy];
CU = [CUx; CUy];

KL = [9; -6];
BL = [BUx; -BUy];
CL = [CUx; -CUy];

% for body 1
AF = [-6; 0];

roL = 2;

% model definition
body1.type = 'slender_link';
body1.pi = A;
body1.pj = AF;
body1.m = norm(A-AF)*roL;

body2a.type = 'slender_link';
body2a.pi = BU;
body2a.pj = KU;
body2a.m = norm(KU-BU)*roL;

body2b.type = 'slender_link';
body2b.pi = BU;
body2b.pj = A;
body2b.m = norm(A-BU)*roL;

body3.type = 'slender_link';
body3.m = norm(CU-BU)*roL;
body3.pi = CU;
body3.pj = BU;

body4a.type = 'slender_link';
body4a.pi = BL;
body4a.pj = KL;
body4a.m = norm(KL-BL)*roL;

body4b.type = 'slender_link';
body4b.pi = BL;
body4b.pj = A;
body4b.m = norm(A-BL)*roL;

body5.type = 'slender_link';
body5.m = norm(CL-BL)*roL;
body5.pi = CL;
body5.pj = BL;

model_def.bodies = {body1, body2a, body2b, body3, body4a, body4b, body5};

joint1.type = 'revolute';
joint1.ibody = 0;
joint1.jbody = 4;
joint1.p = CU;

joint2.type = 'revolute';
joint2.ibody = 0;
joint2.jbody = 7;
joint2.p = CL;

joint3.type = 'revolute';
joint3.ibody = 2;
joint3.jbody = 4;
joint3.p = BU;

joint4.type = 'revolute';
joint4.ibody = 3;
joint4.jbody = 4;
joint4.p = BU;

joint5.type = 'revolute';
joint5.ibody = 5;
joint5.jbody = 7;
joint5.p = BL;

joint6.type = 'revolute';
joint6.ibody = 6;
joint6.jbody = 7;
joint6.p = BL;

joint7.type = 'revolute';
joint7.ibody = 1;
joint7.jbody = 3;
joint7.p = A;

joint8.type = 'revolute';
joint8.ibody = 1;
joint8.jbody = 6;
joint8.p = A;

% We need extra two joints - fix angle between 2a and 2b as well as between
% 4a and 4b
% Also need to fix some coords.

joint9.type = 'fix_orientation';
joint9.ibody = 2;
joint9.jbody = 3;

joint10.type = 'fix_orientation';
joint10.ibody = 5;
joint10.jbody = 6;

joint11.type = 'fix_coordinates';
joint11.ibody = 1;
joint11.cidx = [2; 4];

model_def.joints = {joint1, joint2, joint3, joint4, ...
    joint5, joint6, joint7, joint8, joint9, ...
    joint10, joint11};

% force1.type = 'gravitational';
% force1.g = [0; -9.81];

force1.type = 'point';
force1.ibody = 1;
force1.F = @(t) [-500*(1-t); 0];
force1.c = 1;

model_def.forces = {force1};

% simulation settings
settings.tspan = linspace(0, 1, 101);
settings.method.type = 'baumgarte';
settings.method.alfa = 1;
settings.method.beta = 1;
settings.ode.solver = 'ode45';
settings.ode.abstol = 1e-7;
settings.ode.reltol = 1e-4;

%% Solution

result = solveSystem(model_def, settings);
% plot(result.P(:, 7), result.P(:, 8)) % upper end point 
idxgt2 = result.P(:, 8) >= 2;
x0 = result.P(1, 7);

fitnessValue = norm(result.P(idxgt2, 7) - x0) / sum(idxgt2);
if result.P(end, 8) >= 2
    fitnessValue = fitnessValue + result.P(end, 8) - 2;
end

end

