%% define model using our assumptions
% Only link bodies using natural coordinates. 
% Only revolute joints.
% Only gravitational and concentrated forces.
% Baumgarte stabilization.

% Also we will use matlab structures to define model

% What we have to input and what we want to get as output?

% Input:
% Model definition:
% - bodies
% - joints
% - forces
% Simulation settings:
% - time span
% - Baumgarte parameters
% - maybe some extra parameters, e.g. ode solver and its tolerances

% Output:
% E.g. - for easier post-processing
% positions and velocities of end points of all links

%% Next - what to input for the bodies:
% - mass
% - end points at the initial configuration

% For joints (revolute):
% - ids of connected bodies
% - where joint is located

% For forces:
% - g vector for gravity
% - for concentrated forces - c1 value and force

%% So create model - for tests - single pendulum under gravity forces

% model points
p1 = [0; 0];
p2 = [1; 1];
p3 = [2; 3];
p4 = [2; 0];

% model definition
body1.type = 'slender_link';
body1.m = 28.3340254918;
body1.pi = p1;
body1.pj = p2;

body2.type = 'slender_link';
body2.m = 44.3622412228;
body2.pi = p2;
body2.pj = p3;

body3.type = 'slender_link';
body3.m = 59.2608254916;
body3.pi = p3;
body3.pj = p4;

model_def.bodies = {body1, body2, body3};

joint1.type = 'revolute';
joint1.ibody = 0;
joint1.jbody = 1;
joint1.p = p1;

joint2.type = 'revolute';
joint2.ibody = 1;
joint2.jbody = 2;
joint2.p = p2;

joint3.type = 'revolute';
joint3.ibody = 2;
joint3.jbody = 3;
joint3.p = p3;

joint4.type = 'revolute';
joint4.ibody = 3;
joint4.jbody = 0;
joint4.p = p4;

model_def.joints = {joint1, joint2, joint3, joint4};

force1.type = 'gravitational';
force1.g = [0; -9.81];

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
plot(result.T, result.P(:, 9:10))