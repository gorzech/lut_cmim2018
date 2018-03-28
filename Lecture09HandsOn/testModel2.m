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
p2 = [1; 0];
p3 = [2; 0];

% model definition
body1.type = 'slender_link';
body1.m = 2;
body1.pi = p1;
body1.pj = p2;

body2.type = 'slender_link';
body2.m = 2;
body2.pi = p2;
body2.pj = p3;

model_def.bodies = [body1, body2];

joint1.type = 'revolute';
joint1.ibody = 0;
joint1.jbody = 1;
joint1.p = p1;

joint2.type = 'revolute';
joint2.ibody = 1;
joint2.jbody = 2;
joint2.p = p2;

model_def.joints = [joint1, joint2];

force1.type = 'gravitational';
force1.g = [0; -9.81];

model_def.forces = force1;

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
plot(result.T, result.P(:, 7:8))