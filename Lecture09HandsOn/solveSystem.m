function result = solveSystem(model_def, settings)

% we need to compute system mass matrix (constant), force vector and
% constraints
% but before that we need constraints and know how they map into bodies
% (because they are shared between the bodies)

% preprocessing
% we need to define which coordinated in the bodies refer to them
% firstly calculate number of coordinates
no_bodies = length(model_def.bodies);
% no_joints = length(model_def.joints);
% vector of all bodies coordinates
np = 4*no_bodies;

p = zeros(np, 1);
Mp = zeros(np, np);
Qg = zeros(np, 1);
Lbody = zeros(no_bodies, 1);

% check if there is gravitational force
g = [0; 0];
no_forces = length(model_def.forces);
for ii = 1:no_forces
    if strcmp(model_def.forces{ii}.type, 'gravitational')
        g = model_def.forces{ii}.g;
        % delete vector
        model_def.forces(ii) = [];
        break;
    end
end

for ii = 1:no_bodies
    pidxi = 4*ii-3:4*ii;
    p(pidxi) = [model_def.bodies{ii}.pi
        model_def.bodies{ii}.pj];
    Lbody(ii) = norm(model_def.bodies{ii}.pi-model_def.bodies{ii}.pj);
    Mp(pidxi, pidxi) = Mp(pidxi, pidxi) + massMatrix(model_def.bodies{ii}.m);
    Qg(pidxi) = Qg(pidxi) + externalForce(0.5, model_def.bodies{ii}.m*g);
end

[ helper, model_def ] = eliminateCoordinates( model_def, p );

% after joint elimination - preprocess joints
nJoints = length(model_def.joints);

helperJoints = cell(nJoints, 1);
for ii = 1:nJoints
    if ~strcmp(model_def.joints{ii}.type, 'fix_orientation')
        error('Unknown joint type after coordinate elimination');
    end
    ib = model_def.joints{ii}.ibody;
    jb = model_def.joints{ii}.jbody;
    vi = model_def.bodies{ib}.pi - model_def.bodies{ib}.pj;
    vj = model_def.bodies{jb}.pi - model_def.bodies{jb}.pj;
    dotValue = vi'*vj;
    if abs(dotValue) < 1e-3
        error('Value close to zero in fix_orientation joint');
    end
    helperJoints{ii} = struct('dotValue', dotValue);
end

helper.joints = helperJoints;

% now coordinate vector
q0 = p(helper.rindep);

helper.M = helper.Cd'*Mp*helper.Cd;
helper.Qg = helper.Cd'*Qg;
helper.Lbody = Lbody;
helper.np = np;
nq = length(q0);
helper.nq = nq;

% run ode
if ~strcmp(settings.method.type, 'baumgarte')
    error('Only can solve using Baumgarte method');
end

opts = odeset('AbsTol', settings.ode.abstol, 'RelTol', settings.ode.reltol);
odefun = @(t, y) [y(nq+1:end)
    systemAccelerations(model_def, settings, helper, t, y)];
y0 = [q0; zeros(size(q0))];
if strcmp(settings.ode.solver, 'ode45')
    [T, Y] = ode45(odefun, settings.tspan, y0, opts);
else
    error('Unknown ode solver');
end

result.T = T;
% compute P and PP
P = zeros(length(T), np);
PP = zeros(length(T), np);
for ii = 1:length(T)
    q = Y(ii, 1:helper.nq)';
    qp = Y(ii, helper.nq+1:end)';

    p = helper.p0 + helper.Cd*q;
    pp = helper.Cd*qp;
    P(ii, :) = p';
    PP(ii, :) = pp';
end
result.P = P;
result.PP = PP;
