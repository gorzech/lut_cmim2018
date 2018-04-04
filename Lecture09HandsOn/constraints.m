function [ C, Cq, Cp, G ] = constraints(model_def, helper, p, pp)

nBodies = length(model_def.bodies);
nJoints = length(model_def.joints);
np = helper.np;

nConstr = nBodies + nJoints;

C = zeros(nConstr, 1);
Cq = zeros(nConstr, np);
% Cp = C;
G = C;

for ii = 1:nBodies
    dx = p(4*ii-3) - p(4*ii-1);
    dy = p(4*ii-2) - p(4*ii);
    C(ii) = dx*dx + dy*dy - helper.Lbody(ii)^2;
    Cq(ii, 4*ii-3) = 2*dx;
    Cq(ii, 4*ii-2) = 2*dy;
    Cq(ii, 4*ii-1) = -2*dx;
    Cq(ii, 4*ii) = -2*dy;
    % Cq * qp = 2*dx * dxp + 2dy*dyp
    dxp = pp(4*ii-3) - pp(4*ii-1);
    dyp = pp(4*ii-2) - pp(4*ii);
    G(ii) = - 2*dxp*dxp - 2*dyp*dyp;
end
for ii = 1:nJoints
    ib = model_def.joints{ii}.ibody;
    jb = model_def.joints{ii}.jbody;
    vi = p(4*ib-3:4*ib-2) - p(4*ib-1:4*ib);
    vj = p(4*jb-3:4*jb-2) - p(4*jb-1:4*jb);
    dotValue = vi'*vj;
    C(ii + nBodies) = dotValue - helper.joints{ii}.dotValue;
    Cq(ii + nBodies, 4*ib-3:4*ib-2) = vj';
    Cq(ii + nBodies, 4*ib-1:4*ib) = -vj';
    Cq(ii + nBodies, 4*jb-3:4*jb-2) = vi';
    Cq(ii + nBodies, 4*jb-1:4*jb) = -vi';
    vip = pp(4*ib-3:4*ib-2) - pp(4*ib-1:4*ib);
    vjp = pp(4*jb-3:4*jb-2) - pp(4*jb-1:4*jb);
    G(ii + nBodies) = -2*vip'*vjp;
end

Cp = Cq*pp;

Cq = Cq*helper.Cd;

