function [ C, Cq, Cp, G ] = constraints(model_def, helper, p, pp)

no_constr = length(model_def.bodies);
np = length(p);

C = zeros(no_constr, 1);
Cq = zeros(no_constr, np);
% Cp = C;
G = C;

for i = 1:no_constr
    dx = p(4*i-3) - p(4*i-1);
    dy = p(4*i-2) - p(4*i);
    C(i) = dx*dx + dy*dy - helper.Lbody(i)^2;
    Cq(i, 4*i-3) = 2*dx;
    Cq(i, 4*i-2) = 2*dy;
    Cq(i, 4*i-1) = -2*dx;
    Cq(i, 4*i) = -2*dy;
    % Cq * qp = 2*dx * dxp + 2dy*dyp
    dxp = pp(4*i-3) - pp(4*i-1);
    dyp = pp(4*i-2) - pp(4*i);
    G(i) = - 2*dxp*dxp - 2*dyp*dyp;
end

Cp = Cq*pp;

Cq = Cq*helper.Cd;

