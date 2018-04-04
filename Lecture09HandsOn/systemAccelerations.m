function acc = systemAccelerations(model_def, settings, helper, t, y)

q = y(1:helper.nq);
qp = y(helper.nq+1:end);

p = helper.p0 + helper.Cd*q;
pp = helper.Cd*qp;

[ C, Cq, Cp, G ] = constraints(model_def, helper, p, pp);

% forces
Qp = zeros(helper.np, 1);
for ii = 1:length(model_def.forces)
    force = model_def.forces{ii};
    if strcmp(force.type, 'point')
        ibody = force.ibody;
        Qp(4*ibody-3:4*ibody) = Qp(4*ibody-3:4*ibody) + externalForce(force.c, force.F(t));
    end
end
Qq = helper.Qg + helper.Cd'*Qp;

LHS = [helper.M, Cq'
    Cq, zeros(length(C))];

beta = settings.method.beta;
RHS = [Qq
    G - 2*settings.method.alfa*Cp - beta*beta*C];

x = LHS\RHS;

acc = x(1:helper.nq);