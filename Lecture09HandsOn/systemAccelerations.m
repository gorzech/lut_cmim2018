function acc = systemAccelerations(model_def, settings, helper, y)

q = y(1:helper.nq);
qp = y(helper.nq+1:end);

p = helper.p0 + helper.Cd*q;
pp = helper.Cd*qp;

[ C, Cq, Cp, G ] = constraints(model_def, helper, p, pp);

LHS = [helper.M, Cq'
    Cq, zeros(length(C))];

beta = settings.method.beta;
RHS = [helper.Qg
    G - 2*settings.method.alfa*Cp - beta*beta*C];

x = LHS\RHS;

acc = x(1:helper.nq);