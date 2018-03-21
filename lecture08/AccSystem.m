function [ qb ] = AccSystem( y, m, L, g, alfa, beta )

M = MassMatrix(m, L);
[ C, Cq, Cp, G ] = Constraints( y, L );
[ Qg ] = GravForces( m, g );

LHS = [M, Cq'
    Cq, zeros(4)];

RHS = [Qg
    G - 2*alfa*Cp - beta*beta*C];

x = LHS\RHS;

qb = x(1:6);

end

