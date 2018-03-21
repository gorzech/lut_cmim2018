function [ C, Cq, Cp, G ] = Constraints( y, L )

r1 = y(1:2);
fi1 = y(3);
r2 = y(4:5);
fi2 = y(6);

A1 = rot(fi1);
A2 = rot(fi2);

u1_1 = [ -L/2; 0 ];

u2_1 = [ L/2; 0 ];
u2_2 = [ -L/2; 0 ];

C = [r1 + A1*u1_1
    r1 + A1*u2_1 - r2 - A2*u2_2];

I2 = eye(2);
Om = [0, -1; 1, 0];
Cq = [I2, Om*A1*u1_1, zeros(2, 3)
    I2, Om*A1*u2_1, -I2, -Om*A2*u2_2];

qp = y(7:12);

Cp = Cq*qp;
fi1p = qp(3);
fi2p = qp(6);

G = -[-A1*u1_1*fi1p*fi1p
    -A1*u2_1*fi1p*fi1p+A2*u2_2*fi2p*fi2p];

end

function A = rot(fi)
c = cos(fi);
s = sin(fi);
A = [c, -s
    s, c];
end