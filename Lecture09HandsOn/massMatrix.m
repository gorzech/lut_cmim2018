function [ M ] = massMatrix( m )

m3 = m/3; m6 = m/6;
M = [m3 0 m6 0
    0 m3 0 m6
    m6 0 m3 0
    0 m6 0 m3];


end

