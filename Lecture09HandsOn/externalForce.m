function [ Qex ] = externalForce(c1, fv)

DT = [1-c1 0
    0 1-c1
    c1 0
    0 c1];

Qex = DT*fv;