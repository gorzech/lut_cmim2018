function [ helper ] = eliminateCoordinates( model_def, p )

np = length(p);
no_joints = length(model_def.joints);
Cqc = zeros(2*no_joints, np);

joffset = 0; % offset of coordinates
for i = 1:no_joints
    jtype = model_def.joints(i).type;
    ib = model_def.joints(i).ibody;
    jb = model_def.joints(i).jbody;
    jp = model_def.joints(i).p;
    if ~strcmp(jtype, 'revolute')
        error('Unknown joint type!');
    end
    if ib > 0
        % first body is ib - check point
        if norm(model_def.bodies(ib).pi - jp) < 1e-12
            pidx = [4*ib-3, 4*ib-2];
        elseif norm(model_def.bodies(ib).pj - jp) < 1e-12
            pidx = [4*ib-1, 4*ib];
        else 
            error('Point doesn''t match in joint');
        end
        Cqc(joffset + (1:2), pidx) = eye(2);
    end
    if jb > 0
        % first body is ib - check point
        if norm(model_def.bodies(jb).pi - jp) < 1e-12
            pidx = [4*jb-3, 4*jb-2];
        elseif norm(model_def.bodies(jb).pj - jp) < 1e-12
            pidx = [4*jb-1, 4*jb];
        else 
            error('Point doesn''t match in joint');
        end
        Cqc(joffset + (1:2), pidx) = -eye(2);
    end
    joffset = joffset + 2;
end

% perform coordinate partitioning into dependent and independent
[~, ~, rowp] = lu(Cqc', 'vector');
rdep = sort(rowp(1:2*no_joints));
rindep = sort(rowp(2*no_joints+1:end));
nq = length(rindep);
Cdi = Cqc(:, rdep)\Cqc(:, rindep);
Cd = zeros(np, nq);
Cd(rindep, :) = eye(nq);
Cd(rdep, :) = -Cdi;

q = p(rindep);
ptemp = Cd*q;
p0 = zeros(size(p));
p0(rdep) = p(rdep) - ptemp(rdep);

helper.Cd = Cd;
helper.rdep = rdep;
helper.rindep = rindep;
helper.p0 = p0;

end

