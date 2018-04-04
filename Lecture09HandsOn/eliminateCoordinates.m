function [ helper, model_def ] = eliminateCoordinates( model_def, p )

np = length(p);
no_joints = length(model_def.joints);
% to ensure the size loop over all joints
jointsToDelete = false(no_joints, 1);
nConstraints = 0;
for ii = 1:no_joints
    if strcmp(model_def.joints{ii}.type, 'revolute')
        nConstraints = nConstraints + 2;
        jointsToDelete(ii) = true;
    elseif strcmp(model_def.joints{ii}.type, 'fix_coordinates')
        nConstraints = nConstraints + length(model_def.joints{ii}.cidx);
        jointsToDelete(ii) = true;
    end
end

Cqc = zeros(nConstraints, np);

joffset = 0; % offset of coordinates
for ii = 1:no_joints
    jtype = model_def.joints{ii}.type;
    if strcmp(jtype, 'revolute')
        ib = model_def.joints{ii}.ibody;
        jb = model_def.joints{ii}.jbody;
        jp = model_def.joints{ii}.p;
        if ib > 0
            % first body is ib - check point
            if norm(model_def.bodies{ib}.pi - jp) < 1e-12
                pidx = [4*ib-3, 4*ib-2];
            elseif norm(model_def.bodies{ib}.pj - jp) < 1e-12
                pidx = [4*ib-1, 4*ib];
            else 
                error('Point doesn''t match in joint');
            end
            Cqc(joffset + (1:2), pidx) = eye(2);
        end
        if jb > 0
            % first body is ib - check point
            if norm(model_def.bodies{jb}.pi - jp) < 1e-12
                pidx = [4*jb-3, 4*jb-2];
            elseif norm(model_def.bodies{jb}.pj - jp) < 1e-12
                pidx = [4*jb-1, 4*jb];
            else 
                error('Point doesn''t match in joint');
            end
            Cqc(joffset + (1:2), pidx) = -eye(2);
        end
        joffset = joffset + 2;
    elseif strcmp(jtype, 'fix_coordinates')
        ib = model_def.joints{ii}.ibody;
        cidx = model_def.joints{ii}.cidx;
        pidx = 4*ib-3:4*ib;
        nC = length(cidx);
        Cqc(joffset + (1:nC), pidx(cidx)) = eye(nC);
        joffset = joffset + nC;
    end
end

if rank(Cqc) ~= nConstraints
    error('Jacobian is not full rank - cannot handle')
end
% perform coordinate partitioning into dependent and independent
[~, ~, rowp] = lu(Cqc', 'vector');
rdep = sort(rowp(1:nConstraints));
rindep = sort(rowp(nConstraints+1:end));
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

% delete linear joints - not to be confused with nonlinear
model_def.joints(jointsToDelete) = [];

end

