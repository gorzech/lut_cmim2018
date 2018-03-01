function compareResults( t1, y1, t2, y2 )
%compareResults Compare two sets of results

nt1 = length(t1);
nt2 = length(t2);
if nt1 > nt2
    error('Wrong sizes')
end
divFactor = round(nt2/nt1);
% verify
diffT = norm(t2(1:divFactor:end)-t1); % if zero indexes match

% so test solution
rmseY1 = rmse(y2(1:divFactor:end, 1), y1(:, 1));
rmseY2 = rmse(y2(1:divFactor:end, 2), y1(:, 2));
rmseY3 = rmse(y2(1:divFactor:end, 3), y1(:, 3));
rmseY4 = rmse(y2(1:divFactor:end, 4), y1(:, 4));

disp(['Diff in time: ',num2str(diffT), ...
    ' rmse in Y1: ', num2str(rmseY1), ...
    ' Y2: ', num2str(rmseY2), ...
    ' Y3: ', num2str(rmseY3), ...
    ' Y4: ', num2str(rmseY4)])


end

