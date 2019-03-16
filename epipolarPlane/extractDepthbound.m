function depthbound = extractDepthbound(D, min_disp, max_disp, min_bound, constant)
    [n, m] = size(D);
    depthbound_min = ones([n,m])* min_disp;
    depthbound_max = ones([n,m])* max_disp;
    D_mask = (D~= constant);
    % fill up depthbound
    for i = 1:size(D,1) % row by row
        row =find(D_mask(i,:));
        if isempty(row)
            continue
        end
        depthbound_max(i, 1:row(1)) = D(i,row(1));
        depthbound_min(i, 1:row(1)) = D(i,row(1));
        for j = 1:size(row,2)-1
            left = D(i, row(j));
            right = D(i, row(j+1));
            if right > left
                depthbound_max(i, row(j):row(j+1)) = right;
                depthbound_min(i, row(j):row(j+1)) = left;
            else
                depthbound_max(i, row(j):row(j+1)) = left;
                depthbound_min(i, row(j):row(j+1)) = right;
            end
        end
        depthbound_max(i, row(end):end) = D(i,row(end));
        depthbound_min(i, row(end):end) = D(i,row(end));
    end
    % check min_bound
    diff = depthbound_max - depthbound_min;
    revisitMask = (diff < min_bound);
    depthbound_min(revisitMask) = depthbound_min(revisitMask) - 0.5*(min_bound-diff(revisitMask));
    depthbound_max(revisitMask) = depthbound_max(revisitMask) + 0.5*(min_bound-diff(revisitMask));
    % final check
    depthbound_min(depthbound_min < min_disp) = min_disp;
    depthbound_max(depthbound_max > max_disp) = max_disp;
    depthbound_min(D_mask) = D(D_mask);
    depthbound_max(D_mask) = D(D_mask);
    depthbound = cat(3, depthbound_min, depthbound_max);
end