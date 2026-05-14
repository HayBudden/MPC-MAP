function [path] = plan_path(read_only_vars, public_vars)
planning_required = false;
if isempty(public_vars.path)
    planning_required = true;
else
    pos = public_vars.estimated_pose(1:2);
    dists = sqrt(sum((public_vars.path - pos).^2, 2));
    min_dist = min(dists);
    if min_dist > 1.5
        planning_required = true;
    end
end
if planning_required
    path = astar(read_only_vars, public_vars);
    path = smooth_path(path);
else
    path = public_vars.path;
end
end
