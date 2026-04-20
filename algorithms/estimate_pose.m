function [estimated_pose] = estimate_pose(public_vars)

if isempty(public_vars.mu)
    estimated_pose = nan(1, 3);
    return;
end

estimated_pose = public_vars.mu(:)';
estimated_pose(3) = atan2(sin(estimated_pose(3)), cos(estimated_pose(3)));

end

