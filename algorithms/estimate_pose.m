function [estimated_pose] = estimate_pose(public_vars)

if isempty(public_vars.particles)
    estimated_pose = nan(1, 3);
    return;
end

% mean of all particles
estimated_pose = mean(public_vars.particles, 1);

% wrap angle to [-pi, pi]
estimated_pose(3) = atan2(sin(estimated_pose(3)), cos(estimated_pose(3)));

end
