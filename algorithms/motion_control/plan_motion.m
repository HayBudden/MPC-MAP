function [public_vars] = plan_motion(read_only_vars, public_vars)
pose = public_vars.estimated_pose;
if isempty(pose) || any(isnan(pose)) || isempty(public_vars.path)
    public_vars.motion_vector = [0, 0];
    return;
end
goal = read_only_vars.map.goal(1:2);
dist_to_goal = sqrt((goal(1) - pose(1))^2 + (goal(2) - pose(2))^2);
if dist_to_goal < 0.3
    public_vars.motion_vector = [0, 0];
    return;
end
path = public_vars.path;
target = get_target(pose, path);
dx = target(1) - pose(1);
dy = target(2) - pose(2);
desired_heading = atan2(dy, dx);
heading_error = atan2(sin(desired_heading - pose(3)), cos(desired_heading - pose(3)));
max_v = read_only_vars.agent_drive.max_vel;
L = read_only_vars.agent_drive.interwheel_dist;
curvature_factor = compute_curvature_factor(pose, path);
abs_error = abs(heading_error);
if abs_error > pi/2
    v = 0;
elseif abs_error > pi/4
    v = 0.3 * max_v * (1 - (abs_error - pi/4) / (pi/4));
else
    v = max_v * (1 - abs_error / (pi/4) * 0.1);
end
v = v * curvature_factor;
if isfield(public_vars, 'position_uncertain') && public_vars.position_uncertain
    v = v * 0.3;
end
if dist_to_goal < 1.0
    v = v * max(0.3, dist_to_goal / 1.0);
end
Kw = 2.5;
omega = Kw * heading_error;
max_omega = 2 * max_v / L;
omega = max(min(omega, max_omega), -max_omega);
v_right = v + omega * L / 2;
v_left = v - omega * L / 2;
max_wheel = max(abs(v_right), abs(v_left));
if max_wheel > max_v
    v_right = v_right * max_v / max_wheel;
    v_left = v_left * max_v / max_wheel;
end
public_vars.motion_vector = [v_right, v_left];
end
function factor = compute_curvature_factor(pose, path)
pos = pose(1:2);
dists = sqrt(sum((path - pos).^2, 2));
[~, closest_idx] = min(dists);
look_points = 5;
end_idx = min(size(path, 1), closest_idx + look_points);
if end_idx - closest_idx < 2
    factor = 1.0;
    return;
end
segment = path(closest_idx:end_idx, :);
max_angle_change = 0;
for i = 2:size(segment, 1) - 1
    v1 = segment(i, :) - segment(i-1, :);
    v2 = segment(i+1, :) - segment(i, :);
    len1 = sqrt(sum(v1.^2));
    len2 = sqrt(sum(v2.^2));
    if len1 > 1e-6 && len2 > 1e-6
        cos_angle = dot(v1, v2) / (len1 * len2);
        cos_angle = max(-1, min(1, cos_angle));
        angle_change = acos(cos_angle);
        if angle_change > max_angle_change
            max_angle_change = angle_change;
        end
    end
end
if max_angle_change > pi/2
    factor = 0.3;
elseif max_angle_change > pi/4
    factor = 0.3 + 0.7 * (1 - (max_angle_change - pi/4) / (pi/2 - pi/4));
else
    factor = 1.0;
end
end
