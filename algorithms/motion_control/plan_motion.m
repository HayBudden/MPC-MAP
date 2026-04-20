function [public_vars] = plan_motion(read_only_vars, public_vars)

pose = public_vars.estimated_pose;

if isempty(pose) || any(isnan(pose)) || isempty(public_vars.path)
    public_vars.motion_vector = [0, 0];
    return;
end

target = get_target(pose, public_vars.path);

dx = target(1) - pose(1);
dy = target(2) - pose(2);
dist = sqrt(dx^2 + dy^2);

goal = public_vars.path(end, :);
dist_to_goal = sqrt((goal(1) - pose(1))^2 + (goal(2) - pose(2))^2);
if dist_to_goal < 0.5
    public_vars.motion_vector = [0, 0];
    return;
end

desired_heading = atan2(dy, dx);
heading_error = atan2(sin(desired_heading - pose(3)), cos(desired_heading - pose(3)));

Kv = 0.8;
Kw = 2.0;

v = Kv * dist * max(0, cos(heading_error));
v = min(v, read_only_vars.agent_drive.max_vel);
omega = Kw * heading_error;

L = read_only_vars.agent_drive.interwheel_dist;
v_right = v + omega * L / 2;
v_left = v - omega * L / 2;

max_v = read_only_vars.agent_drive.max_vel;
v_right = max(min(v_right, max_v), -max_v);
v_left = max(min(v_left, max_v), -max_v);

public_vars.motion_vector = [v_right, v_left];

end
