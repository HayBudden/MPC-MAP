function [public_vars] = plan_motion(read_only_vars, public_vars)

pose = read_only_vars.mocap_pose;

if isempty(pose) || isempty(public_vars.path)
    public_vars.motion_vector = [0, 0];
    return;
end

target = get_target(pose, public_vars.path);

% vector from robot to target
dx = target(1) - pose(1);
dy = target(2) - pose(2);
dist = sqrt(dx^2 + dy^2);

% stop if close enough to final waypoint
goal = public_vars.path(end, :);
dist_to_goal = sqrt((goal(1)-pose(1))^2 + (goal(2)-pose(2))^2);
if dist_to_goal < 0.5
    public_vars.motion_vector = [0, 0];
    return;
end

% desired heading and heading error
desired_heading = atan2(dy, dx);
heading_error = desired_heading - pose(3);

% [-pi, pi]
heading_error = atan2(sin(heading_error), cos(heading_error));

% controller gains
Kv = 0.8;   % forward speed gain
Kw = 2.0;   % turning gain

v = Kv * dist;
v = min(v, read_only_vars.agent_drive.max_vel);

omega = Kw * heading_error;

% convert (v, omega) to (v_right, v_left)
L = read_only_vars.agent_drive.interwheel_dist;
v_right = v + omega * L / 2;
v_left  = v - omega * L / 2;

% clamp to max velocity
max_v = read_only_vars.agent_drive.max_vel;
v_right = max(min(v_right, max_v), -max_v);
v_left  = max(min(v_left,  max_v), -max_v);

public_vars.motion_vector = [v_right, v_left];

end
