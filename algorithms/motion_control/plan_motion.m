function [public_vars] = plan_motion(read_only_vars, public_vars)

target = get_target(public_vars.estimated_pose, public_vars.path);

% Stay still for data collection (Tasks 2-4)
public_vars.motion_vector = [0, 0];

% --- Task 5: open-loop path for indoor_1 ---
% start_position = [1.65, 1, pi/2], map = indoor_1
% [v, v] = straight, [-vt, vt] = right turn, [vt, -vt] = left turn
%
% k = read_only_vars.counter;
% v  = 1.0;
% vt = 0.5236;
%
% if     k <= 75,  public_vars.motion_vector = [v, v];      % up
% elseif k <= 78,  public_vars.motion_vector = [-vt, vt];   % right turn
% elseif k <= 114, public_vars.motion_vector = [v, v];      % right
% elseif k <= 117, public_vars.motion_vector = [-vt, vt];   % right turn
% elseif k <= 187, public_vars.motion_vector = [v, v];      % down
% elseif k <= 190, public_vars.motion_vector = [vt, -vt];   % left turn
% elseif k <= 224, public_vars.motion_vector = [v, v];      % right
% elseif k <= 227, public_vars.motion_vector = [vt, -vt];   % left turn
% elseif k <= 302, public_vars.motion_vector = [v, v];      % up
% else,            public_vars.motion_vector = [0, 0];
% end

end
