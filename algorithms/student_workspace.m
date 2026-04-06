function [public_vars] = student_workspace(read_only_vars, public_vars)

if (read_only_vars.counter == 1)
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);

    % Select which path to use: 'line', 'arc', 'sine'
    path_type = 'sine';
    public_vars.path = generate_path(path_type);
end

% Filters (stubs, not used this week)
public_vars.particles = update_particle_filter(read_only_vars, public_vars);
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
public_vars.estimated_pose = estimate_pose(public_vars);

% Motion control
public_vars = plan_motion(read_only_vars, public_vars);

end


function path = generate_path(type)

switch type
    case 'line'
        % straight line from (2,5) to (14,5), safe horizontal corridor
        N = 50;
        x = linspace(2, 14, N)';
        y = 5 * ones(N, 1);
        path = [x, y];

    case 'arc'
        % circular arc, center (8,5), radius 2
        cx = 8; cy = 5; r = 2;
        theta = linspace(pi, 0, 60)';
        x = cx + r * cos(theta);
        y = cy + r * sin(theta);
        path = [x, y];

    case 'sine'
        % sine wave through the safe corridor
        N = 80;
        x = linspace(2, 14, N)';
        y = 7 + 1.5 * sin(2 * pi * (x - 2) / 12);
        path = [x, y];

    otherwise
        path = [2, 5; 14, 5];
end

end
