function [public_vars] = student_workspace(read_only_vars, public_vars)
if (read_only_vars.counter == 1)
    public_vars.gnss_available = false;
    public_vars.kf_initialized = false;
    public_vars.init_phase = true;
    public_vars.init_iterations = 25;
    public_vars.poor_fit_count = 0;
    public_vars.pf_converged = false;
    public_vars.prev_estimated_pose = nan(1, 3);
    public_vars.position_uncertain = false;
    public_vars.goal_stuck_count = 0;
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);
end
gnss = read_only_vars.gnss_position;
public_vars.gnss_available = ~isempty(gnss) && ~any(isnan(gnss));
if public_vars.gnss_available && ~public_vars.kf_initialized
    public_vars.mu = [gnss(1); gnss(2); 0];
    public_vars.sigma = diag([0.5, 0.5, pi^2]);
    public_vars.kf_initialized = true;
end
[public_vars.particles, public_vars.poor_fit_count] = update_particle_filter(read_only_vars, public_vars);
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
public_vars.estimated_pose = estimate_pose(public_vars);
if ~any(isnan(public_vars.estimated_pose))
    pose_jump = 0;
    if ~any(isnan(public_vars.prev_estimated_pose))
        pose_jump = norm(public_vars.estimated_pose(1:2) - public_vars.prev_estimated_pose(1:2));
    end
    public_vars.prev_estimated_pose = public_vars.estimated_pose;
    particles = public_vars.particles;
    spread_x = 0;
    spread_y = 0;
    if ~isempty(particles) && size(particles, 1) > 1
        spread_x = std(particles(:,1));
        spread_y = std(particles(:,2));
    end
    public_vars.position_uncertain = (pose_jump > 0.15) || (spread_x > 0.5) || (spread_y > 0.5);
    goal_pos = read_only_vars.map.goal(1:2);
    dist_to_goal_est = norm(public_vars.estimated_pose(1:2) - goal_pos);
    if dist_to_goal_est < 0.5
        public_vars.goal_stuck_count = public_vars.goal_stuck_count + 1;
    else
        public_vars.goal_stuck_count = 0;
    end
else
    public_vars.position_uncertain = true;
end
if read_only_vars.counter <= public_vars.init_iterations
    if public_vars.gnss_available && public_vars.kf_initialized && read_only_vars.counter >= 5
        public_vars.pf_converged = true;
        public_vars.init_phase = false;
        public_vars.poor_fit_count = 0;
    else
        rot_speed = 0.4;
        public_vars.motion_vector = [rot_speed, -rot_speed];
        return;
    end
end
if ~public_vars.pf_converged
    particles = public_vars.particles;
    if ~isempty(particles) && size(particles, 1) > 1
        spread_x = std(particles(:,1));
        spread_y = std(particles(:,2));
        max_extra_rotation = 15;
        time_exceeded = read_only_vars.counter > (public_vars.init_iterations + max_extra_rotation);
        if (spread_x < 0.5 && spread_y < 0.5) || time_exceeded
            public_vars.pf_converged = true;
            public_vars.init_phase = false;
            public_vars.poor_fit_count = 0;
        else
            rot_speed = 0.4;
            public_vars.motion_vector = [rot_speed, -rot_speed];
            return;
        end
    end
end
if ~any(isnan(public_vars.estimated_pose))
    if ~public_vars.init_phase && (public_vars.goal_stuck_count > 15 || public_vars.poor_fit_count >= 10)
        public_vars = init_particle_filter(read_only_vars, public_vars);
        public_vars.pf_converged = false;
        public_vars.init_phase = true;
        public_vars.goal_stuck_count = 0;
        public_vars.poor_fit_count = 0;
        public_vars.init_iterations = read_only_vars.counter + 20;
        rot_speed = 0.4;
        public_vars.motion_vector = [rot_speed, -rot_speed];
        return;
    end
    public_vars.path = plan_path(read_only_vars, public_vars);
end
public_vars = plan_motion(read_only_vars, public_vars);
end
