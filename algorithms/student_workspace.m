function [public_vars] = student_workspace(read_only_vars, public_vars)

if read_only_vars.counter == 1
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars.mode = 'task4';

    public_vars = init_kalman_filter(read_only_vars, public_vars);
    public_vars.kf_enabled = 1;
    public_vars.pf_enabled = 0;
    public_vars.path = generate_path();
    public_vars.gnss_init_data = [];
    public_vars.init_sample_target = 120;
    public_vars.init_done = false;
end

if ~public_vars.init_done
    z = read_only_vars.gnss_position;
    if ~isempty(z) && ~any(isnan(z))
        public_vars.gnss_init_data = [public_vars.gnss_init_data; z];
    end

    if size(public_vars.gnss_init_data, 1) >= public_vars.init_sample_target
        public_vars.gnss_mean = mean(public_vars.gnss_init_data, 1);
        public_vars.gnss_cov = cov(public_vars.gnss_init_data);
        if any(isnan(public_vars.gnss_cov(:))) || isempty(public_vars.gnss_cov)
            public_vars.gnss_cov = diag([0.05, 0.05]);
        end
        public_vars = init_kalman_filter(read_only_vars, public_vars);
        public_vars.init_done = true;
        fprintf('GNSS mean: %.4f %.4f\n', public_vars.gnss_mean(1), public_vars.gnss_mean(2));
        disp('GNSS covariance:');
        disp(public_vars.gnss_cov);
        disp('Process covariance:');
        disp(public_vars.kf.R);
    end

    public_vars.motion_vector = [0, 0];
    public_vars.estimated_pose = estimate_pose(public_vars);
    return;
end

public_vars.particles = update_particle_filter(read_only_vars, public_vars);
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
public_vars.estimated_pose = estimate_pose(public_vars);
public_vars = plan_motion(read_only_vars, public_vars);

end


function path = generate_path()

waypoints = [
    2.0 2.0
    2.0 3.5
    2.0 5.5
    3.5 6.6
    5.5 7.2
    7.5 7.8
    9.5 7.8
    11.5 7.4
    13.0 6.2
    14.5 4.6
    15.6 3.2
    16.0 2.0
];

path = [];
for i = 1:size(waypoints, 1) - 1
    a = waypoints(i, :);
    b = waypoints(i + 1, :);
    d = norm(b - a);
    n = max(2, ceil(d / 0.3));
    t = linspace(0, 1, n)';
    part = (1 - t) .* a + t .* b;
    if i > 1
        part = part(2:end, :);
    end
    path = [path; part];
end

end
