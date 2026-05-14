function [particles, poor_fit_count] = update_particle_filter(read_only_vars, public_vars)
particles = public_vars.particles;
poor_fit_count = public_vars.poor_fit_count;
if isempty(particles)
    return;
end
N = size(particles, 1);
for i = 1:N
    particles(i,:) = predict_pose(particles(i,:), public_vars.motion_vector, read_only_vars);
end
measurements = zeros(N, length(read_only_vars.lidar_config));
for i = 1:N
    measurements(i,:) = compute_lidar_measurement(read_only_vars.map, particles(i,:), read_only_vars.lidar_config);
end
weights = weight_particles(measurements, read_only_vars.lidar_distances);
avg_weight = 1 / N;
max_weight = max(weights);
poor_fit = max_weight < 2 * avg_weight;
if poor_fit
    poor_fit_count = poor_fit_count + 1;
else
    poor_fit_count = 0;
end
is_init = read_only_vars.counter <= public_vars.init_iterations;
if is_init
    temperature = 0.5;
    weights = weights .^ temperature;
    weights = weights / sum(weights);
end
particles = resample_particles(particles, weights, is_init);
n_gnss = 0;
gnss = read_only_vars.gnss_position;
if ~isempty(gnss) && ~any(isnan(gnss))
    n_gnss = max(2, round(0.05 * N));
    mean_theta = atan2(mean(sin(particles(:,3))), mean(cos(particles(:,3))));
    for k = 1:n_gnss
        gx = gnss(1) + 0.3 * randn;
        gy = gnss(2) + 0.3 * randn;
        gtheta = mean_theta + 0.3 * randn;
        particles(N - k + 1, :) = [gx, gy, gtheta];
    end
end
if poor_fit_count >= 2
    n_random = max(2, round(0.20 * N));
    limits = read_only_vars.map.limits;
    injected = 0;
    max_tries = n_random * 20;
    tries = 0;
    while injected < n_random && tries < max_tries
        tries = tries + 1;
        x = limits(1) + (limits(3) - limits(1)) * rand;
        y = limits(2) + (limits(4) - limits(2)) * rand;
        theta = -pi + 2 * pi * rand;
        pose = [x, y, theta];
        dists = compute_lidar_measurement(read_only_vars.map, pose, read_only_vars.lidar_config);
        if all(dists > 0.15)
            injected = injected + 1;
            particles(N - n_gnss - injected + 1, :) = pose;
        end
    end
    particles(:,1) = particles(:,1) + 0.08 * randn(N, 1);
    particles(:,2) = particles(:,2) + 0.08 * randn(N, 1);
    particles(:,3) = particles(:,3) + 0.05 * randn(N, 1);
end
end
