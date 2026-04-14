function [particles] = update_particle_filter(read_only_vars, public_vars)

particles = public_vars.particles;

% prediction
for i = 1:size(particles, 1)
    particles(i,:) = predict_pose(particles(i,:), public_vars.motion_vector, read_only_vars);
end

% compute expected measurements for each particle
measurements = zeros(size(particles, 1), length(read_only_vars.lidar_config));
for i = 1:size(particles, 1)
    measurements(i,:) = compute_lidar_measurement(read_only_vars.map, particles(i,:), read_only_vars.lidar_config);
end

% weighting
weights = weight_particles(measurements, read_only_vars.lidar_distances);

% resampling
particles = resample_particles(particles, weights);

end
