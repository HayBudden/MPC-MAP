function [weights] = weight_particles(particle_measurements, lidar_distances)

N = size(particle_measurements, 1);

% only compare beams where both real and predicted are finite
valid = ~isinf(lidar_distances);

sigma = 1.0;
weights = zeros(N, 1);

for i = 1:N
    v = valid & ~isinf(particle_measurements(i,:));
    if sum(v) == 0
        weights(i) = 1e-300;
        continue;
    end
    diff = particle_measurements(i, v) - lidar_distances(v);
    weights(i) = exp(-0.5 * sum(diff.^2) / sigma^2);
end

% normalize
total = sum(weights);
if total > 0
    weights = weights / total;
else
    weights = ones(N, 1) / N;
end

end
