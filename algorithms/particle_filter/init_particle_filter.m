function [public_vars] = init_particle_filter(read_only_vars, public_vars)
N = 500;
limits = read_only_vars.map.limits;
x_min = limits(1);
y_min = limits(2);
x_max = limits(3);
y_max = limits(4);
real_lidar = read_only_vars.lidar_distances;
sigma = 0.25;
n_candidates = N * 20;
candidates = zeros(n_candidates, 3);
scores = zeros(n_candidates, 1);
n_valid = 0;
max_attempts = n_candidates * 4;
attempts = 0;
while n_valid < n_candidates && attempts < max_attempts
    attempts = attempts + 1;
    x = x_min + (x_max - x_min) * rand;
    y = y_min + (y_max - y_min) * rand;
    theta = -pi + 2 * pi * rand;
    pose = [x, y, theta];
    pred_lidar = compute_lidar_measurement(read_only_vars.map, pose, read_only_vars.lidar_config);
    if all(pred_lidar > 0.1)
        n_valid = n_valid + 1;
        candidates(n_valid, :) = pose;
        log_score = 0;
        for j = 1:length(real_lidar)
            if isinf(real_lidar(j)) && isinf(pred_lidar(j))
                continue;
            elseif isinf(real_lidar(j)) || isinf(pred_lidar(j))
                log_score = log_score - 5;
            else
                diff = pred_lidar(j) - real_lidar(j);
                log_score = log_score - 0.5 * (diff / sigma)^2;
            end
        end
        scores(n_valid) = log_score;
    end
end
candidates = candidates(1:n_valid, :);
scores = scores(1:n_valid);
scores = scores - max(scores);
temperature = 0.5;
weights = exp(scores * temperature);
weights = weights / sum(weights);
cdf = cumsum(weights);
particles = zeros(N, 3);
r = rand / N;
j = 1;
for i = 1:N
    u = r + (i - 1) / N;
    while u > cdf(j)
        j = j + 1;
    end
    particles(i, :) = candidates(j, :);
end
particles(:,1) = particles(:,1) + 0.05 * randn(N, 1);
particles(:,2) = particles(:,2) + 0.05 * randn(N, 1);
particles(:,3) = particles(:,3) + 0.05 * randn(N, 1);
public_vars.particles = particles;
end
