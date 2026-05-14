function [weights] = weight_particles(particle_measurements, lidar_distances)
N = size(particle_measurements, 1);
sigma = 0.25;
weights = zeros(N, 1);
for i = 1:N
    log_w = 0;
    n_valid = 0;
    for j = 1:length(lidar_distances)
        real_val = lidar_distances(j);
        pred_val = particle_measurements(i, j);
        if isinf(real_val) && isinf(pred_val)
            continue;
        end
        if isinf(real_val) || isinf(pred_val)
            log_w = log_w - 5;
            n_valid = n_valid + 1;
            continue;
        end
        diff = pred_val - real_val;
        log_w = log_w - 0.5 * (diff / sigma)^2;
        n_valid = n_valid + 1;
    end
    if n_valid == 0
        weights(i) = 1e-300;
    else
        weights(i) = exp(log_w);
    end
end
total = sum(weights);
if total > 0
    weights = weights / total;
else
    weights = ones(N, 1) / N;
end
end
