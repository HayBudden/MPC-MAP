function [new_particles] = resample_particles(particles, weights, is_init)
if nargin < 3
    is_init = false;
end
N = size(particles, 1);
n_eff = 1 / sum(weights.^2);
if is_init
    resample_threshold = 0.75 * N;
else
    resample_threshold = N / 2;
end
if n_eff > resample_threshold
    new_particles = particles;
    return;
end
cdf = cumsum(weights);
cdf = cdf / cdf(end);
r = rand / N;
new_particles = zeros(size(particles));
j = 1;
for i = 1:N
    u = r + (i - 1) / N;
    while u > cdf(j)
        j = j + 1;
    end
    new_particles(i, :) = particles(j, :);
end
if is_init
    new_particles(:,1) = new_particles(:,1) + 0.03 * randn(N, 1);
    new_particles(:,2) = new_particles(:,2) + 0.03 * randn(N, 1);
    new_particles(:,3) = new_particles(:,3) + 0.02 * randn(N, 1);
else
    new_particles(:,1) = new_particles(:,1) + 0.04 * randn(N, 1);
    new_particles(:,2) = new_particles(:,2) + 0.04 * randn(N, 1);
    new_particles(:,3) = new_particles(:,3) + 0.03 * randn(N, 1);
end
end
