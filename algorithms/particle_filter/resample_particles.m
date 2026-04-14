function [new_particles] = resample_particles(particles, weights)

N = size(particles, 1);
cdf = cumsum(weights);
cdf = cdf / cdf(end);

% systematic resampling
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

% roughening - add small noise to prevent particle depletion
new_particles(:,1) = new_particles(:,1) + 0.02 * randn(N, 1);
new_particles(:,2) = new_particles(:,2) + 0.02 * randn(N, 1);
new_particles(:,3) = new_particles(:,3) + 0.01 * randn(N, 1);

end
