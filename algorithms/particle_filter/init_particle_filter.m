function [public_vars] = init_particle_filter(read_only_vars, public_vars)

N = read_only_vars.max_particles;
lim = read_only_vars.map.limits;

% random particles within map bounds
x = lim(1) + (lim(3) - lim(1)) * rand(N, 1);
y = lim(2) + (lim(4) - lim(2)) * rand(N, 1);
theta = -pi + 2 * pi * rand(N, 1);

public_vars.particles = [x, y, theta];

end
