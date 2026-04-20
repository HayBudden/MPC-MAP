function [mu, sigma] = update_kalman_filter(read_only_vars, public_vars)

mu = public_vars.mu;
sigma = public_vars.sigma;

if isempty(mu) || isempty(sigma)
    return;
end

u = public_vars.motion_vector;
[mu, sigma] = ekf_predict(mu, sigma, u, public_vars.kf, read_only_vars.sampling_period);

z = read_only_vars.gnss_position;
[mu, sigma] = kf_measure(mu, sigma, z, public_vars.kf);

end

