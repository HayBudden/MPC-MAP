function [new_mu, new_sigma] = kf_measure(mu, sigma, z, kf)

new_mu = mu;
new_sigma = sigma;

if isempty(z) || any(isnan(z))
    return;
end

z = z(:);
H = kf.C;
S = H * sigma * H' + kf.Q;
K = sigma * H' / S;

new_mu = mu + K * (z - H * mu);
new_sigma = (eye(3) - K * H) * sigma;

new_mu(3) = atan2(sin(new_mu(3)), cos(new_mu(3)));
new_sigma = (new_sigma + new_sigma') / 2;

end

