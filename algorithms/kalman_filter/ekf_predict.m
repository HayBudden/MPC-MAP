function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, sampling_period)

if isempty(u)
    u = [0, 0];
end

vR = u(1);
vL = u(2);
v = (vR + vL) / 2;
omega = (vR - vL) / kf.L;
theta = mu(3);

new_mu = [
    mu(1) + v * cos(theta) * sampling_period
    mu(2) + v * sin(theta) * sampling_period
    mu(3) + omega * sampling_period
];

G = [
    1 0 -sin(theta) * v * sampling_period
    0 1  cos(theta) * v * sampling_period
    0 0 1
];

new_sigma = G * sigma * G' + kf.R;
new_mu(3) = atan2(sin(new_mu(3)), cos(new_mu(3)));
new_sigma = (new_sigma + new_sigma') / 2;

end

