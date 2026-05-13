function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, sampling_period)
if isempty(mu) || isempty(sigma)
    new_mu = mu;
    new_sigma = sigma;
    return;
end
if isempty(u)
    u = [0, 0];
end
vR = u(1);
vL = u(2);
v = (vR + vL) / 2;
omega = (vR - vL) / kf.L;
theta = mu(3);
dt = sampling_period;
if abs(omega) > 1e-6
    r = v / omega;
    new_mu = [
        mu(1) + r * (sin(theta + omega*dt) - sin(theta));
        mu(2) - r * (cos(theta + omega*dt) - cos(theta));
        mu(3) + omega * dt
    ];
    G = [
        1 0 r*(cos(theta + omega*dt) - cos(theta));
        0 1 r*(sin(theta + omega*dt) - sin(theta));
        0 0 1
    ];
else
    new_mu = [
        mu(1) + v * cos(theta) * dt;
        mu(2) + v * sin(theta) * dt;
        mu(3) + omega * dt
    ];
    G = [
        1 0 -v * sin(theta) * dt;
        0 1  v * cos(theta) * dt;
        0 0 1
    ];
end
new_sigma = G * sigma * G' + kf.R;
new_mu(3) = atan2(sin(new_mu(3)), cos(new_mu(3)));
new_sigma = (new_sigma + new_sigma') / 2;
end
