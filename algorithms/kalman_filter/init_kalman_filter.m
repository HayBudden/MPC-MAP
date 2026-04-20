function [public_vars] = init_kalman_filter(read_only_vars, public_vars)

public_vars.kf.C = [1 0 0; 0 1 0];
public_vars.kf.R = diag([0.01, 0.01, 0.01]);
public_vars.kf.L = read_only_vars.agent_drive.interwheel_dist;

if isfield(public_vars, 'gnss_cov') && ~isempty(public_vars.gnss_cov)
    public_vars.kf.Q = public_vars.gnss_cov;
else
    public_vars.kf.Q = diag([0.05, 0.05]);
end

public_vars.mu = [];
public_vars.sigma = [];

if ~isfield(public_vars, 'gnss_mean') || isempty(public_vars.gnss_mean)
    return;
end

if strcmp(public_vars.mode, 'task3')
    public_vars.mu = [2; 2; pi / 2];
    public_vars.sigma = zeros(3);
else
    public_vars.mu = [public_vars.gnss_mean(1); public_vars.gnss_mean(2); 0];
    public_vars.sigma = [public_vars.gnss_cov, [0; 0]; 0 0 4];
end

end

