function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
public_vars.kf.C = [1 0 0; 0 1 0];
public_vars.kf.R = diag([0.01, 0.01, 0.005]);
public_vars.kf.Q = diag([0.5, 0.5]);
public_vars.kf.L = read_only_vars.agent_drive.interwheel_dist;
public_vars.mu = [];
public_vars.sigma = [];
end
