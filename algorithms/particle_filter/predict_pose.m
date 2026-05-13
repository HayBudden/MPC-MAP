function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)
vR = motion_vector(1);
vL = motion_vector(2);
L = read_only_vars.agent_drive.interwheel_dist;
dt = read_only_vars.sampling_period;
speed = (abs(vR) + abs(vL)) / 2;
noise_std = 0.02 + 0.1 * speed;
vR = vR + noise_std * randn;
vL = vL + noise_std * randn;
v = (vR + vL) / 2;
omega = (vR - vL) / L;
x = old_pose(1) + v * cos(old_pose(3)) * dt;
y = old_pose(2) + v * sin(old_pose(3)) * dt;
theta = old_pose(3) + omega * dt;
new_pose = [x, y, theta];
end
