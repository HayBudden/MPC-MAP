function [measurement] = compute_lidar_measurement(map, pose, lidar_config)
n_beams = length(lidar_config);
measurement = inf(1, n_beams);
walls = map.walls;
n_walls = size(walls, 1);
if n_walls == 0
    return;
end
ox = pose(1);
oy = pose(2);
p1x = walls(:, 1);
p1y = walls(:, 2);
p2x = walls(:, 3);
p2y = walls(:, 4);
sx = p2x - p1x;
sy = p2y - p1y;
d_x = p1x - ox;
d_y = p1y - oy;
for i = 1:n_beams
    angle = pose(3) + lidar_config(i);
    dx = cos(angle);
    dy = sin(angle);
    denom = dx * sy - dy * sx;
    t = (d_x .* sy - d_y .* sx) ./ denom;
    u = (d_x * dy - d_y * dx) ./ denom;
    valid = abs(denom) > 1e-10 & t > 0 & u >= 0 & u <= 1;
    if any(valid)
        measurement(i) = min(t(valid));
    end
end
end
