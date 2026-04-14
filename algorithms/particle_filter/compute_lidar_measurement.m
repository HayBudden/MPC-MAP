function [measurement] = compute_lidar_measurement(map, pose, lidar_config)

n_beams = length(lidar_config);
measurement = inf(1, n_beams);

walls = map.walls;
origin = pose(1:2);

for i = 1:n_beams
    angle = pose(3) + lidar_config(i);
    dx = cos(angle);
    dy = sin(angle);

    min_dist = inf;

    for w = 1:size(walls, 1)
        p1 = walls(w, 1:2);
        p2 = walls(w, 3:4);

        s = p2 - p1;
        denom = dx * s(2) - dy * s(1);
        if abs(denom) < 1e-10
            continue;
        end

        d = p1 - origin;
        t = (d(1) * s(2) - d(2) * s(1)) / denom;
        u = (d(1) * dy - d(2) * dx) / denom;

        if t > 0 && u >= 0 && u <= 1
            min_dist = min(min_dist, t);
        end
    end

    measurement(i) = min_dist;
end

end
