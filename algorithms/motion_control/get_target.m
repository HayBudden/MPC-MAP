function [target] = get_target(estimated_pose, path)

target = [0, 0];

if isempty(path)
    return;
end

pos = estimated_pose(1:2);
lookahead = 0.8;

dists = sqrt(sum((path - pos).^2, 2));
[~, closest_idx] = min(dists);

target_idx = closest_idx;
for i = closest_idx:size(path, 1)
    if sqrt(sum((path(i, :) - pos).^2)) >= lookahead
        target_idx = i;
        break;
    end
    target_idx = i;
end

target = path(target_idx, :);

end

