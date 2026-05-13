function [target] = get_target(estimated_pose, path)
target = [0, 0];
if isempty(path)
    return;
end
pos = estimated_pose(1:2);
lookahead_dist = 0.5;
dists = sqrt(sum((path - pos).^2, 2));
[~, closest_idx] = min(dists);
accumulated_dist = 0;
target_idx = closest_idx;
for i = closest_idx:size(path, 1)-1
    seg_len = sqrt(sum((path(i+1,:) - path(i,:)).^2));
    accumulated_dist = accumulated_dist + seg_len;
    target_idx = i + 1;
    if accumulated_dist >= lookahead_dist
        break;
    end
end
target = path(target_idx, :);
end
