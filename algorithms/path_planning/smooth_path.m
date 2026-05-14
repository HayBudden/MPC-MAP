function [new_path] = smooth_path(old_path)
new_path = old_path;
if isempty(old_path) || size(old_path, 1) < 3
    return;
end
    weight_smooth = 0.05;
    weight_data = 0.9;
    new_path = old_path;
    for iter = 1:50
    change = 0;
    for i = 2:size(new_path, 1) - 1
        for j = 1:2
            old_val = new_path(i, j);
            new_path(i, j) = new_path(i, j) + weight_data * (old_path(i, j) - new_path(i, j));
            new_path(i, j) = new_path(i, j) + weight_smooth * (new_path(i-1, j) + new_path(i+1, j) - 2 * new_path(i, j));
            change = change + abs(old_val - new_path(i, j));
        end
    end
    if change < 1e-4
        break;
    end
end
end
