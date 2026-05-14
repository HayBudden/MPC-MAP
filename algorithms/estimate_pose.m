function [estimated_pose] = estimate_pose(public_vars)
estimated_pose = nan(1, 3);
has_kf = ~isempty(public_vars.mu) && ~any(isnan(public_vars.mu));
has_pf = ~isempty(public_vars.particles) && size(public_vars.particles, 1) > 0;
if has_pf
    particles = public_vars.particles;
    N = size(particles, 1);
    radius = 0.5;
    best_count = 0;
    best_idx = 1;
    n_check = min(N, 50);
    step_size = max(1, floor(N / n_check));
    for i = 1:n_check
        idx = 1 + (i-1) * step_size;
        dx = particles(:,1) - particles(idx,1);
        dy = particles(:,2) - particles(idx,2);
        nearby = (dx.^2 + dy.^2) < radius^2;
        count = sum(nearby);
        if count > best_count
            best_count = count;
            best_idx = idx;
        end
    end
    dx = particles(:,1) - particles(best_idx,1);
    dy = particles(:,2) - particles(best_idx,2);
    nearby = (dx.^2 + dy.^2) < radius^2;
    min_cluster = max(5, round(0.1 * N));
    if sum(nearby) >= min_cluster
        cluster = particles(nearby, :);
    else
        med_x = median(particles(:, 1));
        med_y = median(particles(:, 2));
        dists_from_med = (particles(:,1) - med_x).^2 + (particles(:,2) - med_y).^2;
        n_keep = round(0.9 * N);
        [~, sorted_idx] = sort(dists_from_med);
        keep_idx = sorted_idx(1:n_keep);
        cluster = particles(keep_idx, :);
    end
    pf_x = mean(cluster(:, 1));
    pf_y = mean(cluster(:, 2));
    pf_theta = atan2(mean(sin(cluster(:, 3))), mean(cos(cluster(:, 3))));
    estimated_pose(1) = pf_x;
    estimated_pose(2) = pf_y;
    estimated_pose(3) = pf_theta;
elseif has_kf
    estimated_pose = public_vars.mu(:)';
end
if ~any(isnan(estimated_pose))
    estimated_pose(3) = atan2(sin(estimated_pose(3)), cos(estimated_pose(3)));
end
end
