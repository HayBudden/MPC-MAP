function [public_vars] = student_workspace(read_only_vars, public_vars)

% Initialization
if (read_only_vars.counter == 1)
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);

    public_vars.lidar_data = [];
    public_vars.gnss_data  = [];
    public_vars.analysis_done = false;
end

% Collect sensor data each iteration
public_vars.lidar_data = [public_vars.lidar_data; read_only_vars.lidar_distances];
public_vars.gnss_data  = [public_vars.gnss_data;  read_only_vars.gnss_position];

% After 150 samples, do analysis
if read_only_vars.counter == 150 && ~public_vars.analysis_done
    public_vars.analysis_done = true;

    lidar = public_vars.lidar_data;   % 150x8
    gnss  = public_vars.gnss_data;    % 150x2

    lidar(isinf(lidar)) = NaN;

    % Task 2: standard deviations
    lidar_std = zeros(1, 8);
    for ch = 1:8
        vals = lidar(~isnan(lidar(:, ch)), ch);
        lidar_std(ch) = std(vals);
    end
    gnss_std = std(gnss);

    fprintf('\n--- Task 2: Standard Deviations ---\n');
    for ch = 1:8
        fprintf('LiDAR ch%d: std = %.4f m\n', ch, lidar_std(ch));
    end
    fprintf('GNSS X: std = %.4f m,  GNSS Y: std = %.4f m\n', gnss_std(1), gnss_std(2));

    % Histograms
    figure('Name', 'LiDAR histograms');
    for ch = 1:8
        subplot(2, 4, ch);
        histogram(lidar(:, ch));
        title(sprintf('ch%d (std=%.3f)', ch, lidar_std(ch)));
    end

    figure('Name', 'GNSS histograms');
    subplot(1, 2, 1); histogram(gnss(:, 1));
    title(sprintf('GNSS X (std=%.3f)', gnss_std(1)));
    subplot(1, 2, 2); histogram(gnss(:, 2));
    title(sprintf('GNSS Y (std=%.3f)', gnss_std(2)));

    % Task 3: covariance matrices
    valid = all(~isnan(lidar), 2);
    if sum(valid) > 1
        lidar_cov = cov(lidar(valid, :));
    else
        lidar_cov = diag(lidar_std.^2);
    end
    gnss_cov = cov(gnss);

    fprintf('\nLiDAR covariance (8x8):\n'); disp(lidar_cov);
    fprintf('GNSS covariance (2x2):\n');  disp(gnss_cov);

    % Task 4: normal PDF plot
    valid_ch = find(lidar_std > 0, 1);
    x = linspace(-3*gnss_std(1), 3*gnss_std(1), 500);

    figure('Name', 'Normal PDF');
    plot(x, norm_pdf(x, 0, lidar_std(valid_ch)), 'b', 'LineWidth', 1.5); hold on;
    plot(x, norm_pdf(x, 0, gnss_std(1)), 'r', 'LineWidth', 1.5);
    legend(sprintf('LiDAR ch%d (\\sigma=%.3f)', valid_ch, lidar_std(valid_ch)), ...
           sprintf('GNSS X (\\sigma=%.3f)', gnss_std(1)));
    xlabel('noise [m]'); ylabel('pdf'); title('Normal distribution'); grid on;
end

% Filters and planning
public_vars.particles = update_particle_filter(read_only_vars, public_vars);
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
public_vars.estimated_pose = estimate_pose(public_vars);
public_vars.path = plan_path(read_only_vars, public_vars);
public_vars = plan_motion(read_only_vars, public_vars);

end
