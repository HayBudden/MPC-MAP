function [path] = astar(read_only_vars, public_vars)
path = [];
discrete_map = read_only_vars.discrete_map;
grid = discrete_map.map;
step = read_only_vars.map.discretization_step;
limits = read_only_vars.map.limits;
[rows, cols] = size(grid);
inflated = grid;
for r = 1:rows
    for c = 1:cols
        if grid(r, c) == 1
            for dr = -1:1
                for dc = -1:1
                    nr = r + dr;
                    nc = c + dc;
                    if nr >= 1 && nr <= rows && nc >= 1 && nc <= cols
                        inflated(nr, nc) = 1;
                    end
                end
            end
        end
    end
end
start_pos = public_vars.estimated_pose(1:2);
goal_pos = read_only_vars.map.goal(1:2);
start_col = round((start_pos(1) - limits(1)) / step) + 1;
start_row = round((start_pos(2) - limits(2)) / step) + 1;
goal_col = round((goal_pos(1) - limits(1)) / step) + 1;
goal_row = round((goal_pos(2) - limits(2)) / step) + 1;
start_row = max(1, min(rows, start_row));
start_col = max(1, min(cols, start_col));
goal_row = max(1, min(rows, goal_row));
goal_col = max(1, min(cols, goal_col));
if inflated(start_row, start_col) == 1
    inflated(start_row, start_col) = 0;
end
if inflated(goal_row, goal_col) == 1
    inflated(goal_row, goal_col) = 0;
end
if start_row == goal_row && start_col == goal_col
    path = [goal_pos(1), goal_pos(2)];
    return;
end
g_best = inf(rows, cols);
g_best(start_row, start_col) = 0;
open_list = [start_row, start_col, 0, heuristic(start_row, start_col, goal_row, goal_col), 0, 0];
closed = false(rows, cols);
came_from = zeros(rows, cols, 2);
while ~isempty(open_list)
    [~, idx] = min(open_list(:, 3) + open_list(:, 4));
    current = open_list(idx, :);
    open_list(idx, :) = [];
    cr = current(1);
    cc = current(2);
    g = current(3);
    if closed(cr, cc)
        continue;
    end
    closed(cr, cc) = true;
    came_from(cr, cc, :) = current(5:6);
    if cr == goal_row && cc == goal_col
        path = reconstruct_path(came_from, start_row, start_col, goal_row, goal_col, limits, step);
        return;
    end
    neighbors = [-1 0; 1 0; 0 -1; 0 1; -1 -1; -1 1; 1 -1; 1 1];
    costs = [1; 1; 1; 1; 1.414; 1.414; 1.414; 1.414];
    for i = 1:8
        nr = cr + neighbors(i, 1);
        nc = cc + neighbors(i, 2);
        if nr < 1 || nr > rows || nc < 1 || nc > cols
            continue;
        end
        if closed(nr, nc)
            continue;
        end
        if inflated(nr, nc) == 1
            continue;
        end
        new_g = g + costs(i);
        if new_g < g_best(nr, nc)
            g_best(nr, nc) = new_g;
            h = heuristic(nr, nc, goal_row, goal_col);
            open_list = [open_list; nr, nc, new_g, h, cr, cc];
        end
    end
end
end
function h = heuristic(r1, c1, r2, c2)
    h = sqrt((r1 - r2)^2 + (c1 - c2)^2);
end
function path = reconstruct_path(came_from, sr, sc, gr, gc, limits, step)
    path_cells = [gr, gc];
    cr = gr;
    cc = gc;
    while ~(cr == sr && cc == sc)
        prev = squeeze(came_from(cr, cc, :))';
        cr = prev(1);
        cc = prev(2);
        path_cells = [cr, cc; path_cells];
    end
    path = zeros(size(path_cells, 1), 2);
    for i = 1:size(path_cells, 1)
        path(i, 1) = limits(1) + (path_cells(i, 2) - 1) * step;
        path(i, 2) = limits(2) + (path_cells(i, 1) - 1) * step;
    end
end
