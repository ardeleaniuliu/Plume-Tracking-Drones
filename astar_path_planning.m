function [path, total_cost, iter] = astar_path_planning(domain, robot, start, goal, drone_speed_set, options)
    % If using this, consider citing Iuliu Ardelean PhD Thesis
    arguments
        domain
        robot
        start
        goal
        drone_speed_set
        options.use_aero_model = 1;
        options.d3q26 = 1;
        options.d3q6 = 0;
        options.heuristic = 3;
        options.export_gif = 0;
        options.filename = 'astar_exploration_20DEC2022.gif'
        options.space_step = 5;
        options.speed_step = 0;
        options.enableKE = 0;
        options.make_XY = 0; % i.e. in-plane gravity
        options.make_XZ = 0; % i.e. out-of-plane gravity
    end
    velx = domain.velx;
    vely = domain.vely;
    velz = domain.velz;
    occupancy = domain.occupancy;

    speed_start = drone_speed_set(1);
    speed_start_ind = 1;

    sizex = size(occupancy, 1);
    sizey = size(occupancy, 2);
    sizez = size(occupancy, 3);
    sizes = size(drone_speed_set, 2);

    if start(1) < 1 || start(2) < 1 || start(3) < 1 || start(1) > sizex || start(2) > sizey || start(3) > sizez
        error('start out of bounds')
    end
    if goal(1) < 1 || goal(2) < 1 || goal(3) < 1 || goal(1) > sizex || goal(2) > sizey || goal(3) > sizez
        error('start out of bounds')
    end

    priority = Inf(sizex, sizey, sizez, sizes);
    if options.heuristic == 2
        priority(start(1), start(2), start(3), speed_start_ind) = heuristic_cost_fun_2(robot, goal, start, velx, vely, velz, speed_start);
    elseif options.heuristic == 1
        priority(start(1), start(2), start(3), speed_start_ind) = heuristic_cost_fun(occupancy, goal, start);
    elseif options.heuristic == 3
        priority(start(1), start(2), start(3), speed_start_ind) = heuristic_cost_fun_3(robot, goal, start, drone_speed_set);
    end
    reached = zeros(sizex, sizey, sizez, sizes);
    reached(start(1), start(2), start(3), speed_start_ind) = 0;

    came_from_x = zeros(sizex, sizey, sizez, sizes);
    came_from_x(start(1), start(2), start(3), speed_start_ind) = nan;
    came_from_y = zeros(sizex, sizey, sizez, sizes);
    came_from_y(start(1), start(2), start(3), speed_start_ind) = nan;
    came_from_z = zeros(sizex, sizey, sizez, sizes);
    came_from_z(start(1), start(2), start(3), speed_start_ind) = nan;
    came_from_s = zeros(sizex, sizey, sizez, sizes);
    came_from_s(start(1), start(2), start(3), speed_start_ind) = nan;

    cost_so_far = zeros(sizex, sizey, sizez, sizes);
    cost_so_far(start(1), start(2), start(3), speed_start_ind) = 0;

    if options.d3q26
        to_neighbours = [1 1 1; 1 1 0; 1 1 -1;
            1 0 1; 1 0 0; 1 0 -1;
            1 -1 1; 1 -1 0; 1 -1 -1;
            0 1 1; 0 1 0; 0 1 -1;
            0 0 1; 0 0 -1;
            0 -1 1; 0 -1 0; 0 -1 -1;
            -1 1 1; -1 1 0; -1 1 -1;
            -1 0 1; -1 0 0; -1 0 -1;
            -1 -1 1; -1 -1 0; -1 -1 -1;
            ];

        if options.make_XY
            to_neighbours(to_neighbours(:,3)~=0,:) = [];
        end
        if options.make_XZ
            to_neighbours(to_neighbours(:,2)~=0,:) = [];
        end
    end
    if options.d3q6
        to_neighbours = [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1;];
    end
    to_neighbours(:, 4) = 1;
    to_neighbours2 = to_neighbours;
    to_neighbours2(:, 4) = 0;
    to_neighbours = [to_neighbours; to_neighbours2];
    to_neighbours2(:, 4) = -1;
    to_neighbours = [to_neighbours; to_neighbours2];
    to_neighbours(:, 1:3) = to_neighbours(:, 1:3) * options.space_step;
    to_neighbours(:, 4) = to_neighbours(:, 4) * options.speed_step;
    num_neighbours = size(to_neighbours, 1);

    iter = 0;
    while true
        iter = iter+1;
        
        [~, ind] = min(priority, [], "all");
        [x1, x2, x3, speed_now_ind] = ind2sub(size(priority), ind);
        pos_now = [x1 x2 x3];
        speed_now = drone_speed_set(speed_now_ind);
        priority(pos_now(1), pos_now(2), pos_now(3), speed_now_ind) = Inf;

        if all(pos_now == goal)
            %disp('eh')
            break
        end
        dist = abs(pos_now-goal);
        if all(dist <= options.space_step)
            break
        end

        for this_neighbour = 1:num_neighbours
            pos_next = pos_now + to_neighbours(this_neighbour, 1:3);
            speed_next_ind = speed_now_ind + to_neighbours(this_neighbour, 4);

            if (speed_next_ind < 1 || speed_next_ind > sizes) && numel(drone_speed_set) ~= 1
                continue
            end
            speed_next = drone_speed_set(speed_next_ind);

            if pos_next(1) < 1 || pos_next(2) < 1 || pos_next(3) < 1 || pos_next(1) > sizex || pos_next(2) > sizey || pos_next(3) > sizez
                continue
            end
            if numel(drone_speed_set) == 1 && speed_next_ind ~= speed_start_ind
                continue
            end
            if occupancy(pos_next(1), pos_next(2), pos_next(3)) == 1
                continue
            end

            if options.use_aero_model
                actual_cost = actual_cost_fun_11MAR2023(robot, pos_next, pos_now, velx, vely, velz, speed_now);
                actual_cost = actual_cost + options.enableKE*1/2*2.5*(speed_next^2-speed_now^2);
            else
                if options.heuristic == 2
                    actual_cost = heuristic_cost_fun_2(robot, pos_next, pos_now, velx, vely, velz, speed_now);
                elseif options.heuristic == 1
                    actual_cost = heuristic_cost_fun(occupancy, pos_next, pos_now);
                elseif options.heuristic == 3
                    actual_cost = heuristic_cost_fun_3(robot, pos_next, pos_now, drone_speed_set);
                end
            end

            new_cost = cost_so_far(pos_now(1), pos_now(2), pos_now(3), speed_now_ind) + actual_cost;

            if ~reached(pos_next(1), pos_next(2), pos_next(3), speed_next_ind) || new_cost < cost_so_far(pos_next(1), pos_next(2), pos_next(3), speed_next_ind)
                reached(pos_next(1), pos_next(2), pos_next(3), speed_next_ind) = 1;
                cost_so_far(pos_next(1), pos_next(2), pos_next(3), speed_next_ind) = new_cost;

                if options.heuristic == 2
                    h_cost = heuristic_cost_fun_2(robot, goal, pos_next, velx, vely, velz, speed_next);
                elseif options.heuristic == 1
                    h_cost = heuristic_cost_fun(occupancy, goal, pos_next);
                elseif options.heuristic == 3
                    h_cost = heuristic_cost_fun_3(robot, goal, pos_next, drone_speed_set);
                end
                priority(pos_next(1), pos_next(2), pos_next(3), speed_next_ind) = new_cost + h_cost;

                came_from_x(pos_next(1), pos_next(2), pos_next(3), speed_next_ind) = pos_now(1);
                came_from_y(pos_next(1), pos_next(2), pos_next(3), speed_next_ind) = pos_now(2);
                came_from_z(pos_next(1), pos_next(2), pos_next(3), speed_next_ind) = pos_now(3);
                came_from_s(pos_next(1), pos_next(2), pos_next(3), speed_next_ind) = speed_now_ind;
            end
        end
    end

    ok = 0;
    state_now = [pos_now, speed_now_ind];
    if ok == 0
        total_cost = cost_so_far(state_now(1), state_now(2), state_now(3), state_now(4));
        ok = 1; %#ok<*NASGU>
    end
    path = [];
    while ~(all(state_now(1:3) == start) && state_now(4) == speed_start_ind)
        %disp(now)
        path = [path; state_now]; %#ok<AGROW>
        dummy = nan(1,4);
        dummy(1) = came_from_x(state_now(1), state_now(2), state_now(3), state_now(4));
        dummy(2) = came_from_y(state_now(1), state_now(2), state_now(3), state_now(4));
        dummy(3) = came_from_z(state_now(1), state_now(2), state_now(3), state_now(4));
        dummy(4) = came_from_s(state_now(1), state_now(2), state_now(3), state_now(4));
        state_now = dummy;
    end
    path = [path; start, speed_start_ind];

    for i = 1:size(path,1)
        v_ind = path(i, 4);
        path(i,4) = drone_speed_set(v_ind);
    end

end

function min_cost = actual_cost_fun_11MAR2023(robot, pos_next, pos_now, velx, vely, velz, speed)
    dist = sqrt(sum((pos_next - pos_now).^2));
    dist_vec = pos_next - pos_now;
    if dist_vec(2)
        vel_g = (pos_next - pos_now) / dist * min(speed, 5); % m/s
    else
        vel_g = (pos_next - pos_now) / dist * speed; % m/s
    end
    this_speed_g = sqrt(sum(vel_g.^2));
    wx = velx(pos_now(1), pos_now(2), pos_now(3));
    wy = vely(pos_now(1), pos_now(2), pos_now(3));
    wz = velz(pos_now(1), pos_now(2), pos_now(3));
    vel_a = vel_g - [wx wy wz];
    vel_a = double(vel_a);
    power = get_cost(robot, vel_a, vel_g(2));

    min_cost = power/this_speed_g*dist;

end


function cost = heuristic_cost_fun(~, pos_next, pos_now)
    cost = sqrt(sum((pos_next-pos_now).^2));
end

function cost = heuristic_cost_fun_2(robot, goal, pos_now, velx, vely, velz, speed_g)
    dist = sqrt(sum((goal - pos_now).^2));
    if dist==0
        cost = 0;
        return;
    end
    vel_g = (goal - pos_now)/dist * speed_g;
    this_speed_g = sqrt(sum(vel_g.^2));
    wx = velx(pos_now(1), pos_now(2), pos_now(3));
    wy = vely(pos_now(1), pos_now(2), pos_now(3));
    wz = velz(pos_now(1), pos_now(2), pos_now(3));
    vel_a = vel_g - [wx wy wz];
    vel_a = double(vel_a);
    power = get_cost(robot, vel_a, vel_g(2));

    cost = power/this_speed_g*dist;

end

function cost = heuristic_cost_fun_3(robot, goal, pos_now, drone_speed_set)
    dist = sqrt(sum((goal - pos_now).^2));
    if dist==0
        cost = 0;
        return;
    end
    power = robot.min_power;

    cost = power/max(drone_speed_set)*dist;

end
