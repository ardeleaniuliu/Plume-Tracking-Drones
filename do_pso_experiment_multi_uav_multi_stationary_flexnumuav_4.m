function [num_src_found, mean_energy_consumed, mean_dist_travelled, num_iter, time_needed, energy_by_source, travel_path_uav_iter] = do_pso_experiment_multi_uav_multi_stationary_flexnumuav_4(domain, robot, algo_options, other, options)
    arguments
        domain
        robot
        algo_options
        other
        options.one_window = 1;
    end
    %% inputs
    thresh_fnd = other.thresh_fnd;
    fig_id = other.figID;
    this_slice = 1;
    ndim = length(size(domain.concentration{this_slice}));
    src_pos = domain.source_locations{this_slice};
    concentration = domain.concentration{this_slice};
    occupancy = domain.occupancy{this_slice};
    battery_energy = robot.battery_energy;
    %     cost_forward = robot.cost_forward;
    %     cost_climb = robot.cost_climb;
    %     cost_descent = robot.cost_descent;
    %     cost_per_hover = robot.cost_hover * robot.time_hover;
    swarm_size = algo_options.swarm_size;
    y1 = algo_options.y1;
    y2 = algo_options.y2;
    inertia = algo_options.inertia;
    num_stationary = algo_options.num_stationary;
    num_uav = algo_options.num_uav;
    apf_radius = algo_options.apf_radius;
    apf_strength = algo_options.apf_strength;
    stationary_height = 9;
    vel_factor = domain.f_re;
    velx = domain.velx*vel_factor;
    vely = domain.vely*vel_factor;
    velz = domain.velz*vel_factor;
    domain.velx = domain.velx*vel_factor;
    domain.vely = domain.vely*vel_factor;
    domain.velz = domain.velz*vel_factor;

    %% data import
    [num_src_tot, ~] = size(src_pos);
    num_src_found = 0;
    [sizex, sizey, sizez] = size(concentration);
    %% swarm init
    swarm = nan(swarm_size, ndim);
    swarm_fit = nan(swarm_size, 1);
    for particle = 1:swarm_size
        swarm(particle, :) = [1,1,1] + [sizex-1, sizey-1, sizez-1].*rand(1,3);
        swarm(1:num_stationary, 2) = stationary_height;
        if algo_options.make_2d
            source_loc = domain.source_locations{1};
            swarm(particle,2) = source_loc(2);
        end
        occupied = occupancy(round(swarm(particle, 1)), round(swarm(particle, 2)), round(swarm(particle, 3)));
        while occupied ~= 0
            swarm(particle, :) = [1,1,1] + [sizex-1, sizey-1, sizez-1].*rand(1,3);
            swarm(1:num_stationary,2) = stationary_height;
            if algo_options.make_2d
                source_loc = domain.source_locations{1};
                swarm(particle,2) = source_loc(2);
            end
            occupied = occupancy(round(swarm(particle, 1)), round(swarm(particle, 2)), round(swarm(particle, 3)));
        end
        swarm_fit(particle) = fit_fun(swarm(particle, :));
    end

    best_pos_each = swarm;
    best_fit_each = swarm_fit;
    [~, index] = max(best_fit_each);
    best_pos_all = swarm(index,:);
    vel = randn(swarm_size, ndim)/10;
    new_swarm_fit = nan(swarm_size, 1);
    %% get init costs
    power_sample = nan(num_uav, 1);
    power_travel = nan(num_uav, 1);
    power_wait = nan(num_uav, 1);
    total_energy_sample = zeros(num_uav, 1);
    total_energy_travel = zeros(num_uav, 1);
    total_energy_wait = zeros(num_uav, 1);
    budget_remaining = ones(num_uav, 1)*battery_energy;
    dist_travelled = zeros(num_uav, 1);
    travel_path_uav = cell(num_uav, 1);
    time_moving = zeros(num_uav, 1);
    power_cons = ones(num_uav, 1)*robot.cost_hover;
    pos_final = nan(num_uav, ndim);
    pos_visited = [];
    pos_visited_uav = cell(1, num_uav);
    flag = 0;
    travel_path_uav_iter = cell(1, num_uav);

    %% plot if fig id
    if fig_id

        make_red_at_the_end = 1;
        one_window = options.one_window;
        make_export_gif = other.make_export_gif;
        gif_name_3d = domain.batch_name+"_10OCT2023.gif";
        gif_name_2d = domain.batch_name+"_timeseries_10OCT2023.gif";
        color_and_alpha = robot.color_and_alpha;
        colors = repmat(color_and_alpha(1:3), [swarm_size 1]);
        % colors = rand(swarm_size,3);
        line_width = 2;
        line_style = 'none';
        marker_style = '.';
        line_alpha = color_and_alpha(4);
        fontsize = 14;
        marker_size = 9;
        cell_size = 1/domain.domain_size;
        norm_traj_val = domain.L_ref/domain.convert_to_meters;

        this_transform = make_3d_plot_from_domain(domain, domain.batch_name, domain.process, "perspective", "iso");

        figure(domain.fig_id_3d_plot)
        traj_plots_3dfig = cell(swarm_size, 1);
        point_plots_3dfig = cell(swarm_size, 1);
        iter_plot_3dfig = cell(swarm_size, 1);
        for particle = 1:swarm_size
            traj_plots_3dfig{particle} = plot3(swarm(particle,2)/norm_traj_val, ...
                swarm(particle,1)/norm_traj_val, swarm(particle,3)/norm_traj_val, ...
                "LineStyle", 'none', "Color", [colors(particle,:) line_alpha], ...
                "LineWidth", line_width, "MarkerSize", 12, "Marker", "none");
            set(traj_plots_3dfig{particle}, "Parent", this_transform)
            point_plots_3dfig{particle} = plot3(swarm(particle,2)/norm_traj_val, ...
                swarm(particle,1)/norm_traj_val, swarm(particle,3)/norm_traj_val, ...
                "LineStyle", "none", "Color", [colors(particle,:) line_alpha], ...
                "LineWidth", line_width, "MarkerSize", 12, "Marker", marker_style);
            set(point_plots_3dfig{particle}, "Parent", this_transform)
            iter_plot_3dfig{particle} = plot3(swarm(particle,2)/norm_traj_val, ...
                swarm(particle,1)/norm_traj_val, swarm(particle,3)/norm_traj_val, ...
                "LineStyle", "none", "Color", [0.2 0.2 0.2 0.8], ...
                "LineWidth", line_width, "MarkerSize", 12, "Marker", marker_style);
            set(iter_plot_3dfig{particle}, "Parent", this_transform)
        end
        drawnow
        ax3d = gca;
        if make_export_gif
            exportgraphics(ax3d, gif_name_3d, "Append", false)
        end

        figure(fig_id)
        %clf
        pose = get(gcf, "Position");
        pose(1) = 50;
        pose(2) = 42;
        pose(3) = 2*560;
        pose(4) = 954;
        set(gcf, "Position", pose);
        ax2d = gcf;

        if one_window
            figure(fig_id)
            subplot(4,2,1)
        else
            figure(fig_id*1000+1)
            pose = get(gcf, "Position");
            pose(1) = 500;
            pose(2) = 200;
            pose(3) = 560;
            pose(4) = 300;
            set(gcf, "Position", pose);
            clf
        end
        hold on
        power_plots = cell(swarm_size, 1);
        for particle = 1:swarm_size
            power_plots{particle} = plot(0, power_cons(particle)/robot.cost_hover, "LineStyle", line_style,...
                "Color", [colors(particle,:) line_alpha], "LineWidth", line_width, ...
                "Marker", marker_style, "MarkerSize", marker_size);
        end
        a = gca;
        a.TickLabelInterpreter = 'latex';
        a.YDir = 'normal';
        xlabel("Iteration Number, $N_{iter}$ (-)", 'Interpreter', 'latex')
        ylabel(["Average Power Required," "$P/P_{hover,ref}$ (-)"],'Interpreter','latex')
        set(gca, 'FontSize', fontsize)
        if get(gca, 'XMinorGrid') == 0
            grid minor
        end
        axis tight
        box on
        ylim([0.5, 1.5])

        if one_window
            figure(fig_id)
            subplot(4,2,2)
        else
            figure(fig_id*1000+2)
            pose = get(gcf, "Position");
            pose(1) = 500;
            pose(2) = 200;
            pose(3) = 560;
            pose(4) = 300;
            set(gcf, "Position", pose);
            clf
        end
        hold on
        budget_plots = cell(swarm_size, 1);
        for particle = 1:swarm_size
            budget_plots{particle} = plot(0, budget_remaining(particle)/battery_energy*battery_energy, "LineStyle", line_style,...
                "Color", [colors(particle,:) line_alpha], "LineWidth", line_width, ...
                "Marker", marker_style, "MarkerSize", marker_size);
        end
        a = gca;
        a.TickLabelInterpreter = 'latex';
        a.YDir = 'normal';
        xlabel("Iteration Number, $N_{iter}$ (-)", 'Interpreter', 'latex')
        ylabel(["Battery Energy Depletion," "$E_{bat}/E_{bat,ref}$ (-)"],'Interpreter','latex')
        set(gca, 'FontSize', fontsize)
        if get(gca, 'XMinorGrid') == 0
            grid minor
        end
        axis tight
        box on
        ylim([0,battery_energy])

        if one_window
            figure(fig_id)
            subplot(4,2,3)
        else
            figure(fig_id*1000+3)
            pose = get(gcf, "Position");
            pose(1) = 500;
            pose(2) = 200;
            pose(3) = 560;
            pose(4) = 300;
            set(gcf, "Position", pose);
            clf
        end
        hold on
        distance_plots = cell(swarm_size, 1);
        for particle = 1:swarm_size
            distance_plots{particle} = plot(0, dist_travelled(particle)/robot.expected_range, "LineStyle", line_style,...
                "Color", [colors(particle,:) line_alpha], "LineWidth", line_width, ...
                "Marker", marker_style, "MarkerSize", marker_size);
        end
        a = gca;
        a.TickLabelInterpreter = 'latex';
        a.YDir = 'normal';
        xlabel("Iteration Number, $N_{iter}$ (-)", 'Interpreter', 'latex')
        ylabel(["Cumulative Travelled" "Distance, $D_{travel}/D_{range}$ (-)"],'Interpreter','latex')
        set(gca, 'FontSize', fontsize)
        if get(gca, 'XMinorGrid') == 0
            grid minor
        end
        axis tight
        box on
        ylim([0,0.5])

        if one_window
            figure(fig_id)
            subplot(4,2,4)
        else
            figure(fig_id*1000+4)
            pose = get(gcf, "Position");
            pose(1) = 500;
            pose(2) = 200;
            pose(3) = 560;
            pose(4) = 300;
            set(gcf, "Position", pose);
            clf
        end
        hold on
        swarm_conc_plot = cell(swarm_size, 1);
        for particle = 1:swarm_size
            swarm_conc_plot{particle} = plot(0, swarm_fit(particle), "LineStyle", line_style,...
                "Color", [colors(particle,:) line_alpha], "LineWidth", line_width, ...
                "Marker", marker_style, "MarkerSize", marker_size);
        end
        a = gca;
        a.TickLabelInterpreter = 'latex';
        a.YDir = 'normal';
        xlabel("Iteration Number, $N_{iter}$ (-)", 'Interpreter', 'latex')
        ylabel(["Sampled Concentration," "$C/C_{ref}$ (-)"],'Interpreter','latex')
        set(gca, 'FontSize', fontsize)
        if get(gca, 'XMinorGrid') == 0
            grid minor
        end
        axis tight
        box on
        ylim([0, 0.5])
        set(gca, 'YScale', 'log')
        set(gca, 'YTick', [1e-4 1e-3 1e-2 1e-1 1e0])

        figure(fig_id)
        subplot(4,2,5)
        hold on
        costtravel_plots = cell(swarm_size, 1);
        for particle = 1:swarm_size
            costtravel_plots{particle} = plot(0, 0, "LineStyle", line_style,...
                "Color", [colors(particle,:) line_alpha], "LineWidth", line_width, ...
                "Marker", marker_style, "MarkerSize", marker_size);
        end
        a = gca;
        a.TickLabelInterpreter = 'latex';
        a.YDir = 'normal';
        xlabel("Iteration Number, $N_{iter}$ (-)", 'Interpreter', 'latex')
        ylabel(["Instantaneous Travel Cost," "$C_{travel}/C_{travel,ref}$ (-)"],'Interpreter','latex')
        set(gca, 'FontSize', fontsize)
        if get(gca, 'XMinorGrid') == 0
            grid minor
        end
        axis tight
        box on
        ylim([0, 1000])
        set(gca, 'YScale', 'log')
        set(gca, 'YTick', [0.1 1 10 100 1000])

        subplot(4,2,6)
        %         speed_of_center = sqrt(sum(sum(vel, 1).^2));
        %         avg_p_speed = mean(sqrt(sum(vel.^2, 2)));
        %         swarm_coh = speed_of_center/avg_p_speed;
        swarm_diam = mean(sqrt(sum((swarm-mean(swarm, 1)).^2, 2)))./domain.domain_size;
        hold on
        swarmdiam_plot = cell(swarm_size, 1);
        for particle = 1:swarm_size
            swarmdiam_plot{particle} = plot(0, swarm_diam, "LineStyle", line_style,...
                "Color", [colors(particle,:) line_alpha], "LineWidth", line_width, ...
                "Marker", marker_style, "MarkerSize", marker_size);
        end
        a = gca;
        a.TickLabelInterpreter = 'latex';
        a.YDir = 'normal';
        xlabel("Iteration Number, $N_{iter}$ (-)", 'Interpreter', 'latex')
        ylabel(["Normalized Swarm Size," "$D_{swarm}/L_{domain}$ (-)"],'Interpreter','latex')
        set(gca, 'FontSize', fontsize)
        if get(gca, 'XMinorGrid') == 0
            grid minor
        end
        axis tight
        box on
        ylim([0,0.5])

        subplot(4,2,7)
        my_map = flipud(hot(100));
        colormap(my_map(1:end,:));
        concentration_slice = concentration(:, :, sizez/2);
        contour((1:sizex)*cell_size, (1:sizey)*cell_size, concentration_slice', LineWidth=line_width)
        hold on
        border_color = [0.1 0.1 0.1];
        occupancy_alpha = occupancy(:,:,sizez/2)' == 1;
        c_data = ones(sizey, sizex, 3);
        c_data(:,:,1) = ones(sizey, sizex)*border_color(1);
        c_data(:,:,2) = ones(sizey, sizex)*border_color(2);
        c_data(:,:,3) = ones(sizey, sizex)*border_color(3);
        imagesc('XData', (1:sizex)*cell_size, 'YData', (1:sizey)*cell_size, 'CData', c_data, 'AlphaData', occupancy_alpha)
        axis image
        a = gca;
        a.TickLabelInterpreter = 'latex';
        a.YDir = 'normal';
        xlabel("Normalized Displacement, $X/L_{domain}$ (-)",'Interpreter','latex')
        ylabel(["Normalized Displacement," "$Y/L_{domain}$ (-)"],'Interpreter','latex')
        %         xlim([0, 140])
        set(gca, 'FontSize', fontsize)
        traj_plots = cell(swarm_size, 1);
        red_plot = cell(swarm_size, 1);
        for particle = 1:swarm_size
            traj_plots{particle} = plot(swarm(particle,1)*cell_size, swarm(particle,2)*cell_size, "LineStyle", line_style, "Color", [colors(particle,:) line_alpha], "LineWidth", line_width, "MarkerSize", 24, "Marker","none");
            red_plot{particle} = plot(swarm(:,1)*cell_size, swarm(:,2)*cell_size, '.', ...
                "MarkerSize", 24, "Color", [colors(particle,:) 0], 'MarkerEdgeColor', 'k');
        end
        subplot(4,2,8)
        hold on
        niter_plots = cell(swarm_size, 1);
        for particle = 1:swarm_size
            niter_plots{particle} = plot(0, 0, "LineStyle", line_style,...
                "Color", [colors(particle,:) line_alpha], "LineWidth", line_width, ...
                "Marker", marker_style, "MarkerSize", marker_size);
        end
        a = gca;
        a.TickLabelInterpreter = 'latex';
        a.YDir = 'normal';
        xlabel("Iteration Number, $N_{iter}$ (-)", 'Interpreter', 'latex')
        ylabel(["Elapsed Time," "$t_{elapsed}/t_{endurance}$ (-)"],'Interpreter','latex')
        set(gca, 'FontSize', fontsize)
        if get(gca, 'XMinorGrid') == 0
            grid minor
        end
        axis tight
        box on
        ylim([0, 1.2])

        %         traj_plots{swarm_size}.Color = [1 0 0 0.5];
        %         budget_plots{swarm_size}.Color = [1 0 0 0.5];
        %         power_plots{swarm_size}.Color = [1 0 0 0.5];
        %         distance_plots{swarm_size}.Color = [1 0 0 0.5];
        %         costtravel_plots{swarm_size}.Color = [1 0 0 0.5];
        %         traj_plots_3dfig{swarm_size}.Color = [1 0 0 0.5];
        drawnow
        if make_export_gif
            exportgraphics(ax2d, gif_name_2d, "Append",false)
        end

        num_bins = 10;
        figure(fig_id*1000)
        pose = get(gcf, "Position");
        pose(1) = 100;
        pose(2) = 100;
        pose(3) = 560*3;
        pose(4) = 300;
        set(gcf, "Position", pose);

        subplot(1,3,1)
        hold on
        plot([robot.cost_hover robot.cost_hover], [0 100], 'k-', "LineWidth", 2)
        histo_sample = histogram(power_sample, "BinWidth", 0.1);
        a = gca;
        a.TickLabelInterpreter = 'latex';
        a.YDir = 'normal';
        xlabel("Average Sample Power (J/s)", 'Interpreter', 'latex')
        ylabel("Frequency, $f$ (-)",'Interpreter','latex')
        set(gca, 'FontSize', fontsize)
        if get(gca, 'XMinorGrid') == 0
            grid minor
        end
        axis tight
        box on
        xlim([0, 3*robot.cost_hover])

        subplot(1,3,2)
        hold on
        % plot([robot.cost_hover robot.cost_hover], [0 100], 'k-', "LineWidth", 2)
        plot([robot.cost_forward*robot.speed_horiz robot.cost_forward*robot.speed_horiz], [0 400], 'k-', "LineWidth", 2)
        child_exists = 0;
        for each_child = 1:length(gca().Children)
            this_child = gca().Children(each_child);
            if this_child.Tag == "myhisto"
                child_exists = 1;
                histo_travel = this_child;
            end
            if this_child.Tag == "histomean"
                child_exists = 1;
                histo_travel_mean = this_child;
            end
        end
        if ~child_exists
            histo_travel = histogram(power_travel, "BinWidth", 0.1, "Tag", "myhisto");
            histomean = mean(histo_travel.Data);
            histo_travel_mean = plot([histomean histomean], [0 400], '-', "LineWidth", 2, "Color", [0.8 0.5 0.2], "Tag", "histomean");
        end
        a = gca;
        a.TickLabelInterpreter = 'latex';
        a.YDir = 'normal';
        xlabel("Average Travel Power, $P_{travel}$ (J/s)", 'Interpreter', 'latex')
        ylabel("Frequency, $f$ (-)",'Interpreter','latex')
        set(gca, 'FontSize', fontsize)
        if get(gca, 'XMinorGrid') == 0
            grid minor
        end
        axis tight
        box on
        %xlim([0, 3*robot.cost_forward*robot.speed_horiz])
        xlim([0 3*robot.cost_forward*robot.speed_horiz])
        ylim([0, 400])

        subplot(1,3,3)
        hold on
        plot([robot.cost_hover robot.cost_hover], [0 100], 'k-', "LineWidth", 2)
        histo_wait = histogram(power_wait, "BinWidth", 0.1);
        a = gca;
        a.TickLabelInterpreter = 'latex';
        a.YDir = 'normal';
        xlabel("Average Wait Power (J/s)", 'Interpreter', 'latex')
        ylabel("Frequency, $f$ (-)",'Interpreter','latex')
        set(gca, 'FontSize', fontsize)
        if get(gca, 'XMinorGrid') == 0
            grid minor
        end
        axis tight
        box on
        xlim([0, 3*robot.cost_hover])

    end

    %% init ctd
    for uav = 1:num_uav
        this_uav_swarm = swarm(num_stationary+uav:num_uav:end, :);
        [budget_remaining_this, pos_final_this, pos_visited_this, ~, dist_travelled_this, time_moving_this, this_travel_path, this_energy_sample, this_energy_travel] = get_greedy_costs_from_points(this_uav_swarm, budget_remaining(uav), dist_travelled(uav), time_moving(uav));
        total_energy_sample(uav) = total_energy_sample(uav) + this_energy_sample;
        total_energy_travel(uav) = total_energy_travel(uav) + this_energy_travel;
        budget_remaining(uav) = budget_remaining_this;
        dist_travelled(uav) = dist_travelled_this;
        time_moving(uav) = time_moving_this;
        pos_final(uav,:) = pos_final_this;
        pos_visited = [pos_visited; pos_visited_this]; %#ok<AGROW>
        pos_visited_uav{uav} = pos_visited_this;
        travel_path_uav{uav} = this_travel_path;
    end

    for uav = 1:num_uav
        travel_path_uav_iter(1, uav) = travel_path_uav(uav);
    end

    %% check if found
    for uav = 1:num_uav
        pv_uav = pos_visited_uav{uav};
        dist = ((pv_uav(:,1) - src_pos(1)).^2 + ...
            (pv_uav(:,2) - src_pos(2)).^2 + ...
            (pv_uav(:,3) - src_pos(3)).^2).^0.5;
        if any(dist(end) < thresh_fnd) && (budget_remaining(uav) > 0) %%&& (swarm_fit(uav)>1e-3)
            num_src_found = 1;
        end
    end

    if all(budget_remaining < 0)
        flag = 123;
    end

    %% run iter
    num_iter = 0;
    while flag ~= 123
        old_swarm = swarm;
        num_iter = num_iter + 1;
        if num_iter >= 1e6
            warning("iter 1e6")
            break;
        end
        if all(abs(pos_visited - pos_visited(1,:)) <= 0.5, 'all')
            warning('converged')
            break;
        end
        if num_src_found == num_src_tot
            break;
        end
        if robot.time_hover == 0
            if sum(budget_remaining > 0) == 1
                break;
            end
        end

        %% get vel
        delta_1 = best_pos_each - swarm;
        delta_2 = -[swarm(:,1) - best_pos_all(1), ...
            swarm(:,2) - best_pos_all(2), ...
            swarm(:,3) - best_pos_all(3)];
        vel = inertia*vel + y1*rand(swarm_size, ndim).*delta_1 + ...
            y2*rand(swarm_size, ndim).*delta_2;

        %% get new swarm
        swarm = swarm + vel;

        %% artificial potential field
        if apf_strength ~= 0
            apf_offset = zeros(swarm_size, ndim);
            for particle = 1:swarm_size
                for pp = 1:swarm_size
                    if pp == particle
                        continue
                    end
                    apf_dist = sqrt(sum((swarm(particle,:) - swarm(pp,:)).^2));
                    if apf_dist >= apf_radius
                        continue
                    end
                    apf_dir = swarm(particle,:) - swarm(pp,:);
                    this_apf_offset = apf_strength*(1/apf_dist - 1/apf_radius)^2;
                    apf_offset(particle, :) = apf_offset(particle, :) + apf_dir*this_apf_offset;
                end

            end
            swarm = swarm + apf_offset;
        end

        %% swarm lock
        swarm((swarm(:,1) > sizex-1),1) = sizex-1;
        swarm((swarm(:,2) > sizey-1),2) = sizey-1;
        swarm((swarm(:,3) > sizez-1),3) = sizez-1;
        swarm((swarm(:,1) < 2),1) = 2;
        swarm((swarm(:,2) < 2),2) = 2;
        swarm((swarm(:,3) < 2),3) = 2;
        if algo_options.make_2d
            source_loc = domain.source_locations{1};
            swarm(particle, 2) = source_loc(2);
        end
        swarm(1:num_stationary,:) = old_swarm(1:num_stationary,:);
        swarm(budget_remaining <= 0, :) = old_swarm(budget_remaining <= 0,:); % only works for Nuav=Nmobile and Nstationary = 0;

        %% adjust for obstacles and stationary and get new fit
        for particle = 1:swarm_size
            occupied = occupancy(round(swarm(particle, 1)), round(swarm(particle, 2)), round(swarm(particle, 3)));
            
            if occupied == 1
                % swarm(particle, 2) = 1*sizey/4;
                swarm(particle, 2) = 62;
                %                 swarm(particle, :) = old_swarm(particle, :);
            end
            
            if algo_options.make_2d
                source_loc = domain.source_locations{1};
                swarm(particle,2) = source_loc(2);
                while occupied
                    swarm(particle,1) = swarm(particle,1) - 1;
                    occupied = occupancy(round(swarm(particle, 1)), round(swarm(particle, 2)), round(swarm(particle, 3)));
                end
            end

            new_swarm_fit(particle) = fit_fun(swarm(particle, :));
        end

        %% check  improvement
        bfeo = best_fit_each;
        %         bfeo = swarm_fit;

        best_fit_each(new_swarm_fit > bfeo) = new_swarm_fit(new_swarm_fit > bfeo);
        best_pos_each(new_swarm_fit > bfeo, :) = swarm(new_swarm_fit > bfeo, :);
        [~, index] = max(best_fit_each);
        best_pos_all = best_pos_each(index,:);
        swarm_fit = new_swarm_fit;

        %% get budget left
        budget_old = budget_remaining;
        time_old = time_moving;
        dist_travelled_old = dist_travelled;
        pos_visited = [];
        pos_visited_uav = cell(1, num_uav);
        travel_path_uav = cell(num_uav, 1);
        for uav = 1:num_uav
            this_uav_swarm = [pos_final(uav,:); swarm(num_stationary+uav:num_uav:end, :)];
            if budget_remaining(uav) > 0
                [budget_remaining_this, pos_final_this, pos_visited_this,...
                    ~, dist_travelled_this, time_moving_this,...
                    this_travel_path, this_energy_sample, this_energy_travel] = get_greedy_costs_from_points(this_uav_swarm,....
                    budget_remaining(uav), dist_travelled(uav), time_moving(uav));
            else
                budget_remaining_this = budget_remaining(uav);
                pos_final_this = pos_final(uav,:);
                pos_visited_this = pos_final(uav,:);
                % flag_this = 0;
                dist_travelled_this = dist_travelled(uav);
                time_moving_this = time_moving(uav);
                this_travel_path = [pos_final(uav,:) pos_final(uav,:)];
                this_energy_sample = 0;
                this_energy_travel = 0;
            end
            total_energy_sample(uav) = total_energy_sample(uav) + this_energy_sample;
            total_energy_travel(uav) = total_energy_travel(uav) + this_energy_travel;
            power_sample(uav) = this_energy_sample/robot.time_hover;
            power_travel(uav) = this_energy_travel/(time_moving_this - time_old(uav) - robot.time_hover);
            budget_remaining(uav) = budget_remaining_this;
            dist_travelled(uav) = dist_travelled_this;
            time_moving(uav) = time_moving_this;
            pos_final(uav,:) = pos_final_this;
            pos_visited = [pos_visited; pos_visited_this]; %#ok<AGROW>
            pos_visited_uav{uav} = pos_visited_this;
            travel_path_uav{uav} = this_travel_path;
            if isnan(budget_remaining_this)
                error('ss')
            end
            %             if power_travel(uav)>200
            %                 warning("power_travel(uav)>200")
            %                 disp(this_travel_path)
            %             end
        end

        for uav = 1:num_uav
            travel_path_uav_iter{num_iter+1, uav} = travel_path_uav{uav};
        end
        %% plot
        if fig_id
            for particle = 1:swarm_size
                trajuav = travel_path_uav{particle};
                waypoints = pos_visited_uav{particle};
                traj_plots{particle}.XData = [traj_plots{particle}.XData, trajuav(:,1)'*cell_size];
                traj_plots{particle}.YData = [traj_plots{particle}.YData, trajuav(:,2)'*cell_size];
                traj_plots_3dfig{particle}.XData = [traj_plots_3dfig{particle}.XData trajuav(:,2)'./norm_traj_val];
                traj_plots_3dfig{particle}.YData = [traj_plots_3dfig{particle}.YData trajuav(:,1)'./norm_traj_val];
                traj_plots_3dfig{particle}.ZData = [traj_plots_3dfig{particle}.ZData trajuav(:,3)'./norm_traj_val];
                point_plots_3dfig{particle}.XData = [point_plots_3dfig{particle}.XData waypoints(end,2)'./norm_traj_val];
                point_plots_3dfig{particle}.YData = [point_plots_3dfig{particle}.YData waypoints(end,1)'./norm_traj_val];
                point_plots_3dfig{particle}.ZData = [point_plots_3dfig{particle}.ZData waypoints(end,3)'./norm_traj_val];
                iter_plot_3dfig{particle}.XData = waypoints(end,2)./norm_traj_val;
                iter_plot_3dfig{particle}.YData = waypoints(end,1)./norm_traj_val;
                iter_plot_3dfig{particle}.ZData = waypoints(end,3)./norm_traj_val;
                distance_plots{particle}.XData = [distance_plots{particle}.XData, num_iter];
                distance_plots{particle}.YData = [distance_plots{particle}.YData, dist_travelled(particle)/robot.expected_range];
                %                 costtravel_plots{particle}.XData = [costtravel_plots{particle}.XData, dist_travelled(particle)/(domain.domain_size*domain.convert_to_meters)];
                %                 costtravel_plots{particle}.YData = [costtravel_plots{particle}.YData, budget_remaining(particle)/battery_energy];
                %                 budget_plots{particle}.XData = [budget_plots{particle}.XData, num_iter];
                %                 budget_plots{particle}.YData = [budget_plots{particle}.YData, budget_remaining(particle)/battery_energy];
                red_plot{particle}.XData = swarm(particle,1)*cell_size;
                red_plot{particle}.YData = swarm(particle,2)*cell_size;
                %                 niter_plots{particle}.XData = [niter_plots{particle}.XData, num_iter];
                %                 niter_plots{particle}.YData = [niter_plots{particle}.YData, time_moving(particle)];
            end
        end

        %% loiter
        t_before_wait = time_moving;
        budget_before_wait = budget_remaining;
        [budget_remaining, time_moving, ~] = costs_due_to_loiter(budget_remaining, time_moving, pos_final);
        if all(budget_remaining<0)
            flag = 123;
            % warning("all(budget_remaining<0)")
        end
        total_energy_wait = total_energy_wait + (budget_before_wait - budget_remaining);
        power_wait = (budget_before_wait - budget_remaining)./(time_moving - t_before_wait);

        % disp(table(power_sample, power_travel, power_wait))

        %% check if found
        for uav = 1:num_uav
            pv_uav = pos_visited_uav{uav};
            dist = ((pv_uav(:,1) - src_pos(1)).^2 + ...
                (pv_uav(:,2) - src_pos(2)).^2 + ...
                (pv_uav(:,3) - src_pos(3)).^2).^0.5;
            if any(dist(end) < thresh_fnd) && (budget_remaining(uav) > 0) %%&& (swarm_fit(uav) > 1e-3)
                num_src_found = 1;
            end
            if num_src_found && fig_id && make_red_at_the_end && 1
                for i = 1:num_uav
                    traj_plots_3dfig{i}.LineStyle = 'none';
                end
                traj_plots{uav}.Color = [1 0 0 0.5];
                traj_plots{uav}.LineStyle = '-';
                budget_plots{uav}.Color = [1 0 0 0.5];
                budget_plots{uav}.LineStyle = '-';
                power_plots{uav}.Color = [1 0 0 0.5];
                power_plots{uav}.LineStyle = '-';
                distance_plots{uav}.Color = [1 0 0 0.5];
                distance_plots{uav}.LineStyle = '-';
                costtravel_plots{uav}.Color = [1 0 0 0.5];
                costtravel_plots{uav}.LineStyle = '-';
                swarm_conc_plot{uav}.Color = [1 0 0 0.5];
                swarm_conc_plot{uav}.LineStyle = '-';
                point_plots_3dfig{uav}.Color = [1 0 0 0.5];
                point_plots_3dfig{uav}.Marker = ".";
                traj_plots_3dfig{uav}.Color = [1 0 0 0.5];
                traj_plots_3dfig{uav}.LineStyle = '-';

                uistack(traj_plots_3dfig{uav},"top")
                uistack(power_plots{uav},"top")
                uistack(budget_plots{uav},"top")
                uistack(distance_plots{uav},"top")
                uistack(swarm_conc_plot{uav},"top")

                %                 figure(domain.fig_id_3d_plot)
                %                 blue_traj_plots_3dfig = plot3(nan, nan, nan, ...
                %                     "LineStyle", '-', "Color", [0.2 0.2 0.8 line_alpha], ...
                %                     "LineWidth", line_width, "MarkerSize", 12, "Marker", "none");
                %                 set(blue_traj_plots_3dfig, "Parent", this_transform)
                %                 num_iter_blue = 114;
                %                 set(blue_traj_plots_3dfig, "XData", traj_plots_3dfig{uav}.XData(1:num_iter_blue))
                %                 set(blue_traj_plots_3dfig, "YData", traj_plots_3dfig{uav}.YData(1:num_iter_blue))
                %                 set(blue_traj_plots_3dfig, "ZData", traj_plots_3dfig{uav}.ZData(1:num_iter_blue))
                %
                %                 figure(domain.fig_id_3d_plot)
                %                 blue_points_plots_3dfig = plot3(nan, nan, nan, ...
                %                     "LineStyle", "none", "Color", [0.2 0.2 0.8 line_alpha], ...
                %                     "LineWidth", line_width, "MarkerSize", 12, "Marker", ".");
                %                 set(blue_points_plots_3dfig, "Parent", this_transform)
                %                 num_iter_blue = 24;
                %                 set(blue_points_plots_3dfig, "XData", point_plots_3dfig{uav}.XData(1:num_iter_blue))
                %                 set(blue_points_plots_3dfig, "YData", point_plots_3dfig{uav}.YData(1:num_iter_blue))
                %                 set(blue_points_plots_3dfig, "ZData", point_plots_3dfig{uav}.ZData(1:num_iter_blue))

                break
            end
        end

        %% plot
        if fig_id
            histo_sample.Data = [histo_sample.Data power_sample];
            histo_travel.Data = [histo_travel.Data power_travel];
            histo_travel.Data(histo_travel.Data==0) = nan;
            histo_wait.Data = [histo_wait.Data power_wait];
            histo_sample.NumBins = num_bins;
            histo_travel.NumBins = num_bins;
            histo_wait.NumBins = num_bins;
            histomean = mean(histo_travel.Data, "all", "omitnan");
            histo_travel_mean.XData = [histomean histomean];
            %             speed_of_center = sqrt(sum(sum(vel, 1).^2));
            %             avg_p_speed = mean(sqrt(sum(vel.^2, 2)));
            %             swarm_coh = speed_of_center/avg_p_speed;
            %             swarm_diam = max(sqrt(sum((swarm-mean(swarm, 1)).^2, 2))).*domain.convert_to_meters;
            swarm_diam = mean(sqrt(sum((swarm-mean(swarm, 1)).^2, 2)))./domain.domain_size;
            
            swarm_diam = min(sqrt(sum((swarm-src_pos).^2, 2)))./domain.domain_size;
            
            %             swarm_diam2 = max(sqrt(sum((swarm-best_pos_all).^2, 2))).*domain.convert_to_meters;
            power_cons = (budget_old-budget_remaining)./(time_moving-time_old)./robot.cost_hover;
            travel_cost = (budget_old-budget_remaining)./(dist_travelled-dist_travelled_old)./robot.cost_forward;
            for particle = 1:swarm_size
                distance_plots{particle}.XData = [distance_plots{particle}.XData, num_iter];
                distance_plots{particle}.YData = [distance_plots{particle}.YData, dist_travelled(particle)/robot.expected_range];
                costtravel_plots{particle}.XData = [costtravel_plots{particle}.XData, num_iter];
                costtravel_plots{particle}.YData = [costtravel_plots{particle}.YData, travel_cost(particle)];
                budget_plots{particle}.XData = [budget_plots{particle}.XData, num_iter];
                budget_plots{particle}.YData = [budget_plots{particle}.YData, budget_remaining(particle)/battery_energy*battery_energy];
                power_plots{particle}.XData = [power_plots{particle}.XData, num_iter];
                power_plots{particle}.YData = [power_plots{particle}.YData, power_cons(particle)];
                swarm_conc_plot{particle}.XData = [swarm_conc_plot{particle}.XData, num_iter];
                swarm_conc_plot{particle}.YData = [swarm_conc_plot{particle}.YData, swarm_fit(particle)];
            end
            niter_plots{particle}.XData = [niter_plots{particle}.XData, num_iter];
            niter_plots{particle}.YData = [niter_plots{particle}.YData, time_moving(particle)./robot.expected_endurance];
            swarmdiam_plot{particle}.XData = [swarmdiam_plot{particle}.XData, num_iter];
            swarmdiam_plot{particle}.YData = [swarmdiam_plot{particle}.YData, swarm_diam];
            drawnow limitrate
            if make_export_gif
                for i = 1:3
                    exportgraphics(ax3d, gif_name_3d, "Append", true)
                    exportgraphics(ax2d, gif_name_2d, "Append", true)
                end
            end
        end

    end % END OF EXPERIMENT


    %% get final metrics
    mean_energy_consumed =  mean(abs(budget_remaining - battery_energy));
    mean_dist_travelled =  mean(dist_travelled);
    time_needed = mean(time_moving);
    energy_by_source.sample = mean(total_energy_sample);
    energy_by_source.travel = mean(total_energy_travel);
    energy_by_source.wait = mean(total_energy_wait);

    if any((budget_remaining + total_energy_sample + total_energy_travel + total_energy_wait) - battery_energy > 1)
        quant = (budget_remaining + total_energy_sample + total_energy_travel + total_energy_wait) - battery_energy;
        quant = quant(quant > 1);
        quant = max(quant);
        % warning("energy not matching up by source: " + quant + "J")
    end

    if fig_id
        for particle = 1:swarm_size
            %             red_plot{particle}.XData = nan(swarm_size, 1);
            %             red_plot{particle}.YData = nan(swarm_size, 1);
            %             niter_plots{particle}.XData = niter_plots{particle}.XData/time_moving(particle);
            %             distance_plots{particle}.XData = distance_plots{particle}.XData/time_moving(particle);
            %             budget_plots{particle}.XData = budget_plots{particle}.XData/time_moving(particle);
            %             power_plots{particle}.XData = power_plots{particle}.XData/time_moving(particle);
        end
        drawnow
        if make_export_gif
            for i = 1:15
                exportgraphics(ax2d, gif_name_2d, "Append", true)
                exportgraphics(ax3d, gif_name_3d, "Append", true)
            end
        end

        ok = 0;
        figure(fig_id*6000+1)
        position = get(gcf,"Position");
        position(3) = 1120;
        position(4) = 420;
        position(1) = 100;
        position(2) = 100;
        set(gcf, "Position",position)

        children = get(gca, "Children");
        for child = children
            if child.Tag == "histo"
                my_histo_map = child;
                ok = 1;
            end
        end

%         plots_for_histo = traj_plots_3dfig;
        plots_for_histo = point_plots_3dfig;
        points_vis = nan(1, 3);
        for particle = 1:swarm_size
            num_points_visited = length(plots_for_histo{particle}.XData);
            size_points_vis = size(points_vis,1);
            points_vis(size_points_vis+1:size_points_vis+num_points_visited, 1) = plots_for_histo{particle}.XData;
            points_vis(size_points_vis+1:size_points_vis+num_points_visited, 2) = plots_for_histo{particle}.YData;
            points_vis(size_points_vis+1:size_points_vis+num_points_visited, 3) = plots_for_histo{particle}.ZData;
        end
        %         x_histo = reshape(points_vis(:,:,2), [num_points_visited*swarm_size, 1])*norm_traj_val;
        %         y_histo = reshape(points_vis(:,:,3), [num_points_visited*swarm_size, 1])*norm_traj_val;
        x_histo = points_vis(:,2);
        y_histo = points_vis(:,1);

        if ok == 0
            colormap(flipud(hot(10)))
            histogram2(x_histo,y_histo,"Tag","histo", "FaceColor","flat", "NumBins", [28, 5])
            hcb = colorbar("eastoutside");
            hcb.TickLabelInterpreter = 'latex';
            hcb.FontSize = fontsize;
            hcb.Ticks = 0:50:200;
            ylabel(hcb, "Visitation Frequency (-)",'Interpreter','latex', 'FontSize', fontsize)
            view(2)
            axis equal
            xlim([0, sizex/norm_traj_val])
            ylim([0, sizey/norm_traj_val])
            xticks(0:1:sizex/norm_traj_val)
            yticks(0:1:sizey/norm_traj_val)
            caxis([0, 200])
            xlabel("$X/L_{ref}$ (-)",'Interpreter','latex')
            ylabel("$Y/L_{ref}$ (-)",'Interpreter','latex')
            set(gca, "ticklabelinterpreter", 'latex')
            set(gca, "fontsize", fontsize)
        end

        if ok == 1
            new_histo_data = [x_histo, y_histo];
            my_histo_map.Data = [my_histo_map.Data; new_histo_data];
        end
    
    end


    %% util
    function [budget_remaining, point_final, points_visited, flag, dist_end, time_end, travel_path, energy_sample, energy_travel] = get_greedy_costs_from_points(points, budget_start, dist_start, time_start)
        flag = 0;
        [num_points,~] = size(points);
        if num_points == 1
            pos_id = round(points(1, :));
            linear_idx = sub2ind(size(occupancy), pos_id(:,1), pos_id(:,2), pos_id(:,3));
            vx_line = velx(linear_idx);
            vy_line = vely(linear_idx);
            vz_line = velz(linear_idx);
            vel_g = [0 0 0];
            vel_a = vel_g - [vx_line vy_line vz_line];
            power_this = get_cost(robot, vel_a, vel_g(2));
            cost_hover = robot.time_hover*power_this;
            cost_travel = 0;
            budget_remaining = budget_start - cost_travel - cost_hover;
            dist_end = dist_start;
            time_end = time_start + robot.time_hover;
            point_final = points(num_points, :);
            points_visited = points;
            travel_path = [point_final; point_final];
            energy_sample = cost_hover;
            energy_travel = cost_travel;
            return;
        end
        [cost_travel, time_travel, dist_travel, travel_path] = travel_cost_fun(points(1, :), points(2, :));
        pos_id = round(points(2, :));
        linear_idx = sub2ind(size(occupancy), pos_id(:,1), pos_id(:,2), pos_id(:,3));
        vx_line = velx(linear_idx);
        vy_line = vely(linear_idx);
        vz_line = velz(linear_idx);
        vel_g = [0 0 0];
        vel_a = vel_g - [vx_line vy_line vz_line];
        power_this = get_cost(robot, vel_a, vel_g(2));
        cost_hover = robot.time_hover*power_this;
        budget_remaining = budget_start - cost_travel - cost_hover;
        dist_end = dist_start + dist_travel;
        time_end = time_start + time_travel + robot.time_hover;
        point_final = points(num_points, :);
        points_visited = points;
        if cost_travel + cost_hover > budget_start
            flag = 1;
            point_final = points(1, :);
            points_visited = points(1, :);
        end
        energy_sample = cost_hover;
        energy_travel = cost_travel;
    end

    %% util fun 1
    function [budget_remaining, time_moving, flag_this] = costs_due_to_loiter(budget_remaining, time_moving, pos_uav)
        flag_this = 0;
        max_time_moving = max(time_moving);
        pos_id = round(pos_uav);
        linear_idx = sub2ind(size(occupancy), pos_id(:,1), pos_id(:,2), pos_id(:,3));
        vx_line = velx(linear_idx);
        vy_line = vely(linear_idx);
        vz_line = velz(linear_idx);
        cost_to_loiter = nan(num_uav, 1);
        for this_uav = 1:num_uav
            vel_g = [0 0 0];
            vel_a = vel_g - [vx_line(this_uav) vy_line(this_uav) vz_line(this_uav)];
            power_this = get_cost(robot, vel_a, vel_g(2));
            cost_to_loiter(this_uav) = abs(time_moving(this_uav) - max_time_moving)*power_this;
        end

        time_moving(:) = max(time_moving);
        budget_remaining(budget_remaining>0) = budget_remaining(budget_remaining>0) - cost_to_loiter(budget_remaining>0);
        if any(budget_remaining < 0)
            flag_this = 1;
        end

    end

    function [energy_taken, time_taken, distance_taken, travel_line] = travel_cost_fun(start, finish)
        if all(round(start) == round(finish))
            energy_taken = 0;
            time_taken = 0;
            distance_taken = 0;
            travel_line = [round(start); round(finish)];
            return;
        end
        speed_g_h = robot.speed_horiz; % m/s
        speed_g_v = robot.speed_vertical; % m/s
        travel_line = generate_trajectory_astar(domain, robot, round(start), round(finish), occupancy, "path_planner", algo_options.path_planner);
        linear_idx = sub2ind(size(occupancy), travel_line(:,1), travel_line(:,2), travel_line(:,3));
        vx_line = velx(linear_idx);
        vy_line = vely(linear_idx);
        vz_line = velz(linear_idx);
        [num_points, ~] = size(travel_line);
        energy_taken = 0;
        time_taken = 0;
        distance_taken = 0;
        for p = 1:num_points-1
            trav_dist = sqrt(sum((travel_line(p+1,:) - travel_line(p,:)).^2));
            dist_vec = travel_line(p+1,:) - travel_line(p,:);
            if ~trav_dist
                continue
            end
            if dist_vec(2)
                vel_g = (travel_line(p+1,:) - travel_line(p,:)) / trav_dist * speed_g_v; % m/s
            else
                vel_g = (travel_line(p+1,:) - travel_line(p,:)) / trav_dist * speed_g_h; % m/s
            end
            speed_g = sqrt(sum(vel_g.^2));
            trav_dist = trav_dist*domain.convert_to_meters;
            vel_a = vel_g - [vx_line(p) vy_line(p) vz_line(p)];
            vel_a = double(vel_a);
            power = get_cost(robot, vel_a, vel_g(2));
            this_energy = power/speed_g*trav_dist;
            energy_taken = energy_taken + this_energy;
            time_taken = time_taken + trav_dist/speed_g;
            distance_taken = distance_taken + trav_dist;
            if isnan(energy_taken)
                error('ss')
            end
        end

    end

    %% util fun 2
    function c = fit_fun(x)
        c = concentration(round(x(:,1)), round(x(:,2)), round(x(:,3)));
        if c < robot.LOD
            c = robot.LOD;
        end

        if robot.sensor_noise
            c = c * (1 + randn*robot.sensor_noise);
        end
    end

end
