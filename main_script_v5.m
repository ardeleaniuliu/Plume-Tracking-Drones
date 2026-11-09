% Analysis Parameters
clear
tic
rng('shuffle')

num_combinations1 = 8;
num_combinations2 = 8;

tsens = logspace(-1, 2, num_combinations1);
msens1 = linspace(0, 0.1, num_combinations2);
msens2 = linspace(0, 1, num_combinations2);
msens3 = linspace(0, 10, num_combinations2);
msens4 = linspace(0, 40, num_combinations2);

num_stationary = 0;
num_mobile = 10;
num_uav = 10;

batch_code = 400;
processes = (5) + batch_code;

for counter = 1:4
    msens = eval("msens"+counter);

    [ttsens, mmsens] = meshgrid(tsens, msens);

    results = zeros(1, 9);
    for i = 1:(num_combinations1*num_combinations2)
    
        this_msr = 0;
        for this_process = processes
            % domain size in domain_units, not in meters
            domain = get_uc_domain("loadFromFile", 1, "filePath",...
                "uc_pollution/urban_canyon_04_"+this_process+".mat", 'figID', 0);
            algo_options = get_algo("num_uav", num_uav, "num_mobile", num_mobile, ...
                'num_stationary', num_stationary, 'y1', 2, 'y2', 2, 'inertia', 0.1, ...
                'apf_radius', 10, 'apf_strength', 10);
    
            drone_colors=lines(7);
    
            robot1 = get_robot('mass', 0.1 + mmsens(i), 'use_pi_everywhere', 1,...
                'rotor_radius', 3*2.54/2/100, 'battery_energy', 15e3, ...
                'mean_aero_chord', 0.01, 'equiv_flat_plate_area', 0.0125, 'num_rotors', 4,...
                'color_and_alpha', [drone_colors(1,:), 0.1], 'overwrite_solidity', 0.1, 'robot_tag', "Micro");
            
            robot2 = get_robot('mass', 1 + mmsens(i), 'use_pi_everywhere', 1,...
                'rotor_radius', 9.4*2.54/2/100, 'battery_energy', 77*3600,  ...
                'mean_aero_chord', 0.018, 'equiv_flat_plate_area', 0.0250, 'num_rotors', 4,...
                'color_and_alpha', [drone_colors(2,:), 0.1], 'overwrite_solidity', 0.1, 'robot_tag', "Small");
            
            robot3 = get_robot('mass', 10 + mmsens(i), 'use_pi_everywhere', 1, ...
                'rotor_radius', 21*2.54/2/100, 'battery_energy', 6*100*3600, ...
                'mean_aero_chord', 0.080, 'equiv_flat_plate_area', 0.05, 'num_rotors', 6,...
                'color_and_alpha', [drone_colors(3,:), 0.1], 'overwrite_solidity', 0.1, 'robot_tag', "Medium");
            
            robot4 = get_robot('mass', 40 + mmsens(i), 'use_pi_everywhere', 1, ...
                'rotor_radius', 38*2.54/2/100, 'battery_energy', 5429e3, ...
                'mean_aero_chord', 0.080, 'equiv_flat_plate_area', 0.1, 'num_rotors', 6,...
                'color_and_alpha', [drone_colors(4,:), 0.1], 'overwrite_solidity', 0.1, 'robot_tag', "Large");
            
            robot = eval("robot"+counter);
            robot.time_hover = ttsens(i);
    %         robot = get_robot('mass', 0.1 + mmsens(i), 'use_pi_everywhere', 1, 'rotor_radius', 0.035, 'battery_energy', 15e3, 'response_time', ttsens(i));
            % robot = get_robot('mass', 2.5 + mmsens(i), 'use_pi_everywhere', 1, 'battery_energy', 356e3, 'response_time', ttsens(i));
    %          robot = get_robot('mass', 40 + mmsens(i), 'use_pi_everywhere', 1, 'rotor_radius', 38*2.54/2/100, 'battery_energy', 5429e3, 'response_time', ttsens(i));
    
            % Experiments are performed in domain-units so we need to
            % convert cost_per_meter to cost_per_domain_unit;
            %         robot.cost_forward = robot.cost_forward * domain.convert_to_meters;
            %         robot.cost_climb = robot.cost_climb * domain.convert_to_meters;
            %         robot.cost_descent = robot.cost_descent * domain.convert_to_meters;
    
            % do experiment
            domain.convert_to_meters = 1/3;
            experiments = do_n_experiments(domain, robot, algo_options, ...
                'use_pso_poly_stat_flexnumuav', 1, 'num_experiments', 100,...
                'figID', 0, 'thresh_fnd', 9);
    
            this_msr = this_msr + experiments.mean_num_src_fnd;
            
        end
        toc
    
        % results are stored in meters
        results(i, :) = [domain.domain_size*domain.convert_to_meters,...
            algo_options.step_size*domain.convert_to_meters,...
            robot.battery_energy,...
            robot.cost_forward/domain.convert_to_meters,...
            robot.cost_hover,...
            robot.time_hover,...
            this_msr/numel(processes),...
            robot.cost_climb/domain.convert_to_meters,...
            robot.cost_descent/domain.convert_to_meters];
        
    end
    
    L_domain = results(:, 1);
    L_step = results(:, 2);
    E_battery = results(:, 3);
    C_forward = results(:, 4);
    C_hover = results(:, 5);
    t_response = results(:, 6);
    mean_num_src_fnd = results(:, 7);
    C_climb = results(:, 8);
    C_descent = results(:, 9);
    
    pi_terms = [mean_num_src_fnd, C_forward.*L_domain./E_battery, C_hover.*t_response./E_battery];
    
    fs = 12;
    figure(12+counter)
    clf
    colormap(flipud(bone(100)))
    hold on
    
    x = pi_terms(1:num_combinations2:end, 3);
    y = pi_terms(1:num_combinations2, 2);
    [X,Y] = meshgrid(x,y);
    
    Z = reshape(pi_terms(:,1), num_combinations2, num_combinations1);
    [C,h] = contourf(tsens, msens, Z, [0 0.1 0.3 0.5 0.7 0.9 0.95 0.99], 'LineColor','none');
    % imagesc(tsens, msens, Z)
    [C,h] = contour(tsens, msens, Z, [0 0.1 0.3 0.5 0.7 0.9 0.95 0.99], "ShowText",'off', 'LineColor', [0.9 0 0], 'LineWidth', 1.5);
    clabel(C, h, h.LevelList, 'color', 'r', 'interpreter', 'latex', 'fontsize', fs)
    % set(gca, 'XTick', tsens)
    % set(gca, 'YTick', msens)
    set(gca, 'XScale', 'log')
%     set(gca, 'YScale', 'log')
    set(gca, 'TickLabelInterpreter', 'latex')
    set(gca, 'FontSize', fs)
    
    xlabel("Sensor Response Time, $t_s$ (s)", Interpreter="latex", FontSize=fs)
    ylabel("Sensor Mass, $m_s$ (kg)", Interpreter="latex", FontSize=fs)
    c = colorbar("TickLabelInterpreter","latex");
    c.Label.String = "Success Rate, $\Pi_1$ (-)";
    c.Label.Interpreter = 'latex';
    c.Label.FontSize = fs;
    caxis([0 1])
    % grid on
    set(gca, 'Layer', 'top')
    box on

end