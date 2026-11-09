% Analysis Parameters
clear

rng(1)

num_planets = 2;
opt = ["Terra", "Mars"];
g = [9.81 3.71];
rho = [1.225 0.02];

color_options = lines(num_planets);
color_options(:,4) = 0.5;

for jj = 1:num_planets

    this_opt = opt(jj);

    pi3 = 0.05;
    pi2 = 0.05;

    num_uav = 10;
    num_stationary = 0;

    num_exp = 8;
    mean_num_src_fnd_v = nan(num_exp, 1);
    plume_vol_v = nan(num_exp, 1);

    mass = logspace(-1, 2, num_exp);
    k = logspace(log10(1.75), log10(1.15), num_exp);
    eta = logspace(log10(0.6), log10(0.6), num_exp);
    Sfp = logspace(log10(0.05), log10(0.05), num_exp);
    mach = logspace(log10(0.4), log10(0.4), num_exp);
    delta = logspace(log10(0.03), log10(0.03), num_exp);
    solidity = logspace(log10(0.33), log10(0.1), num_exp);
    pay_fraction = logspace(log10(0.3), log10(0.3), num_exp);

    for i = 1:num_exp

        tic
        this_process = 401;
        % domain size in domain_units, not in meters
        domain = get_uc_domain("loadFromFile", 1, "filePath",...
            "uc_pollution/urban_canyon_04_"+this_process+".mat", 'figID', 0);

        algo_options = get_algo("num_uav", num_uav, "num_mobile", num_uav, ...
            'num_stationary', num_stationary, 'y1', 2, 'y2', 2, 'inertia', 0.1, ...
            'apf_radius', 10, 'apf_strength', 10);

        robot = get_robot('MTOW', mass(i), 'num_rotors', 4, ...
            'overwrite_solidity', solidity(i), 'mach', mach(i), 'k', k(i), ...
            'efficiency', eta(i), 'delta', delta(i), 'Sfp', Sfp(i), ...
            'scale_Sfp', 1, 'scale_cd0', 1, 'payload_fraction', pay_fraction(i), ...
            'g', g(jj), 'rho', rho(jj));

        robot.time_hover = pi3*robot.battery_energy/robot.cost_hover;
        domain.convert_to_meters = pi2*robot.battery_energy/robot.cost_forward./domain.domain_size;

        % do experiment
        experiments = do_n_experiments(domain, robot, algo_options, ...
            'use_pso_poly_stat_flexnumuav', 1, 'num_experiments', 100, ...
            'figID', 0, 'thresh_fnd', 9);

        mean_num_src_fnd_v(i) = experiments.mean_num_src_fnd;
        toc
    end
%%
    fontsize = 12;
    figure(191323)
    % clf
    hold on
    plot(mass, mean_num_src_fnd_v, '.-', LineWidth=1.5, Color=color_options(jj,1:3), ...
        MarkerSize=18, DisplayName=this_opt)
    set(gca, 'TickLabelInterpreter','latex')
    set(gca, 'XScale', 'log')
    xlabel('MTOW (kg)', 'Interpreter','latex', 'FontSize',fontsize)
    ylabel('Success Rate, $\Pi_1$', 'Interpreter','latex', 'FontSize',fontsize)
    set(gca,'XMinorGrid','on');
    set(gca,'YMinorGrid','on');
    legend('Location', 'best', 'Interpreter', 'latex',      'FontSize', fontsize)
    set(gca, 'FontSize', fontsize)
    ylim([0, 1])
    box on
    drawnow
end