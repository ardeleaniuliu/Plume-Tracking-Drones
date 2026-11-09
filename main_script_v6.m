% Analysis Parameters
clear

rng(1)

pi3 = 0.005;
pi2 = 0.005;

num_uav = 10;
num_stationary = 0;

num_MTOW = 4;
mean_num_src_fnd_v = nan(num_MTOW, 1);
plume_vol_v = nan(num_MTOW, 1);

MTOW = logspace(-1, 2, num_MTOW);
k = logspace(log10(1.75), log10(1.15), num_MTOW);
eta = logspace(log10(0.6), log10(0.6), num_MTOW);
Sfp = logspace(log10(0.05), log10(0.05), num_MTOW);
mach = logspace(log10(0.4), log10(0.4), num_MTOW);
delta = logspace(log10(0.03), log10(0.03), num_MTOW);
solidity = logspace(log10(0.33), log10(0.1), num_MTOW);
pay_fraction = logspace(log10(0.3), log10(0.3), num_MTOW);

for i = 1:num_MTOW
    tic
    rng(1)
    this_process = 1;
    % domain size in domain_units, not in meters
    domain = get_uc_domain("loadFromFile", 1, "filePath",...
        "uc_pollution/single_cube_"+this_process+".mat", 'figID', 99, ...
        'f_re', 1, 'flow_file_name', 'uc_flow/single_cube_2')

    algo_options = get_algo("num_uav", num_uav, "num_mobile", num_uav, ...
        'num_stationary', num_stationary, 'y1', 2, 'y2', 2, 'inertia', 0.1, ...
        'apf_radius', 10, 'apf_strength', 10);

    robot = get_robot('MTOW', MTOW(i), 'num_rotors', 4, ...
        'overwrite_solidity', solidity(i), 'mach', mach(i), 'k', k(i), ...
        'efficiency', eta(i), 'delta', delta(i), 'Sfp', Sfp(i), ...
        'scale_Sfp', 1, 'scale_cd0', 1, 'payload_fraction', pay_fraction(i))

    robot.time_hover = pi3*robot.battery_energy/robot.cost_hover;
    domain.convert_to_meters = pi2*robot.battery_energy/robot.cost_forward./domain.domain_size;
    domain.f_re = 1/3.5/domain.convert_to_meters;

    % do experiment
    experiments = do_n_experiments(domain, robot, algo_options, ...
        'use_pso_poly_stat_flexnumuav', 1, 'num_experiments', 1, ...
        'figID', i, 'thresh_fnd', 9);

    mean_num_src_fnd_v(i) = experiments.mean_num_src_fnd;
    toc
end

fontsize = 12;
figure(191325)
% clf
hold on
%     plot(MTOW, mean_num_src_fnd_v, '.-', LineWidth=1.5, Color=color_options(jj,1:3), ...
%         MarkerSize=18, DisplayName="$\Pi_2=$"+pi3+", $\Pi_3=$"+pi2)
this_plot = plot(MTOW, mean_num_src_fnd_v, '.-', LineWidth=1.5, ...
    MarkerSize=18, DisplayName="$\Pi_{hov}=$"+pi3+", $\Pi_{trav}=$"+pi2+", $f_{vel}=$"+domain.f_re);
std = sqrt(mean_num_src_fnd_v.*(1-mean_num_src_fnd_v)/experiments.num_experiment);
plot(MTOW, mean_num_src_fnd_v+std, '--', LineWidth=1, Color = this_plot.Color, ...
    MarkerSize=18, HandleVisibility='off')
plot(MTOW, mean_num_src_fnd_v-std, '--', LineWidth=1, Color = this_plot.Color, ...
    MarkerSize=18, HandleVisibility='off')

set(gca, 'TickLabelInterpreter','latex')
set(gca, 'XScale', 'log')
xlabel('MTOW (kg)', 'Interpreter','latex', 'FontSize',fontsize)
ylabel('Success Rate, $\Pi_1$', 'Interpreter','latex', 'FontSize',fontsize)
set(gca,'XMinorGrid','on');
set(gca,'YMinorGrid','on');
legend('Location', 'best', 'Interpreter', 'latex', 'FontSize', fontsize)
set(gca, 'FontSize', fontsize)
ylim([0, 1])
box on
drawnow

