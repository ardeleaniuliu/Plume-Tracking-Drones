function experiments = do_n_experiments_corrections(domain, robot, algo_options, options)
    arguments
        domain
        robot
        algo_options
        options.use_pso_mono (1,1) {mustBeNumeric} = 0
        options.use_pso_poly (1,1) {mustBeNumeric} = 0
        options.use_pso_poly_stat (1,1) {mustBeNumeric} = 0
        options.use_pso_poly_stat_flexnumuav (1,1) {mustBeNumeric} = 0
        options.use_straight (1,1) {mustBeNumeric} = 0
        options.figID (1,1) {mustBeNumeric} = 0
        options.num_experiments (1,1) {mustBeNumeric} = 100
        options.make_pso_2d_horiz (1,1) {mustBeNumeric} = 0
        options.make_pso_2d_vert (1,1) {mustBeNumeric} = 0
        options.make_pso_1d (1,1) {mustBeNumeric} = 0
        options.thresh_fnd (1,1) {mustBeNumeric} = 6
        options.make_export_gif = 0;
    end

    other.thresh_fnd = options.thresh_fnd;
    other.figID = options.figID;
    other.make_export_gif = options.make_export_gif;

    num_experiments = options.num_experiments;
    energy_sample = nan(num_experiments, 1);
    energy_travel = nan(num_experiments, 1);
    energy_wait = nan(num_experiments, 1);
    num_src_fnd_batch = nan(num_experiments, 1);
    mean_energy_cnsmd_batch = nan(num_experiments, 1);
    mean_dist_trav_batch = nan(num_experiments, 1);
    num_iter_batch = nan(num_experiments, 1);
    time_needed_batch = nan(num_experiments, 1);

    for experiment = 1:num_experiments
        %rng(100 + experiment)
        disp(experiment+" out of "+num_experiments)
        [nsf, mec, mdt, ni, time, ebs, uavtp] = do_pso_experiment_corrections(domain, robot, algo_options, other);
        mean_energy_cnsmd_batch(experiment) = mec;
        mean_dist_trav_batch(experiment) = mdt;
        num_iter_batch(experiment) = ni;
        time_needed_batch(experiment) = time;
        num_src_fnd_batch(experiment) = nsf;
        energy_sample(experiment) = ebs.sample;
        energy_travel(experiment) = ebs.travel;
        energy_wait(experiment) = ebs.wait;
    end

    experiments.mean_energy_consumed = mean_energy_cnsmd_batch;
    experiments.mean_dist_travelled = mean_dist_trav_batch;
    experiments.mean_num_iter = num_iter_batch;
    experiments.time_needed = time_needed_batch;
    experiments.src_fnd = num_src_fnd_batch;
    experiments.uavtp = uavtp;

    experiments.mean_num_src_fnd = mean(num_src_fnd_batch);
    experiments.std_num_src_fnd = std(num_src_fnd_batch);
    experiments.num_experiment = num_experiments;

    experiments.energy_sample = energy_sample;
    experiments.energy_travel = energy_travel;
    experiments.energy_wait = energy_wait;

end
