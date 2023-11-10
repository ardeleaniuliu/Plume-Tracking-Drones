function robot = get_robot(options)
    % if using this code, please cite: https://doi.org/10.2514/1.C037299
    arguments
        options.MTOW = 2.5; 
        options.payload_fraction = 0.3; % max payload as fraction of MTOW
        options.num_rotors = 4; 
        options.solidity = 0.1; % rotor disk solidity
        options.mach = 0.4; 
        options.k = 1.15; % induced power correction factor (from Leishmann)
        options.efficiency = 0.70; % power train efficiency
        options.g = 9.81;
        options.rho = 1.225;
    end

    MTOW = options.MTOW;
    zero_pay_mass = (1 - options.payload_fraction)*MTOW;
    eta = options.efficiency;
    rho = options.rho;
    k = options.k;
    Mach = options.mach;
    
    bat_cap_fun = @(x)(177*(0.29*x^0.87)^0.98 * 3600); % from Ardelean Unmanned Rotorcraft Scaling paper
    battery_energy = bat_cap_fun(MTOW);
    rotor_radius_fun = @(x)( sqrt( (0.13*x^0.87) / options.num_rotors / pi) ); % from Ardelean Unmanned Rotorcraft Scaling paper
    R = rotor_radius_fun(MTOW);

    MAC = options.solidity*pi*R/2;
    
    Sfp_ref = 0.025; % equivalent flat plate area (from Theys 2020)
    m_ref = 2.5; % mass of drone that corresponds to Sfp_ref (from Theys 2020)
    sfp_fun = @(x)(Sfp_ref*(x/m_ref)^(2/3)); % Assuming Square Cube Law
    Sfp = sfp_fun(MTOW); 

    delta_ref = 0.025; % reference disk profile drag coefficient (from Leishmann)
    Re_ref = 40000; % Re corresponding to reference drag coefficient (from Leishmann)
    OMEGA = Mach*340/R;
    Re = rho*(OMEGA*R)*MAC/(0.0000186);
    if Re<500e3
        delta = delta_ref*(Re/Re_ref)^(-0.4); % from Leishmann
    else
        warning("Blade Reynolds number higher than intended. Is robot.delta value OK?") 
        delta = delta_ref*(Re/Re_ref)^(-0.4); % from Leishmann
    end

    robot.MTOW = MTOW;
    robot.zero_pay_mass = zero_pay_mass;
    robot.battery_energy = battery_energy;
    robot.num_rotors = options.num_rotors;
    robot.rotor_radius = R;
    robot.MAC = MAC;
    robot.Sfp = Sfp;
    robot.delta = delta;
    robot.Mach = Mach;
    robot.k = k;
    robot.efficiency = eta;
    robot.g = options.g;
    robot.rho = options.rho;
    
    power_min = Inf;
    for vel = 1:0.1:20
        power = get_cost(robot, [vel 0 0], 0);
        if power < power_min
            power_min = power;
        end
    end
    robot.min_power = power_min; 
end
