function P_tot = get_cost(robot, V_inf, V_c)
    % please cite Iuliu Ardelean PhD Thesis
    arguments
        robot % robot struct (output from get_robot function)
        V_inf % true air speed [Vx, Vy, Vz]
        V_c % rate of climb wrt Earth reference frame
    end

    Mach = robot.Mach; 
    eta = robot.efficiency; 
    m = robot.zero_pay_mass; % can also use robot.MTOW
    R = robot.rotor_radius;
    MAC = robot.MAC;
    Sfp = robot.Sfp;

    s = 2*MAC/pi/R; 
    OMEGA = Mach*340/R; 
    A = robot.num_rotors*pi*R^2; % rotor area numProps*pi*R^2
    W = m*robot.g; % aircraft weight in Newtons

    rho = robot.rho; 
    delta = robot.delta;
    k = robot.k;

    P0 = delta/8*rho*s*A*OMEGA^3*R^3; % profile
    vh = sqrt(W/(2*rho*A));

    V_inf_mag = sqrt(V_inf(1)^2 + V_inf(2)^2 + V_inf(3)^2);
    D = 0.5*rho*V_inf_mag.^2*Sfp;
    gamma = asind(V_inf(2)/V_inf_mag);
    if isnan(gamma)
        gamma = 0;
    end
    T = sqrt(W^2 + D^2 + 2*D*W*sind(gamma));
    beta = acotd((D*sind(gamma)+W)/(D*cosd(gamma)));
    alpha = beta + gamma;
    mu = abs(V_inf(2))/OMEGA/R;
    vi_fun = @(vi)(T - 2*rho*A*vi*sqrt(V_inf_mag^2 + 2*V_inf_mag*vi*sind(alpha) + vi^2));

    my_vi = newton_raphson_2(vi_fun, vh, 10000, 0.1);
    if isnan(my_vi)
        warning('isnan my vi')
        my_vi = vh;
    end

    P_tot = (P0*(1+3*mu.^2) + k*T*(my_vi + 1*V_inf_mag*sind(alpha)) + 0.5*Sfp*rho*V_inf_mag.^3 + V_c*W)/eta;

    if isnan(P_tot)
        error('isnan P u inf')
    end

    %     inflow_ratio = my_vi/vh;
    %     power_coeff = P_tot/(1/2*rho*A*(OMEGA*R)^3);
    %     thrust_coeff = T/(1/2*rho*A*(OMEGA*R)^2);

end

function x = newton_raphson_2(fun, x0, max_iter, alpha)
    tolX = 1;
    iter = 0;
    delta = 1e-6;
    x = x0;
    df =  @(x) (fun(x+delta)-fun(x-delta))/(2*delta);
    while tolX >= 1e-6
        iter = iter + 1;
        if iter > max_iter
            break
        end
        J = fun(x)./df(x);
        x_new = x - alpha*J;
        tolX = abs(x_new - x);
        x = x_new;
    end
end
