% Example how to use bits of code

% Scalable Aerial Robot and Power Model
robot = get_robot("MTOW", 2.5);
power = get_cost(robot, [10 0 0], 0);

% Vanilla LBM
occupancy = zeros(100, 100);
occupancy(30:40, 45:55) = 1;
[UX, UY, UZ, F, DENSITY] = vanilla_LBM(occupancy, "max_num_step", 1000); % do 1000 steps

[UX, UY, UZ, F, DENSITY] = vanilla_LBM(occupancy, "max_num_step", 1000, ...
    "F", F, "UX", UX, "UY", UY, "DENSITY", DENSITY); % do another 1000 steps
