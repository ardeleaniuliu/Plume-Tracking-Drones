% Example how to use bits of code

% Scalable Aerial Robot and Power Model
robot = get_robot("MTOW", 2.5);

% Power in forward flight at 10 m/s
power = get_cost(robot, [10 0 0], 0)

% Vanilla LBM
occupancy = zeros(100, 100);
occupancy(30:40, 45:55) = 1;

% do max 500 LBM steps 
[UX, UY, UZ, F, DENSITY] = vanilla_LBM(occupancy, "max_num_step", 500); 

% do another max 1000 LBM steps
[UX, UY, UZ, F, DENSITY] = vanilla_LBM(occupancy, "max_num_step", 500, ...
    "F", F, "UX", UX, "UY", UY, "DENSITY", DENSITY); 

% Get robot power at some point in the domain.
p = [20, 50];
UX_p = UX(p(1), p(2));
UZ_p = UY(p(1), p(2)); % note need to swap Y and Z because one system is left-handed and the other right-handed
UY_p = UZ(p(1), p(2));

% if robot is hovering then
power = get_cost(robot, [UX_p UY_p UZ_p], 0)
