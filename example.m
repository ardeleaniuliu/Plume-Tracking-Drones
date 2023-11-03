% Example how to use bits of code
% If using this functionality, consider citing Iuliu Ardelean PhD Thesis

%% Scalable Aerial Robot and Power Model
robot = get_robot("MTOW", 2.5);

% get power in forward flight at 10 m/s
power = get_cost(robot, [10 0 0], 0)


%% Vanilla LBM
occupancy = zeros(150, 100);
occupancy(60:70, 45:55) = 1;

% do max 500 LBM steps 
[UX, UY, UZ, F, DENSITY] = vanilla_LBM(occupancy, "max_num_step", 500); 

% do another max 1000 LBM steps
[UX, UY, UZ, F, DENSITY] = vanilla_LBM(occupancy, "max_num_step", 500, ...
    "F", F, "UX", UX, "UY", UY, "DENSITY", DENSITY); 

%% Get robot power at some point in the domain.
p = [20, 50];
UX_p = UX(p(1), p(2));
UZ_p = UY(p(1), p(2)); % note may need to swap Y and Z if system is left- or right-handed
UY_p = UZ(p(1), p(2));

% if robot is hovering then
power = get_cost(robot, [UX_p UY_p UZ_p], 0)

%% Pollution Modelling

% extrude 2D flow field using repmat
velx = repmat(UX, 1, 1, 50)*100;
vely = repmat(UY, 1, 1, 50)*100;
velz = repmat(UZ, 1, 1, 50)*100;
occu = repmat(occupancy, 1, 1, 50);
occu(end-10,end,:,:) = 2; % set up pollution outlet

domain = get_uc_domain(velx, vely, velz, occu, [20, 50, 25])

% save domain to avoid recomputing
save("computed_domain.mat", "domain")

%% Visualize Pollution using isosurface
load("computed_domain.mat", "domain")
my_transform = make_3d_plot_from_domain(domain);

%% Get A Star path
path = astar_path_planning(domain, robot, [125, 50, 25], [25, 50, 25], 10);
plot_obj = plot3(path(:,2),path(:,1),path(:,3), 'k.-', 'LineWidth',2);
set(plot_obj, "Parent", my_transform)







