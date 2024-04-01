
batch_name = "funky_geometries_03";
flow_file_name = "uc_flow/"+batch_name+".mat";

occupancy = zeros(50, 50, 1);

occupancy(22:28, 22:28, :) = 1;

[vx, vy, vz, F, DENSITY] = vanilla_LBM(occupancy(:,:,1), "omega", 1.800);

% [vx, vy, vz, F, DENSITY] = vanilla_LBM(occupancy(:,:,1), "omega", 1.0);
%%
velx = repmat(vx, [1 1 250])*3000;
vely = repmat(vy, [1 1 250])*3000;
velz = repmat(vz, [1 1 250])*3000;
%%
occupancy(end-10:end,:,:) = 2;
APPRES = 1;
% save(flow_file_name, "APPRES", "occupancy", "velx", "vely", "velz")

%%
batch_name = "infotaxis_cube";
flow_file_name = "uc_flow/"+batch_name+".mat";

load(flow_file_name, "APPRES", "occupancy", "velx", "vely", "velz")

[sizex, sizey, sizez] = size(occupancy);
xlocs = 1:1:sizex;
ylocs = 1:1:sizey;
zlocs = sizez/2;

% internal_model = nan(sizex, sizey, 1, sizex, sizey, 1);

for xloc = xlocs
    for yloc = ylocs
        tic
        domain = get_uc_domain("process", 0, "figID", 1, "batch_name", batch_name, ...
            "flow_file_name", flow_file_name, ...
            "sourceLocation", [xloc yloc zlocs], "simTime", 10, ...
            "APPRES", 1, "noise_mag", 0.1*APPRES, "save_file", 0, "r0", 1);
        toc

        concentration = domain.concentration{1};

        internal_model(xloc, yloc, 1, :, :, 1) = concentration(:,:,sizez/2);

    end
end

%%
load(flow_file_name, "APPRES", "occupancy", "velx", "vely", "velz")

%%
load("canyon_exp_data_u.mat", 'exp_data_u')
load("canyon_exp_data_w.mat", 'exp_data_w')

figure(567)
clf
hold on
plot(exp_data_u(:,1), exp_data_u(:,2), 'o', color='k', displayname='ADV (Chew et al.)', LineWidth=2)

grid on
xlabel('Horizontal Velocity Component, $U_x$ (m/s)', 'Interpreter','latex')
ylabel('Distance along Canyon Centreline, $y/H$ (-)', 'Interpreter','latex')
legend('Location', "best", 'Interpreter','latex')
set(gca, 'TickLabelInterpreter','latex')
set(gca, 'FontSize', 10)
ylim([0, 2.5])
xlim([-0.4, 1.2])

[~, sizey, sizez] = size(velx);
y_uc = round(sizey/5)*2.5;
z_uc = round(sizez/2);
x_val = linspace(0, 5, sizey);
x_uc = 675;

ref = velx(x_uc, 300, z_uc);

plot(velx(x_uc, :, z_uc)/ref, x_val, '-', LineWidth=3, displayname="LBM")

