% Analysis Parameters
clear
% close all

% rng(523253)
% rng(103829)
% rng(867678)
rng(498484)

num_stationary = 0;
num_uav_opts = 10;

domain_Re_opts = 1;
drone_mtow_opts = 10;
source_pos_opts = 1;
apf_radius_opts = 10;
apf_str_opts = 1;
inertia_opts = 0.5;
% path_planner_opts = ["ortho", "euclidean", "wind-aware"];
path_planner_opts = "ortho";
% batch_name_opts = ["no_cube_JAN23_proc", "single_cube_JAN23_proc", "nine_cube_JAN23_proc", "fifteen_cube_JAN23_proc"];
batch_name_opts = "no_cube_JAN23_proc";

exp_success = cell(length(num_uav_opts), length(batch_name_opts),length(source_pos_opts),length(domain_Re_opts), ...
    length(apf_radius_opts),length(apf_str_opts),length(inertia_opts),length(path_planner_opts),length(drone_mtow_opts));

for uav_num = 1:length(num_uav_opts)
    for batch_num = 1:length(batch_name_opts)
        for this_src_pos = source_pos_opts
            for this_re_num = 1:length(domain_Re_opts)
                batch_name = batch_name_opts(batch_num);
                domain = get_uc_domain("loadFromFile", 1, 'figID', 0, "filePath", ...
                    "uc_pollution/"+batch_name+"/"+batch_name+"_"+this_src_pos+".mat", ...
                    'flow_file_name', "uc_flow/"+batch_name+".mat", "obstacle_face_alpha", 1);
                domain.batch_name = batch_name;
                domain.process = this_src_pos;
                domain.L_ref = 20;
                domain.U_ref = 3;
                domain.f_re = domain_Re_opts(this_re_num);
                domain.convert_to_meters = 1/3;

                for apf_rad_num = 1:length(apf_radius_opts)
                    for apf_str_num = 1:length(apf_str_opts)
                        for inertia_num = 1:length(inertia_opts)
                            for path_planner_num = 1:length(path_planner_opts)
                                algo_options = get_algo("num_uav", num_uav_opts(uav_num), "num_mobile", num_uav_opts(uav_num), ...
                                    'num_stationary', num_stationary, 'y1', 2, 'y2', 2, 'inertia', inertia_opts(inertia_num), ...
                                    'apf_radius', apf_radius_opts(apf_rad_num), 'apf_strength', apf_str_opts(apf_str_num), ...
                                    'path_planner', path_planner_opts(path_planner_num));

                                for this_opt_mtow = 1:length(drone_mtow_opts)
                                    robot = get_robot("MTOW", drone_mtow_opts(this_opt_mtow), "num_rotors", 4, "overwrite_solidity", 0.1, ...
                                        "mach", 0.3, "k", 1.15, "efficiency", 0.6, "delta", 0.10, "Sfp", 0.025, ...
                                        "scale_Sfp", 1, "scale_cd0", 1, "color_and_alpha", [0.7 0.7 0.7, 0.3], ...
                                        "use_V_MR", 0, "use_scaling_laws", 1, "payload_fraction", 0.3, "response_time", 1);

                                    rng(498484)
                                    rng(67)
                                    domain.fig_id_3d_plot = 100+path_planner_num;
                                    % do experiment
                                    tic
                                    experiments = do_n_experiments(domain, robot, algo_options, ...
                                        'use_pso_poly_stat_flexnumuav', 1, 'num_experiments', 1, ...
                                        'figID', 1*(10000+uav_num), 'thresh_fnd', 10, 'make_export_gif', 0);
                                    toc

                                    exp_success{uav_num, batch_num, this_src_pos, this_re_num, apf_rad_num, apf_str_num, inertia_num, path_planner_num, this_opt_mtow} = experiments;
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end

%%
% val = nan(1,length(apf_str_opts));
% figure(901328)
% clf
% hold on
% set(gca, "XScale","log")
% grid on
% ylim([0,1])
% 
% for inertia_num = 1%1:length(inertia_opts)
%     for apf_rad_num = 1:length(apf_radius_opts)
%         for apf_str_num = 1:length(apf_str_opts)
%             val(apf_str_num) = mean(exp_success{1,1,5,1,apf_rad_num,apf_str_num,inertia_num}.mean_num_src_fnd);
%         end
%         plot(apf_str_opts, val, LineWidth=2)
% 
%     end
% end

%%
% success_val = nan(1,length(num_uav_opts));
% energy_val = nan(1,length(num_uav_opts));
% figure(937812)
% % clf
% hold on
% % set(gca, "XScale","log")
% grid on
% ylim([0,1])
% 
% for uav_num = 1:length(num_uav_opts)
%     success_val(uav_num) = exp_success{uav_num,1,5}.mean_num_src_fnd;
%     energy_val(uav_num) = mean(exp_success{uav_num,1,5}.mean_energy_consumed);
% end
% plot(num_uav_opts, success_val)

%%
% figure(1000)
% for iii = [5 6 7 8]
%     subplot(4,2,iii)
%     for i = 1:length(gca().Children)
%         set(gca().Children(i), "Visible", "off")
%     end
%     set(gca, "Visible", "off")
% end
% val = nan(1,length(inertia_opts));
% figure(1)
% clf
%
% subplot(1,3,1)
% hold on
% set(gca, "XScale","log")
% grid on
%
% subplot(1,3,2)
% hold on
% set(gca, "XScale","log")
% grid on
%
% subplot(1,3,3)
% hold on
% set(gca, "XScale","log")
% grid on
%
% this_max = -Inf;
% for apf_rad_num = 1:length(apf_radius_opts)
%     for apf_str_num = 1:length(apf_str_opts)
%         batch_num = 2;
%         val(apf_str_num) = mean(exp_success{batch_num,1,apf_rad_num,apf_str_num}.mean_num_src_fnd);
% %         val(apf_str_num) = mean(exp_success{batch_num,1,apf_rad_num,apf_str_num}.mean_dist_travelled);
% %         val(apf_str_num) = mean(exp_success{batch_num,1,apf_rad_num,apf_str_num}.mean_energy_consumed);
% %         val(apf_str_num) = mean(exp_success{batch_num,1,apf_rad_num,apf_str_num}.mean_num_iter);
% %         val(apf_str_num) = mean(exp_success{batch_num,1,apf_rad_num,apf_str_num}.time_needed);
%         if val(apf_str_num) > this_max
%             this_max = val(apf_str_num);
%         end
%     end
%     subplot(1,3,batch_num)
%     plot(apf_str_opts, val, 'LineWidth',2)
% end
%
% for apf_rad_num = 1:length(apf_radius_opts)
%     for apf_str_num = 1:length(apf_str_opts)
%         batch_num = 3;
%         val(apf_str_num) = mean(exp_success{batch_num,1,apf_rad_num,apf_str_num}.mean_num_src_fnd);
% %         val(apf_str_num) = mean(exp_success{batch_num,1,apf_rad_num,apf_str_num}.mean_dist_travelled);
% %         val(apf_str_num) = mean(exp_success{batch_num,1,apf_rad_num,apf_str_num}.mean_energy_consumed);
% %         val(apf_str_num) = mean(exp_success{batch_num,1,apf_rad_num,apf_str_num}.mean_num_iter);
% %         val(apf_str_num) = mean(exp_success{batch_num,1,apf_rad_num,apf_str_num}.time_needed);
%         if val(apf_str_num) > this_max
%             this_max = val(apf_str_num);
%         end
%     end
%     subplot(1,3,batch_num)
%     plot(apf_str_opts, val, 'LineWidth',2)
% end
%
% subplot(1,3,1)
% ylim([0, this_max*1.1])
% subplot(1,3,2)
% ylim([0, this_max*1.1])
% subplot(1,3,3)
% ylim([0, this_max*1.1])


