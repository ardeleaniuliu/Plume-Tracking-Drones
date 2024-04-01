load("uc_flow\internal_model.mat", "internal_model")

flow_file_name = "uc_flow/infotaxis_empty.mat";
load(flow_file_name, "velx", "vely", "velz", "occupancy")

do_internal_visualize = 1;
if do_internal_visualize
    tic
    [sizex, sizey, sizez] = size(occupancy);

    my_color = lines(8);
    xs = 1;
    ys = 1;
    concentration = reshape(internal_model(xs,ys,1,:,:,1), 30, 30, 1);
    figure(127)
    colormap(flipud(bone(10)))
    hold on
    %     caxis([0,1])
    q = (concentration - min(concentration,[],'all'))./(max(concentration,[],'all')-min(concentration,[],'all'));
    imagesc_plot = imagesc(q');
    borderColor = [0.9 0.4 0.1];
    occupancyAlpha = occupancy(:, :, sizez/2)' == 1;
    cData = ones(sizey, sizex, 3);
    cData(:,:,1) = ones(sizey, sizex)*borderColor(1);
    cData(:,:,2) = ones(sizey, sizex)*borderColor(2);
    cData(:,:,3) = ones(sizey, sizex)*borderColor(3);
    imagesc(cData, 'AlphaData', occupancyAlpha)
    [~, contourhandle] = contour(concentration'>=0.5, 1, "LineWidth", 2, "LineColor", my_color(2,:));
    src_plot = plot(xs+1, ys+1, 'x', "LineWidth", 2, "MarkerSize", 12, "Color", my_color(2,:));
    %text(xs+0.5, ys, "Source Location", "FontSize", 12, "Color", "r")
    %text(13, 20.5, "Plume Contour", "FontSize", 12, "Color", "r")
    ss = streamslice(velx(:,:,sizez/2)', vely(:,:,sizez/2)');
    set(ss, 'Color', my_color(3,:));
    set(ss, 'Linewidth', 1);
    cbar = colorbar;
    caxis([0,1])
    cbar.Ticks = [0 1];
    cbar.TickLabelInterpreter = 'latex';
    cbar.Label.String = "Pollution Concentration, $C/C_{ref}$ (-)";
    cbar.Label.FontSize = 14;
    cbar.FontSize = 14;
    cbar.Label.Interpreter = "latex";
    title("Infotaxis Internal Representation", "FontSize", 14, "Interpreter","latex")
    xlabel("X","FontSize",14,"Interpreter","latex")
    ylabel("Y","FontSize",14,"Interpreter","latex")
    %
    hold on
    axis image
    drawnow
    xticks([])
    yticks([])

    for xs = 1:3:30
        for ys = 1:3:30
            concentration = reshape(internal_model(xs,ys,1,:,:,1), 30, 30, 1);
            q = (concentration - min(concentration,[],'all'))./(max(concentration,[],'all')-min(concentration,[],'all'));
            imagesc_plot.CData = q';
            contourhandle.ZData = concentration';
            src_plot.XData = xs+1;
            src_plot.YData = ys+1;
            drawnow
%             exportgraphics(gcf, "infotaxis_gif_internal_empty_11OCT2023a.gif", "Append", true)
            toc
        end
    end
end

%%
pos = [25, 10, 1];

xtrue = 6;
ytrue = 15;
ztrue = 1;
pos_true = [xtrue, ytrue, ztrue];

flow_file_name = "uc_flow/infotaxis_cube.mat";
load(flow_file_name, "velx", "vely", "velz")
load("uc_flow\internal_model_2.mat", "internal_model", "occupancy")
concentration = reshape(internal_model(xtrue,ytrue,1,:,:,1), 30, 30, 1);
[sizex, sizey, sizez] = size(concentration);

load("uc_flow\internal_model.mat", "internal_model")

xs = 1:sizex;
ys = 1:sizey;
zs = 1;

p_src = ones(sizex, sizey, 1)/(sizex*sizey);
% p_src = normalize(rand(sizex, sizey, 1),"range");

num_time_steps = 10000;
traj = nan(num_time_steps,3);
hit_seq = nan(num_time_steps,1);
entropy_seq = nan(num_time_steps,1);

figure(123)
clf
hold on
colormap((gray(10)))
q = (p_src - min(p_src,[],'all'))./(max(p_src,[],'all')-min(p_src,[],'all'));
imagesc_plot = imagesc(q');
borderColor = [0.9 0.4 0.1];
occupancyAlpha = occupancy(:, :, 15)' == 1;
cData = ones(sizey, sizex, 3);
cData(:,:,1) = ones(sizey, sizex)*borderColor(1);
cData(:,:,2) = ones(sizey, sizex)*borderColor(2);
cData(:,:,3) = ones(sizey, sizex)*borderColor(3);
imagesc(cData, 'AlphaData', occupancyAlpha)
ss = streamslice(velx(:,:,15)', vely(:,:,15)');
my_color = lines(8);
set(ss, 'Color', my_color(3,:));
set(ss, 'LineWidth', 1);
contour_plot = contour(concentration'>=0.5, 1, "LineWidth", 2, "LineColor", my_color(2,:));
plot(xtrue+1, ytrue, 'x', "LineWidth", 2, "Color", my_color(2,:), "MarkerSize", 12)
% text(xtrue+0.75, ytrue, "Source Location", "FontSize", 10, "Color", [0.9 0.5 0.1],"BackgroundColor","w")
% text(13, 20.5, "Plume Contour", "FontSize", 10, "Color", [0.9 0.5 0.1],"BackgroundColor","w")
title("Infotaxis Experiment", "FontSize", 14, "Interpreter","latex")
cbar = colorbar;
cbar.Ticks = [0 1];
cbar.TickLabels = ["Low", "High"];
cbar.TickLabelInterpreter = 'latex';
cbar.Label.String = "Predicted Source Location";
cbar.Label.FontSize = 14;
cbar.FontSize = 14;
cbar.Label.Interpreter = 'latex';
% caxis([0,1])
axis image
xlabel("X","FontSize",14,"Interpreter","latex")
ylabel("Y","FontSize",14,"Interpreter","latex")
traj_plot = plot(traj(:,1), traj(:,2), "-", "LineWidth", 2, "Color",my_color(1,:));
agent_plot = plot(pos(:,1), pos(:,2), "o", "LineWidth", 2, "Color",my_color(1,:));
drawnow
pause(0.1)
xticks([])
yticks([])

% [~,ind] = max(p_src,[],'all');
% [i,j] = ind2sub([30,30], ind);
% loc_max_plot = plot(i,j,'x', "LineWidth", 2, "Color", my_color(1,:), "MarkerSize", 12);

for t = 1:num_time_steps

    traj(t,:) = pos;

    if all(abs(pos - pos_true) <= 1)
        source_found = 1;
        break
    end

    this_concentration = concentration(pos(1), pos(2), pos(3)); %% check what plume.sample returns
    hit = this_concentration >= 0.5;
    hit_seq(t) = hit;

    p_src = update_prob_src(p_src, internal_model, xs, ys, zs, pos, hit);
    this_entropy = entropy(p_src);
    entropy_seq(t) = this_entropy;

    moves = [0 1 0; 0 -1 0; 1 0 0; -1 0 0;];
    num_moves = 4;
    delta_entropy_expected = nan(1, num_moves);
    delta_entropy_src_fnd = -this_entropy;
    for i = 1:num_moves
        this_move = moves(i,:);
        next_pos = pos + this_move;

        try
            p_src_fnd = p_src(next_pos(1), next_pos(2), next_pos(3));
            p_src_not_fnd = 1 - p_src_fnd;
            
        catch ME
            continue
        end
        if occupancy(next_pos(1), next_pos(2), next_pos(3)) == 1
            p_src(next_pos(1),next_pos(2),next_pos(3)) = 0;
            continue
        end

        p_sample_1 = get_p_sample(1, next_pos, internal_model, xs, ys, zs, p_src);
        prob_src_1 = update_prob_src(p_src, internal_model, xs, ys, zs, pos, 1);
        entropy_1 = entropy(prob_src_1);
        delta_entropy_1 = entropy_1 - this_entropy;

        p_sample_0 = get_p_sample(0, next_pos, internal_model, xs, ys, zs, p_src);
        prob_src_0 = update_prob_src(p_src, internal_model, xs, ys, zs, pos, 0);
        entropy_0 = entropy(prob_src_0);
        delta_entropy_0 = entropy_0 - this_entropy;

        delta_entropy_src_not_fnd = p_sample_0*delta_entropy_0 + p_sample_1*delta_entropy_1;
        delta_entropy_expected(i) = p_src_fnd*delta_entropy_src_fnd + p_src_not_fnd*delta_entropy_src_not_fnd;
    end

    [~, ind] = min(delta_entropy_expected);
    best_move = moves(ind, :);
    pos = pos + best_move;
    % p_src = normalize(p_src, "range");

    q = (p_src - min(p_src,[],'all'))./(max(p_src,[],'all')-min(p_src,[],'all'));
    imagesc_plot.CData = q';
    traj_plot.XData = traj(:,1);
    traj_plot.YData = traj(:,2);
    agent_plot.XData = pos(:,1);
    agent_plot.YData = pos(:,2);
    [~,ind] = max(p_src,[],'all');
    [i,j] = ind2sub([30,30], ind);
    %     loc_max_plot.XData = i;
    %     loc_max_plot.YData = j;
    drawnow

%     exportgraphics(gca, "infotaxis_gif_11OCT2023a.gif", "Append", true)

end

function p_src = update_prob_src(p_src, internal_model, xs, ys, zs, pos, hit)
    % prob of a hit at pos pos, given src_pos
%     predicted_1 = nan(length(xs),length(ys),length(zs));
%     predicted_0 = nan(length(xs),length(ys),length(zs));
%     for xx = xs
%         for yy = ys
%             for zz = zs
%                 this_concentration = internal_model(xx,yy,1,:,:,1) >= 0.5;
%                 this_concentration = reshape(this_concentration, 30, 30, 1);
% 
%                 predicted_1(xx,yy,zz) = this_concentration(pos(1), pos(2), 1) == 1;
%                 predicted_0(xx,yy,zz) = this_concentration(pos(1), pos(2), 1) == 0;
%             end
%         end
%     end
    
    imdl = internal_model >= 0.5;
    predicted_1 = imdl(:,:,1,pos(1),pos(2),1) == 1;
    predicted_0 = imdl(:,:,1,pos(1),pos(2),1) == 0;

    p_src = p_src + (hit==1)*predicted_1 + (hit==0)*predicted_0;
    p_src(pos(1), pos(2), pos(3)) = 0;
    % p_src = p_src./max(p_src,[],"all");

end

function p_sample_val = get_p_sample(val, next_pos, internal_model, xs, ys, zs, p_src)
%     predicted_val = nan(length(xs),length(ys),length(zs));
%     for xx = xs
%         for yy = ys
%             for zz = zs
%                 this_concentration = internal_model(xx,yy,1,:,:,1);
%                 this_concentration = reshape(this_concentration, 30, 30, 1); 
%                 % this_concentration = this_concentration*p_src./max(p_src,[],"all"); %% added p_src
%                 this_concentration = this_concentration >= 0.5;
%                 predicted_val(xx,yy,zz) = this_concentration(next_pos(1), next_pos(2), 1) == val;
%             end
%         end
%     end

    predicted_val = (internal_model(:, :, 1, next_pos(1), next_pos(2), 1) >= 0.5) == val;

    p_sample_val = sum(predicted_val, "all")/numel(predicted_val);
end



