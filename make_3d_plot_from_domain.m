function trans1 = make_3d_plot_from_domain(domain, options)
    % If using this, consider citing Iuliu Ardelean PhD Thesis
    arguments
        domain
        options.obstacles_face_alpha = 0.0;
        options.obstacles_face_color = [0.4 0.4 0.4];
        options.L_ref = 1;
        options.U_ref = 1;
        options.figID = 1;
    end
    L_ref = options.L_ref;
    U_ref = options.U_ref;
    APPRES = 1/domain.convert_to_meters;

    conc = domain.concentration;
    velx = domain.velx;
    vely = domain.vely;
    velz = domain.velz;
    velmag = (velx.^2+vely.^2+velz.^2).^0.5;
    [faces,verts,color_patch] = isosurface(conc, 1e-3, velmag/U_ref);
    verts = verts/APPRES/L_ref;
    [sizex, sizey, sizez] = size(conc);

    fontsize = 14;
    figure(options.figID)
    clf
    hold on
    xlabel("$X/L_{ref}$ (-)",'Interpreter','latex')
    ylabel("$Y/L_{ref}$ (-)",'Interpreter','latex')
    zlabel("$Z/L_{ref}$ (-)",'Interpreter','latex')
    colormap(turbo(100))
    camproj("perspective")
    view(35, 30)
    light
    grid on
    axis equal
    set(gca, "ticklabelinterpreter", 'latex')
    set(gca, "fontsize", fontsize)

    hcb = colorbar("eastoutside");
    hcb.TickLabelInterpreter = 'latex';
    hcb.FontSize = fontsize;
    hcb.Ticks = 0:0.25:2;
    ylabel(hcb, "Velocity Magnitude, $U_{mag}/U_{ref}$ (-)",'Interpreter','latex', 'FontSize', fontsize)
    clim([0, 1.5])
    
    xlim([0,sizez/APPRES/L_ref])
    ylim([0,sizex/APPRES/L_ref])
    zlim([0,sizey/APPRES/L_ref])

    trans1 = hgtransform('Parent', gca);
    M = makehgtform('translate', [sizez/APPRES/L_ref, 0, 0], 'yrotate', -pi/2);
    set(trans1, 'Matrix', M)

    patchy_patch = patch('Vertices',verts,'Faces',faces,'FaceVertexCData',color_patch,...
        'FaceColor','interp','EdgeColor','none', 'FaceAlpha', 0.4);
    set(patchy_patch, "Parent", trans1)

    drawnow
end
