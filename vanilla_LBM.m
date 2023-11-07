function [UX, UY, UZ, F, DENSITY] = vanilla_LBM(BOUND, options)
    % code based off various online sources
    % this code is intended for quick prototyping and demos
    arguments
        BOUND % This is a 2D occupancy matrix
        options.F = nan;
        options.DENSITY
        options.UX
        options.UY
        options.omega = 1.0;
        options.fig_id = 1;
        options.max_num_step = 1000;
    end
    tic

    nx = size(BOUND, 1);
    ny = size(BOUND, 2);

    c_squ = 1/3;
    t1 = 4/9;
    t2 = 1/9;
    t3 = 1/36;

    density = 1;
    F = repmat(density/9, [nx ny 9]);
    FEQ = F;
    msize = nx*ny;
    CI = 0:msize:msize*7;
    ON = find(BOUND); % matrix offset of each Occupied Node
    TO_REFLECT = [ON+CI(1) ON+CI(2) ON+CI(3) ON+CI(4) ON+CI(5) ON+CI(6) ON+CI(7) ON+CI(8)];
    REFLECTED = [ON+CI(5) ON+CI(6) ON+CI(7) ON+CI(8) ON+CI(1) ON+CI(2) ON+CI(3) ON+CI(4)];
    avu = 1;
    prevavu = 1;
    ts = 0;
    numactivenodes = sum(sum(1-BOUND));

    DENSITY = ones(nx,ny);
    ux_inlet = 0.01;
    uy_inlet = 0;
    UX = zeros(nx,ny);
    UY = zeros(nx,ny);
    UZ = zeros(nx,ny);
    UX(1,:) = ux_inlet;
    UY(1,:) = uy_inlet;

    %     ds = 1;
    %     Re = 10000;
    %     nu = ux_inlet*50/Re;
    %     tau = 3*nu/ds + 0.5;
    %     omega = 1/tau;
    omega = options.omega;

    if ~isnan(options.F)
        F = options.F;
        UX = options.UX;
        UY = options.UY;
        DENSITY = options.DENSITY;
    end

    while (ts<options.max_num_step && 1e-10<abs((prevavu-avu)/avu)) || ts<100
        % Propagate
        F(:,:,1) = F([nx 1:nx-1],:,1);            % x <- x-1
        F(:,:,2) = F([nx 1:nx-1],[ny 1:ny-1],2);  % x <- x-1, y <- y-1
        F(:,:,3) = F(:,[ny 1:ny-1],3);            % y <- y-1
        F(:,:,4) = F([2:nx 1],[ny 1:ny-1],4);     % x <- x+1, y <- y-1
        F(:,:,5) = F([2:nx 1],:,5);               % x <- x+1
        F(:,:,6) = F([2:nx 1],[2:ny 1],6);        % x <- x+1, y <- y+1
        F(:,:,7) = F(:,[2:ny 1],7);               % y <- y+1
        F(:,:,8) = F([nx 1:nx-1],[2:ny 1],8);     % x <- x-1, y <- y+1

        % APPLY KNOWN U AT WEST BOUNDARY
        DENSITY(1,:) = 1./(1-ux_inlet)*(F(1,:,9) + F(1,:,3) + F(1,:,7) + 2*(F(1,:,5) + F(1,:,4) + F(1,:,6)));
        F(1,:,1) = F(1,:,5) + 2/3*DENSITY(1,:).*UX(1,:);
        F(1,:,2) = F(1,:,6) - 0.5*(F(1,:,3)-F(1,:,7)) + 1/6*DENSITY(1,:).*UX(1,:) + 1/2*DENSITY(1,:).*UY(1,:);
        F(1,:,8) = F(1,:,4) + 0.5*(F(1,:,3)-F(1,:,7)) + 1/6*DENSITY(1,:).*UX(1,:) - 1/2*DENSITY(1,:).*UY(1,:);

        % N: WALL
        %         F(:,ny,6) = F(:,ny,2);
        %         F(:,ny,7) = F(:,ny,3);
        %         F(:,ny,8) = F(:,ny,4);
        % N: OPEN
        F(:,ny,6) = F(:,ny-1,6);
        F(:,ny,7) = F(:,ny-1,7);
        F(:,ny,8) = F(:,ny-1,8);

        %         % S: WALL
        %         F(:,1,2) = F(:,1,6);
        %         F(:,1,3) = F(:,1,7);
        %         F(:,1,4) = F(:,1,8);
        % S: OPEN
        F(:,1,2) = F(:,2,2);
        F(:,1,3) = F(:,2,3);
        F(:,1,4) = F(:,2,4);

        % E:
        F(nx,:,4) = F(nx-1,:,4);
        F(nx,:,5) = F(nx-1,:,5);
        F(nx,:,6) = F(nx-1,:,6);

        BOUNCEDBACK = F(TO_REFLECT); % Densities bouncing back at next timestep

        DENSITY = sum(F,3);
        UX = (sum(F(:,:,[1 2 8]),3)-sum(F(:,:,[4 5 6]),3))./DENSITY;
        UY = (sum(F(:,:,[2 3 4]),3)-sum(F(:,:,[6 7 8]),3))./DENSITY;

        UX(1,1:end) = ux_inlet;
        UY(1,1:end) = uy_inlet;
        %     UX(:,1)=0; UY(:,1)=0; % SOUTH wall
        %     UX(:,ny)=0; UY(:,ny)=0; % NORTH wall
        UX(ON) = 0;
        UY(ON) = 0;
        DENSITY(ON) = 0;

        U_SQU = UX.^2 + UY.^2;
        U_C2 = UX + UY;
        U_C4 = -UX + UY;
        U_C6 = -U_C2;
        U_C8 = -U_C4;

        % Calculate equilibrium distribution: stationary
        FEQ(:,:,9) = t1*DENSITY.*(1 - U_SQU/(2*c_squ));
        % nearest-neighbours
        FEQ(:,:,1) = t2*DENSITY.*(1 + UX/c_squ + 0.5*(UX/c_squ).^2 - U_SQU/(2*c_squ));
        FEQ(:,:,3) = t2*DENSITY.*(1 + UY/c_squ + 0.5*(UY/c_squ).^2 - U_SQU/(2*c_squ));
        FEQ(:,:,5) = t2*DENSITY.*(1 - UX/c_squ + 0.5*(UX/c_squ).^2 - U_SQU/(2*c_squ));
        FEQ(:,:,7) = t2*DENSITY.*(1 - UY/c_squ + 0.5*(UY/c_squ).^2 - U_SQU/(2*c_squ));
        % next-nearest neighbours
        FEQ(:,:,2) = t3*DENSITY.*(1 + U_C2/c_squ + 0.5*(U_C2/c_squ).^2 - U_SQU/(2*c_squ));
        FEQ(:,:,4) = t3*DENSITY.*(1 + U_C4/c_squ + 0.5*(U_C4/c_squ).^2 - U_SQU/(2*c_squ));
        FEQ(:,:,6) = t3*DENSITY.*(1 + U_C6/c_squ + 0.5*(U_C6/c_squ).^2 - U_SQU/(2*c_squ));
        FEQ(:,:,8) = t3*DENSITY.*(1 + U_C8/c_squ + 0.5*(U_C8/c_squ).^2 - U_SQU/(2*c_squ));

        F = omega*FEQ + (1-omega)*F;
        F(REFLECTED) = BOUNCEDBACK;
        prevavu = avu;
        avu = sum(sum(UX))/numactivenodes;
        ts = ts + 1;

        if ts == 1 || mod(ts, 100) == 0
            fontsize = 12;
            figure(options.fig_id);
            mymap = turbo(100);
            colormap(mymap(1:end,:))
            %image(2-BOUND');
            hold on;
            step = 1;
            indx = 1:step:nx;
            indy = 1:step:ny;
            velxslice = (UX.^2 + UY.^2).^0.5;
            velxslice = velxslice/ux_inlet;
            imagesc(velxslice')
            % quiver(xind, yind, UX(xind,yind)', UY(xind,yind)', 1.5);
            borderColor = [0.7 0.7 0.7];
            occupancyAlpha = BOUND' == 1;
            cData = ones(length(indy), length(indx), 3);
            cData(:,:,1) = ones(length(indy), length(indx))*borderColor(1);
            cData(:,:,2) = ones(length(indy), length(indx))*borderColor(2);
            cData(:,:,3) = ones(length(indy), length(indx))*borderColor(3);
            imagesc(cData, 'AlphaData', occupancyAlpha)
            ss = streamslice(UX', UY');
            set(ss, 'Color', borderColor);
            clim([0, 1.5])
            title("Flow field after $"+ts+"\delta t = "+toc+"s $", "Interpreter","latex");
            xlabel("Displacement $X/L_{ref}$ (-)",'Interpreter','latex')
            ylabel("Displacement $Z/L_{ref}$ (-)",'Interpreter','latex')
            xticks(0:100:nx)
            yticks(0:100:ny)
            hcb = colorbar("eastoutside");
            hcb.TickLabelInterpreter = 'latex';
            hcb.FontSize = fontsize;
            hcb.Ticks = 0:0.5:3;
            ylabel(hcb, "Velocity Magnitude, $U_{mag}/U_{ref}$ (-)",'Interpreter','latex', 'FontSize', fontsize)
            axis image
            xlim([0, nx])
            ylim([0, ny])
            a = gca;
            a.TickLabelInterpreter = 'latex';
            a.FontSize = fontsize;
            box on
            drawnow
        end
    end

end
