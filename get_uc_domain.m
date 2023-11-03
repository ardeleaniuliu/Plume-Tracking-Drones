function domain = get_uc_domain(velx, vely, velz, occupancy, sourceLocation, options)
    % If using this code, please consider citing Iuliu Ardelean PhD Thesis
    % Algorithm based on GADEN, available at https://doi.org/10.3390/s17071479
    arguments
        velx
        vely
        velz
        occupancy
        sourceLocation
        options.simTime = 200;
        options.ignoreTime = 0;
        options.figID = 1;
        options.APPRES = 1;
        options.noise_mag = 0.1;
        options.r0 = 1;
        options.deltaT = 0.1;
        options.gamma = 0.00333^2;
        options.cellSize = 1;
    end
    ignoreTime = options.ignoreTime;
    simTime = options.simTime;
    noiseMag = options.noise_mag;
    releaseRate = 10;
    gamma = options.gamma;
    r0 = options.r0;
    sequential = 1;
    numMoleculesPerFilament = 1;
    deltaT = options.deltaT;

    % flow field
    cellSize = options.cellSize;
    APPRES = options.APPRES;
    % may need to multiply by APPRES to convert from meters to domain units
    velx = APPRES*velx;
    vely = APPRES*vely;
    velz = APPRES*velz;
    [sizex, sizey, sizez] = size(occupancy);
    minx = 0.0;
    miny = 0.0;
    minz = 0.0;
    maxx = sizex;
    maxy = sizey;
    maxz = sizez;

    numParticles = round(releaseRate*simTime);

    % PARTICLE FILAMENT METHOD PART 1
    % store x y z t in pMatrix
    tic
    pMatrix = nan(round(simTime/deltaT), 4, numParticles);
    for particle = 1:numParticles
        xyz = sourceLocation + (rand(1, 3) - 0.5).*cellSize;
        xyz = min(xyz, [maxx-1,maxy-1,maxz-1]); %% added JUN23
        xyz = max(xyz, [minx,miny,minz]); %% added JUN23
        t = 0;
        pMatrixOne = nan(round(simTime/deltaT), 4);
        while (t <= simTime - deltaT)
            % store position and time
            pMatrixOne(round(t/deltaT) + 1, :) = [xyz(1), xyz(2), xyz(3), t];
            % advection
            indices = floor((xyz - [minx miny minz])/cellSize) + [1 1 1];
            vx = velx(indices(1), indices(2), indices(3));
            vy = vely(indices(1), indices(2), indices(3));
            vz = velz(indices(1), indices(2), indices(3));
            oldxyz = xyz;
            xyz = xyz + [vx vy vz]*deltaT;
            indices = floor((xyz - [minx miny minz])/cellSize) + [1 1 1];
            t = t + deltaT;
            if xyz(1)<=minx || xyz(1)>=maxx || xyz(2)<=miny || xyz(2)>=maxy || xyz(3)<=minz || xyz(3)>=maxz
                xyz = oldxyz;
                indices = floor((xyz - [minx miny minz])/cellSize) + [1 1 1];
            end
            if occupancy(indices(1), indices(2), indices(3)) == 1
                xyz = oldxyz;
                indices = floor((xyz - [minx miny minz])/cellSize) + [1 1 1];
            end
            if occupancy(indices(1), indices(2), indices(3)) == 2
                break
            end
            if (t >= simTime)
                break
            end
            % noise
            oldxyz = xyz;
            xyz = xyz + randn(1,3)*noiseMag;
            indices = floor((xyz - [minx miny minz])/cellSize) + [1 1 1];
            if xyz(1)<=minx || xyz(1)>=maxx || xyz(2)<=miny || xyz(2)>=maxy || xyz(3)<=minz || xyz(3)>=maxz
                xyz = oldxyz;
                indices = floor((xyz - [minx miny minz])/cellSize) + [1 1 1];
            end
            if occupancy(indices(1), indices(2), indices(3)) == 1
                xyz = oldxyz;
            end
        end
        pMatrix(:,:,particle) = pMatrixOne;
    end
    toc

    % PARTICLE FILAMENT METHOD PART 2
    tic
    c_twin_avg = zeros(sizex, sizey, sizez);
    for particle = 1:numParticles
        for t = 0:deltaT:(simTime-deltaT)
            xyzt = pMatrix(round(t/deltaT) + 1, :, particle);
            if (any(isnan(xyzt)))
                break
            end
            tp = xyzt(4) + (mod(particle, simTime)/releaseRate)*sequential;
            if (tp > simTime-deltaT)
                break
            end
            if (tp < ignoreTime)
                continue
            end
            r = (r0^2 + gamma*xyzt(4))^0.5;
            grid_size_m = min(cellSize, r);
            %             numEval = 8;
            numEval = ceil(6*r / grid_size_m);
            eval_vector = 0:numEval;
            %         px = (xyzt(1) - 0.5*numEval*cellSize) + eval_vector*cellSize;
            %         py = (xyzt(2) - 0.5*numEval*cellSize) + eval_vector*cellSize;
            %         pz = (xyzt(3) - 0.5*numEval*cellSize) + eval_vector*cellSize;
            px = (xyzt(1) - 3*r) + eval_vector*grid_size_m;
            py = (xyzt(2) - 3*r) + eval_vector*grid_size_m;
            pz = (xyzt(3) - 3*r) + eval_vector*grid_size_m;
            px = round(px, 10);
            py = round(py, 10);
            pz = round(pz, 10);
            idx = floor((px - minx)/cellSize) + 1;
            idy = floor((py - miny)/cellSize) + 1;
            idz = floor((pz - minz)/cellSize) + 1;
            if any(idx<1 | idx>sizex | idy<1 | idy>sizey | idz<1 | idz>sizez)
                for i = 0:numEval
                    for j = 0:numEval
                        for k = 0:numEval
                            %                         px = (xyzt(1) - 0.5*numEval*cellSize) + i*cellSize;
                            %                         py = (xyzt(2) - 0.5*numEval*cellSize) + j*cellSize;
                            %                         pz = (xyzt(3) - 0.5*numEval*cellSize) + k*cellSize;
                            px = (xyzt(1) - 3*r) + i*grid_size_m;
                            py = (xyzt(2) - 3*r) + j*grid_size_m;
                            pz = (xyzt(3) - 3*r) + k*grid_size_m;
                            idx = floor((px-minx)/cellSize) + 1;
                            idy = floor((py-miny)/cellSize) + 1;
                            idz = floor((pz-minz)/cellSize) + 1;
                            if idx<1 || idx>sizex || idy<1 || idy>sizey || idz<1 || idz>sizez
                                continue
                            end
                            dsq = (px-xyzt(1))^2 + (py-xyzt(2))^2 + (pz-xyzt(3))^2;
                            c_twin_avg(idx,idy,idz) = c_twin_avg(idx,idy,idz) + numMoleculesPerFilament/r^3*exp(-dsq/(2*r^2));
                        end
                    end
                end
                continue
            end
            numEvalTrue = numEval + 1;
            px = repmat(px, numEvalTrue, 1, numEvalTrue);
            py = repmat(py', 1, numEvalTrue, numEvalTrue);
            pz = reshape(pz, 1, 1, numEvalTrue);
            pz = repmat(pz, numEvalTrue, numEvalTrue, 1);
            dsq = (px-xyzt(1)).^2 + (py-xyzt(2)).^2 + (pz-xyzt(3)).^2;
            idxv = min(idx):max(idx);
            idyv = min(idy):max(idy);
            idzv = min(idz):max(idz);
            c_twin_avg(idxv,idyv,idzv) = c_twin_avg(idxv,idyv,idzv) + numMoleculesPerFilament/r^3*exp(-dsq/(2*r^2));
        end
    end
    c_twin_avg = c_twin_avg/(round((simTime-ignoreTime)/deltaT) + 1);
    c_twin_avg = (occupancy ~= 1).*c_twin_avg;
    toc

    if options.figID
        fontsize = 13;
        cellSize = 1/APPRES;
        c_replica_slice = c_twin_avg(:,:,sizez/2);
        borderColor = 0.4*[1 1 1];
        figure(options.figID)
        colormap(flipud(hot(100)))
        contourf((1:sizex)*cellSize, (1:sizey)*cellSize, c_replica_slice', 100,'edgecolor','none')
        hold on
        occupancyAlpha = occupancy(:, :, sizez/2)';
        cData = ones(sizey, sizex, 3);
        cData(:,:,1) = ones(sizey, sizex)*borderColor(1);
        cData(:,:,2) = ones(sizey, sizex)*borderColor(2);
        cData(:,:,3) = ones(sizey, sizex)*borderColor(3);
        imagesc('XData', (1:sizex)*cellSize, 'YData', (1:sizey)*cellSize, 'CData', cData, 'AlphaData', occupancyAlpha)

        ss = streamslice((1:sizex)*cellSize, (1:sizey)*cellSize, velx(:,:,sizez/2)', vely(:,:,sizez/2)');
        set(ss, 'Color', borderColor);
        hold off
        xlabel("Displacement X (m)",'Interpreter','latex')
        ylabel("Displacement Y (m)",'Interpreter','latex')
        hcb = colorbar;
        hcb.TickLabelInterpreter = 'latex';
        ylabel(hcb, 'Concentration (-)','Interpreter','latex')
        axis equal
        a = gca;
        a.TickLabelInterpreter = 'latex';
        a.FontSize = fontsize;

        drawnow
    end

    domain.velx = velx;
    domain.vely = vely;
    domain.velz = velz;
    domain.source_location = sourceLocation;
    domain.concentration = c_twin_avg;
    domain.occupancy = occupancy;
    domain.domain_size = size(c_twin_avg,1);
    domain.convert_to_meters = 1/APPRES;

end
