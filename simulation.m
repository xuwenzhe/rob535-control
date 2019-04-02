    % Copyright (C) 2018, ETH Zurich, D-ITET, Kenneth Kuchera, Alexander Liniger
    % Licensed under the Apache License, Version 2.0 (the "License");
    % you may not use this file except in compliance with the License.
    % You may obtain a copy of the License at
    % 
    %     http://www.apache.org/licenses/LICENSE-2.0
    % 
    % Unless required by applicable law or agreed to in writing, software
    % distributed under the License is distributed on an "AS IS" BASIS,
    % WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    % See the License for the specific language governing permissions and
    % limitations under the License.
    %% MPCC Simulation Script
    % clear
    % close all
    % clc
    %% add spline library
    addpath('splines');

    %% Load Parameters
    %CarModel = 'ORCA';
    CarModel = 'FullSize';

    MPC_vars = getMPC_vars(CarModel);
    ModelParams=getModelParams(MPC_vars.ModelNo);
    % choose optimization interface options: 'Yalmip','CVX','hpipm','quadprog'
    MPC_vars.interface = 'quadprog';

    nx = ModelParams.nx;
    nu = ModelParams.nu;
    N = MPC_vars.N;
    Ts = MPC_vars.Ts;
    %% import an plot track
    % use normal ORCA Track
    load Tracks/TestTrack.mat;
    % append virtual track after the "end" to account for "finish line horizon" boundary condition
    [track_virtual] = append_virtual_track(TestTrack);

    % use RCP track
    % shrink track by half of the car widht plus safety margin
    % TODO implement orientation depending shrinking in the MPC constraints
    safteyScaling = 0.2;
    [track, trackWidth] = borderAdjustment(track_virtual,ModelParams,0.05);

    %trackWidth = norm(track.bl(:,1)-track.br(:,1));
    % plot shrinked and not shrinked track 

    %% Simulation length and plotting
    simN = 2000;
    %0=no plots, 1=plot predictions
    plotOn = 1;
    %0=real time iteration, 1=fixed number of QP iterations, 2=fixed number of damped QP iterations
    QP_iter = 2;
    % number of cars 
    % there are two examples one with no other cars and one with 4 other cars
    % (inspired by the set up shown in the paper)
    n_cars = 1; % no other car
    %% Fit spline to track
    % TODO spline function only works with regular spaced points.
    % Fix add function which given any  line and bound generates equlally
    % space tracks.
    [traj, borders] =splinify(track);

    if plotOn == 1
        figure(1);
        plot(track.br(1,:),track.br(2,:),'r')
        hold on
        plot(track.bl(1,:),track.bl(2,:),'r')
        plot(track_virtual.br(1,:),track_virtual.br(2,:),'k')
        plot(track_virtual.bl(1,:),track_virtual.bl(2,:),'k')
    end

    % refine track
    theta = 0:5:traj.ppx.breaks(end);

    cline = [ppval(traj.ppx, theta);
             ppval(traj.ppy, theta)];

    bl = [ppval(borders.pplx, theta);
          ppval(borders.pply, theta)];
    br = [ppval(borders.pprx, theta);
          ppval(borders.ppry, theta)];


    %Nobs = 25;

    Xobs = load("testSet/obs.mat")
    Xobs = Xobs.Xobs;
    Nobs = numel(Xobs);

    newTrack = struct('cline', cline, 'bl', bl, 'br', br);
    [track,newTrack, trackWidth] = borderAdjustment(newTrack,ModelParams,safteyScaling);
    [traj, borders] = splinify(track);

    %Xobs = cell(1, Nobs);
    if Nobs ~= 0
        %Xobs = generateRandomObstacles(Nobs, TestTrack);
        track = obstacleAdjust(track, Xobs, traj, borders, trackWidth);
        [track,trackOld, trackWidth] = borderAdjustment(track,ModelParams,0.3);
        [traj, borders] = splinify(track);
    end
    trackWidth =  sqrt(sum((track.bl - track.br).^2));
    tl = traj.ppy.breaks(end); % total length

    % store all data in one struct
    TrackMPC = struct('traj',traj,'borders',borders,'track_cline',track.cline,'tl',tl);
    %% Define starting position
    startIdx = 1; %point (in terms of track line array) along the track 
    % where the car starts, on the  line, aligned with the track, driving
    % straight with vx0
    %since the used bicycle model is not well defined for slow velocities use vx0 > 0.5
    if CarModel == "ORCA"
        vx0 = 1;
    elseif CarModel == "FullSize"
        %vx0 = 15;
        %% set the same as the initial speed of the car
        vx0 = 5;
    end

    ode45_options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);

    % find theta that coresponds to the 10th point on the line
    %[theta, ~] = findTheta([track.(1,startIdx),track.cline(2,startIdx)],track.cline,traj.ppx.breaks,trackWidth,startIdx);
    [theta, ~] = findTheta([287, -176], track.cline, traj.ppx.breaks, trackWidth, startIdx);

    x0 = [287, 5, -176, 0, 2, 0, theta]';

    %x0 = [track.(1,startIdx),track.cline(2,startIdx),... % point on centerline
          %atan2(ppval(traj.dppy,theta),ppval(traj.dppx,theta)),... % aligned with line
          %vx0 ,0,0,theta]'; %driving straight with vx0, and correct theta progress
        
    % the find theta function performs a local search to find the projection of
    % the position onto the line, therefore, we use the start index as an
    % starting point for this local search
    last_closestIdx = startIdx;
    %% First initial guess
    %x = repmat(x0,1,N+1); % all points identical to current measurment
    x = zeros(7, N+1);
    x(:, 1) = x0;
    u = repmat([0; 200; 5.5], 1, N);


    % simulate a ode45 first with zero input
    [~, x] = ode45(@(t, x) car_model(t, x, u(:,1)), 0:Ts:N*Ts, x0, ode45_options);
    x = x';
    for i = 2:N+1
        [x(7,i), ~] = findTheta([x(1,i), x(3,i)] , track.cline, traj.ppx.breaks, trackWidth, startIdx);
        u(3, i-1) = (x(7,i) - x(7, i-1))/Ts;
    end

    %for i = 2:N+1
        %theta_next = x(ModelParams.stateindex_theta,i-1)+Ts*vx0;
        %phi_next = atan2(ppval(traj.dppy,theta_next),ppval(traj.dppx,theta_next));
        %% phi_next can jump by two pi, make sure there are no jumps in the
        %% initial guess
        %if (x(ModelParams.stateindex_phi,i-1)-phi_next) < -pi
            %phi_next = phi_next-2*pi;
        %end
        %if (x(ModelParams.stateindex_phi,i-1)-phi_next) > pi
            %phi_next = phi_next+2*pi;
        %end
        %x(:,i) = [ppval(traj.ppx,theta_next),ppval(traj.ppy,theta_next),... % point on line
                  %phi_next,... % aligned with line
                  %vx0 ,0,0,theta_next]'; %driving straight with vx0, and correct theta progress
    %end

    %u = zeros(3,N); % zero inputs
    %uprev = zeros(3,1); % last input is zero
    uprev = u(:,1);
    %%% Ohter cars
    %Y = ObstacelsState(track,traj,trackWidth,n_cars);
    Y = [];

    if size(Y,2) ~= n_cars-1
        error('n_cars and the number of obstacles in "Y" does not match')
    end
    %%% Initialize logging arrays
    X_log = zeros(nx*(N+1),simN);
    U_log = zeros(3*N,simN);
    B_log = zeros(4*N,simN);
    qpTime_log = zeros(1,simN);
    %% initializtion
    % solve problem 5 times without applying input
    % inspiered by sequential quadratic programming (SQP)
    for i = 1:5
        % formulate MPCC problem and solve it
        Iter_damping = 0.0; % 0 no damping
        [x_up, u_up, b, exitflag,info] = optimizer(TrackMPC,MPC_vars,ModelParams, n_cars, Y, x, u, x0, uprev);
        x = Iter_damping*x + (1-Iter_damping)*x_up;
        u = Iter_damping*u + (1-Iter_damping)*u_up;

        if plotOn == 1
            % plot predictions
            PlotPrediction(x,u,b,Y,track,TestTrack,traj,MPC_vars,ModelParams, Xobs)
        end
    end
    %%% Simulation
    for i = 1: simN
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%% MPCC-Call %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %% augment state and inputs by shifting previus optimal solution
        [x,u] = augState(x,u,x0,MPC_vars,ModelParams,tl);
        %%  formulate MPCC problem and solve it
        if QP_iter == 0
            [x, u, b, exitflag,info] = optimizer(TrackMPC,MPC_vars,ModelParams, n_cars, Y, x, u, x0, uprev);
            qpTime_log(i) = info.QPtime;
        elseif QP_iter == 1
            % doing multiple "SQP" steps
            for k = 1:2
                [x, u, b, exitflag,info] = optimizer(TrackMPC,MPC_vars,ModelParams, n_cars, Y, x, u, x0, uprev);
                qpTime_log(i) = qpTime_log(i) + info.QPtime;
            end
        elseif QP_iter == 2
            % doing multiple damped "SQP" steps
            for k = 1:2
                Iter_damping = 0.75; % 0 no damping
                [x_up, u_up, b, exitflag,info] = optimizer(TrackMPC,MPC_vars,ModelParams, n_cars, Y, x, u, x0, uprev);
                x = Iter_damping*x + (1-Iter_damping)*x_up;
                u = Iter_damping*u + (1-Iter_damping)*u_up;
                qpTime_log(i) = qpTime_log(i) + info.QPtime;
            end
        else
            error('invalid QP_iter value')
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%% simulate system %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        x0 = SimTimeStep(x(:,1),u(:,1),Ts,ModelParams)';
        x0 = unWrapX0(x0);
        [ theta, last_closestIdx] = findTheta(x0([1,3]),track.cline,traj.ppx.breaks,trackWidth,last_closestIdx);
        x0(ModelParams.stateindex_theta) = theta;
        uprev = u(:,1);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%% plotting and logging %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        if plotOn == 1
            PlotPrediction(x,u,b,Y,track,TestTrack,traj,MPC_vars,ModelParams, Xobs)
        end
        
        % log predictions and time
        X_log(:,i) = reshape(x,(N+1)*7,1);
        U_log(:,i) = reshape(u,(N)*3,1);
        B_log(:,i) = reshape(b,N*4,1);

    %     X_log(1:(N+1)*7,i) = reshape(x,(N+1)*7,1);
    %     U_log(1:N*3,i) = reshape(u,(N)*3,1);
    %     B_log(1:N*4,i) = reshape(b,N*4,1);    
        
        
        if x0(1) > TestTrack.cline(1,end)
            break
        end
    %     if x(7,end) > traj.ppx.breaks(end)
    %         MPC_vars.N = MPC_vars.N - 1;
    %         N = MPC_vars.N;
    %     end
        
    end

    %PlotLog( X_log,U_log,Y,track,TestTrack,simN,Ts)

    %PlotSimu( X_log,U_log,Y,track,TestTrack,Ts, Xobs)

    totalSteps = size(X_log(:, X_log(1,:)~=0), 2);
    extend = 1;
    U_log(1:3, totalSteps+1:totalSteps+extend) = repmat([0;1000; U_log(3, totalSteps)], 1, extend);
    input = @(t) U_log(1:2, floor(t/Ts)+1);
    t = 0:0.01:(totalSteps+extend)*Ts;
    U = zeros(numel(t), 2);

    for i = 1:numel(t)
        U(i, :) = input(t(i))';
    end
    %U = U_log(1:2, 1:totalSteps+extend)';

    [Y, ~] = forwardIntegrateControlInput2(U);


    if plotOn == 1
        figure(10)
        clf
        hold on
        set(gcf,'Position',[1000,10,800,800])
        plot(track.br(1,:),track.br(2,:),'r')
        plot(track.bl(1,:),track.bl(2,:),'r')
        plot(TestTrack.br(1,:),TestTrack.br(2,:),'k')
        plot(TestTrack.bl(1,:),TestTrack.bl(2,:),'k')
        plot(Y(:,1), Y(:,3), '-ob')
        for i = 1:numel(Xobs)
            thisOb = Xobs{i};
            plot(thisOb(:,1), thisOb(:,2), '-b');
            plot(thisOb([end, 1], 1), thisOb([end, 1], 2), '-b')
        end
    end

    info = getTrajectoryInfo(Y(:,[1,3]), U, Xobs)

    %%% Generating Stats
    %a = 1;
    %for i=1:simN-1
        %if X_log(ModelParams.stateindex_theta,i+1) - X_log(ModelParams.stateindex_theta,i) < -0.9*tl
            %LapStart(a) = i;
            %a = a+1;
        %end
    %end

    %if length(LapStart) > 1
        %LapTime = diff(LapStart)*Ts;
    %else
        %LapTime = NaN;
    %end

    %disp('------------------------------------')
    %disp(['Lap Time(s): ',num2str(LapTime)])
    %disp('------------------------------------')
    %disp(['Mean Computation Time: ',num2str(mean(qpTime_log))])
    %disp(['Max Computation Time: ',num2str(max(qpTime_log))])
    %disp(['Min Computation Time: ',num2str(min(qpTime_log))])
%disp('------------------------------------')
