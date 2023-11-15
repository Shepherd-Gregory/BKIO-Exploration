%{  
    Copyright (C) 2023  Yang Xu
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details. 
%}

clear
clc
%%%%% %%%%%%%% %%%%%
per_infer_time = [];
per_end_time = [];

%%%%% Params %%%%%
USE_GREEDY = 0;
USE_GP = ~USE_GREEDY;

param.Nsam = 200;
param.loopnum = 10;
param.Ncandi = 100; 
param.Nepoch = 30;
%%%%% %%%%%%%% %%%%%
% plot or not
USE_PLOT = 1;
RECORD = 1;
% Initializing the robot's location.[x, y, theta, step]
% map size
param.M = 70;
param.N = 120;
M = param.M;
N = param.N;
MAP_TYPE = 1; % 1: str, 0: unstr
[true_map, obs_list] = gen_map(M,N,MAP_TYPE); % test obs ok
if USE_PLOT
    figure;
    imagesc(true_map);
    axis xy
end

% initialize map with 0.5 occupancy
m = 0.5 * ones(M,N);

% initialize log-odds mapping
L0 = log(m./(ones(M,N)-m));
L = L0;
% initialize map entropy
ENT_M = [];

% sensor config
sensor.minAngle = -1.5; % rad 1.5
sensor.maxAngle = 1.5; % rad 1.5
sensor.resol = 0.05; % rad 0.1
sensor.FOV = sensor.maxAngle - sensor.minAngle;
sensor.maxRange = 30; % 20
sensor.scanAngle = linspace(sensor.minAngle, sensor.maxAngle, sensor.FOV/sensor.resol);

% params for mixture sensor model
param.resol = 1;
param.alpha = 1; % wall
param.z_hit = 0.7;
param.z_max = 0.1;
param.z_rand = 0.1;
param.z_short = 0.1;
param.lambda_short = 0.2;
param.res_m = 10;
param.res_z = 0.5;
param.maxrange = sensor.maxRange;
% sensor range noise
param.dev = 0.1 / param.resol; 
% initialize map belief
B = ones(param.res_m, M*N);

% initialize pose 
x = zeros(4,1000);
x(1:4,1) = [6;6;0;0]; % initial pose
if USE_PLOT
    show_FOV(x, sensor);
end
%% main thread
step = 1;

extend_list = []; % map cells in FOV
[extend_list, B, L, m] = raycasting_crm(x, sensor, obs_list, extend_list, B, L, param, step);

%% main loop
binMap = ogm2bin(m);
[MI, ENT0] = cal_MI(m);
ENT_M = [ENT_M; ENT0];

cover_rate = [0];
temp_cover_rate = compute_cover(m);
cover_rate = [cover_rate; temp_cover_rate];
opti_start_time = tic;
MI_best_for_each_iteration = []; % MI of the selected action
action_track = x(1:3,1)';
action_hist = x(1:3,1)'; % selected action at each step
all_path_points = x(1:2,1)';
InfoThres = 2; % information threshold


if RECORD
    fig=figure;
end
iteration = 0;

while ~isempty(action_track) && iteration < param.loopnum 
    iteration = iteration+1;
    disp('iteration = ')
    disp(iteration)
    %% make gifs
    if RECORD
        imagesc(MI,[0,1]); axis xy; colormap jet; colorbar
        hold on;
        plot(x(2, step), x(1, step), 'ro', 'LineWidth', 3); % indicate robot location
        plot([x(2, step), x(2, step) + 5*sin(x(3, step))], [x(1, step), x(1, step) + 5*cos(x(3, step))], 'r','LineWidth',2);
        pause(0.1);
        % record gif
        frame=getframe(fig);
        im{iteration} = frame2im(frame);
        [A,tutu] = rgb2ind(im{iteration},256);
        if iteration == 1
            imwrite(A,tutu,'OGMI-str.gif','gif','LoopCount',Inf,'DelayTime',0.05);
        else
    %         if mod(iteration,2)==0
            imwrite(A,tutu,'OGMI-str.gif','gif','WriteMode','append','DelayTime',0.05);
        end
    end
    tic
    %% sample the candidates actions
    candidate_list = sample_action_m(extend_list,m, x(:,step), param);
    % plot the candidate actions
    if USE_PLOT
        for i = 1:length(candidate_list(:,1))
            candi = candidate_list(i,:);
            scatter(candi(2),candi(1),20,'w','filled')
            plot([candi(2), candi(2) + 2 * sin(candi(3))], [candi(1), candi(1) + 2 * cos(candi(3))]);
        end
    end
 %% best action selection
    step = step + 1;
%     MI_GT = action_CRMI(candidate_list, B, L, ENT0, sensor, param);  % naive greedy method
    MI_GT = []; % 
    
    if USE_GREEDY
        tic
        MI_best = action_CRMI(candidate_list, B, L, ENT0, sensor, param);
        action_best = candidate_list;
    elseif USE_GP
        num_candies = length(candidate_list(:,1));
        sample_number = min(num_candies,param.Nsam);
        random_index = randsample(num_candies,sample_number);
        % construct traning dataset input
        X_init = zeros(sample_number,3);
        
        for i = 1:sample_number
            X_init(i,1) = candidate_list(random_index(i),1);
            X_init(i,2) = candidate_list(random_index(i),2);
            X_init(i,3) = candidate_list(random_index(i),3);
        end
        X_train = X_init;
        % compute traning dataset output 
        Y_train = action_CRMI(X_train, B, L, ENT0, sensor, param);
    
        % GPBO
%         [action_best,MI_best,infer_time] = bayesian_optimization_crm(candidate_list, X_train,Y_train, B, L, ENT0, sensor, param, MI_GT);
        % BKIO
        [action_best,MI_best,infer_time] = bki_optimization_crm(candidate_list, X_train,Y_train, B, L, ENT0, sensor, param, MI_GT);

    end
    per_infer_time = [per_infer_time; infer_time];
    
    if max(MI_best) > InfoThres
        % greedy selection
        [MI_best_value, max_MI_index] = max(MI_best);
        max_candidate_draw = action_best(max_MI_index,:);
        MI_best_for_each_iteration = [MI_best_for_each_iteration;MI_best_value];
        action_track = [action_track; max_candidate_draw];
    else
        max_candidate_draw = action_track(end,:);
        action_track(end,:) = [];
        MI_best_for_each_iteration = [MI_best_for_each_iteration;InfoThres];
    end
    action_hist = [action_hist; max_candidate_draw];
    % plot the best action
    if USE_PLOT
        hold on
        scatter(max_candidate_draw(2),max_candidate_draw(1),20,'yellow','filled');
        plot([max_candidate_draw(2), max_candidate_draw(2) + 5 * sin(max_candidate_draw(3))], ...
            [max_candidate_draw(1), max_candidate_draw(1) + 5 * cos(max_candidate_draw(3))],'y','linewidth',2);
    end
    %% move and mapping
    % start pose
    x_or = x(1, step - 1);
    y_or = x(2, step - 1);
    yaw_or = x(3, step - 1);
    % target pose
    x_dest = max_candidate_draw(1);
    y_dest = max_candidate_draw(2);
    yaw_dest = max_candidate_draw(3);
    % a star
%     traverse_array = a_star(true_map,[x_or, y_or], [x_dest, y_dest]);
    % use robotics toolbox
    testmap = binaryOccupancyMap(binMap);
    planner = plannerAStarGrid(testmap);
    traverse_array = plan(planner,[x_or, y_or], [x_dest, y_dest]);
    all_path_points = [all_path_points; traverse_array];
    for tp = 1:length(traverse_array(:,1))
        travel = traverse_array(tp,:);
        x(1, step) = travel(1);
        x(2, step) = travel(2);
        x(3, step) = yaw_dest;
        [extend_list, B, L, m] = raycasting_crm(x, sensor, obs_list, extend_list, B, L, param, step);
    end
    % update map entropy 
    binMap = ogm2bin(m);
    [MI, ENT0] = cal_MI(m);
    ENT_M = [ENT_M; ENT0];
    % plot updated map
    if USE_PLOT
        figure
        imagesc(L,[0,1]);
        axis xy
        colormap(gray); colorbar
        hold on
        scatter(extend_list(:,2),extend_list(:,1),10,'r'); 
    end

    %% compute coverage
    temp_cover_rate = compute_cover(m);
    cover_rate = [cover_rate; temp_cover_rate];
    expl_end_time = toc;
    per_end_time = [per_end_time; expl_end_time];
end
t = sum(per_end_time);
disp('End of this exploration...')
if USE_PLOT
    %% plot OGM
    imagesc(m);
    axis xy
    %% plot all traversed points 
    hold on
    for i = 1:length(all_path_points(:,1))
       candi = all_path_points(i,:);
       scatter(candi(2),candi(1),20,'g','s','filled')
    end
    hold on
    plot(all_path_points(1,2),all_path_points(1,1),'ys');
    plot_track_points(action_hist);
    plot(6,6,'ys','markersize',24,'linewidth',3)
end
%% compute travel cost
dist_cost = compute_dist(all_path_points);
