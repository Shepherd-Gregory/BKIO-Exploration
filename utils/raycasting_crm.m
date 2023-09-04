function [cur_extend_list, bel_m, L, exp_m] = raycasting_CRM_test(x, sensor, obs_list, extend_list, bel_m, L, param, step)
%{  
    Copyright (C) 2021  Yang Xu (xuyangxtu@foxmail.com)
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details. 
%}
% Usage: raycasting and map belief update
rmax = sensor.maxRange;
FOV = sensor.FOV;
scanAngle = sensor.scanAngle;
alpha = param.alpha;
cur_extend_list = [];
m_range = rmax * ones(length(scanAngle),1);
log_free = 0.5; 
log_occ = 1;  
log_min = -4;

free = [];
occ = [];
z_meas = [];

vec_com = (0.05:0.01*param.res_m:0.95);
exp_m = compute_expm(bel_m,param);

ix_robot = ceil(x(1,step));
iy_robot = ceil(x(2,step));
theta_m = x(3,step);

for i = 1:length(scanAngle)
    theta = scanAngle(i);
    
    %% find V_ray
    [~, ~, z_ray, z_occ, z_free, meas_range] = find_nearest_obs(obs_list, x(:,step), theta, param);
    V_ray = z_ray;
    [grid_x,grid_y] = ind2sub([param.M,param.N],V_ray);
    cur_extend_list = [cur_extend_list;[grid_x,grid_y]];
    free = [free; z_free];
    occ = [occ; z_occ];
    z_meas = [z_meas; meas_range];
    eta = 0;
    xi = 0.01;
    vec_scm = zeros(length(V_ray),1);
    
    if meas_range >= rmax % maxrange case
        prod = 1;
        for j = 1:length(V_ray)
            prod = prod * (1- exp_m(V_ray(j)));
            f = prod;
            pz =  param.resol/rmax;
            % compute SCM p(c_k| z{0:k},x{0:k})
            pc = pz * f;
            vec_scm(j) = 1/(length(vec_scm));
            eta = eta + pc;
        end
        eta = eta + 1/rmax;
    else %  hit case
        for j = 1:length(V_ray)
            if j==1
                prod = 1;
            else
                prod = prod * (1- exp_m(V_ray(j-1)));
            end
            f = prod * exp_m(V_ray(j));
            [ix,iy] = ind2sub([param.M,param.N],V_ray(j));
            dist = (1/param.resol)*norm([ix_robot,iy_robot] - [ix,iy]);
            pz = beamSensorModel_new(dist, meas_range/param.resol, param);
            pc = pz * f;
            vec_scm(j) = pc;
            eta = eta + pc;
        end
        % normalization
        vec_scm = vec_scm / eta;
    end
    %% compute alpha and beta: alpha < 0, beta > 1 means cell is free; alpha > 0, 0< beta < 1 means occ
    for j = 1:length(V_ray)
        beta(j) = 1 + exp_m(V_ray(j))/(1 - exp_m(V_ray(j)))*sum(vec_scm(j + 1:length(V_ray))) - vec_scm(j);
        gamma(j) = (1  - beta(j))/exp_m(V_ray(j));
    end
    
    % compute the map belief
    for j = 1:size(V_ray)
        bel_m(:,V_ray(j)) = bel_m(:,V_ray(j)) .* (gamma(j) * vec_com' + beta(j) * ones(10,1));
        % normalization
        bel_m(:,V_ray(j)) = bel_m(:,V_ray(j)) *10 / sum(bel_m(:,V_ray(j)));
    end
    
end
extend_list = unique(extend_list,'rows');
cur_extend_list = unique(cur_extend_list,'rows'); 
exp_m = compute_expm(bel_m,param);
end