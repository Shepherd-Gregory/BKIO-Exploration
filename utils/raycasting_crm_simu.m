function [L, B] = raycasting_CRM_simu_test(x, sensor, B, L, param)
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
% Usage: simu the raycasting at the candidate point
rmax = sensor.maxRange;
FOV = sensor.FOV;
scanAngle = sensor.scanAngle;
alpha = param.alpha;
cur_extend_list = [];
obs_list_virtual = [];
m_range = rmax * ones(length(scanAngle),1);
log_free = 0.5; 
log_occ = 1; 
log_min = -2; 

vec_com = (0.05:0.01*param.res_m:0.95);
exp_m = compute_expm(B,param); 
free = [];
occ = [];
z_meas = [];
ix_robot = ceil(x(1));
iy_robot = ceil(x(2));

% CRMI init
m_bar = exp_m;
theta_m = x(3);
for x_virtual = 1:param.M
    for y_virtual = 1:param.N
        if exp_m(x_virtual, y_virtual) > 0.5 
            obs_list_virtual = [obs_list_virtual; [x_virtual, y_virtual]];
        end
    end
end

for i = 1:length(scanAngle)
    theta = scanAngle(i);
    break_flag = 0;    
    
    x_v = rmax * cos(theta);
    y_v = rmax * sin(theta);
    x_m = x_v * cos(theta_m) - y_v * sin(theta_m) + x(1);
    y_m = x_v * sin(theta_m) + y_v * cos(theta_m) + x(2);
    x_m = min(max(x_m,1),param.M);
    y_m = min(max(y_m,1), param.N);
    ray_array = bresenham(x(1), x(2), x_m, y_m);
    
    if isempty(ray_array) == 0
        break
    end
    occ_val = [];
    num_cell = length(ray_array(:,1));
    for ii = 1:num_cell
        occ_val = [occ_val; exp_m(ray_array(ii,1),ray_array(ii,2))];
    end
    
    if sum(abs(occ_val-0.5)>=0.4)/length(occ_val) >= 0.95
        break
    end
end
for i = 1:length(scanAngle)
    theta = scanAngle(i);
    
    %% CRM mapping
    [~, ~, z_ray, z_occ, z_free, meas_range] = find_nearest_obs(obs_list_virtual, x, theta, param);
    V_ray = z_ray;
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
            pz = param.resol/rmax;
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
                prod = prod * (1- exp_m(V_ray(j-1)) / param.resol);
            end
            f = prod * exp_m(V_ray(j));
            [ix,iy] = ind2sub([param.M,param.N],V_ray(j));
            dist = (1/param.resol)*norm([ix_robot,iy_robot] - [ix,iy]);
            pz = beamSensorModel_new(dist, meas_range/param.resol, param);
            pc = pz;
            vec_scm(j) = pc;
            eta = eta + pc;
        end
         % normalization
        vec_scm = vec_scm / eta;
    end
    
    %% compute alpha and beta
    for j = 1:length(V_ray)
        beta(j) = 1 + exp_m(V_ray(j))/(1 - exp_m(V_ray(j)))*sum(vec_scm(j + 1:length(V_ray))) - vec_scm(j);
        gamma(j) = (1  - beta(j))/exp_m(V_ray(j));
    end
    
    % compute the map belief
    for j = 1:size(V_ray)
        B(:,V_ray(j)) = B(:,V_ray(j)) .* (gamma(j) * vec_com' + beta(j) * ones(10,1));
        % normalization
        B(:,V_ray(j)) = B(:,V_ray(j)) *10 / sum(B(:,V_ray(j)));
    end

end
end