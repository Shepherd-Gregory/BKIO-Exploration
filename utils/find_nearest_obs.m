function [ind_x, ind_y, uz_ray, uz_occ, uz_free, meas_range] = find_nearest_obs(obs_list, ro_pose, ray_angle, param)
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
    z_ray = [];
    ind_x = [];
    ind_y = [];
    z_occ = [];
    rx = ro_pose(1);
    ry = ro_pose(2);
    rtheta = ro_pose(3);
    dz = 0.1 * 1/param.resol;
    z = 0.01; %cell，not meter
    while z <= param.maxrange + dz
        x_z = z * cos(ray_angle + rtheta) + rx ;
        y_z = z * sin(ray_angle + rtheta) + ry ;
        ix_z = round(x_z);
        iy_z = round(y_z);

        ind_x = [ind_x; ix_z];
        ind_y = [ind_y; iy_z];
        if iy_z == 0
            hold on
        end
        % if hit the border
        if ix_z >= param.M || ix_z <= 1 || iy_z >= param.N || iy_z <= 1 || ismember([ix_z, iy_z], obs_list,'rows')
            ix_z = min(max(ix_z,1),param.M);
            iy_z = min(max(iy_z,1), param.N);
            ind_z = sub2ind([param.M, param.N],ix_z, iy_z); 
            meas_range = round(z);
            z_ray = [z_ray; ind_z];
            break
        end
        % if not
        ind_z = sub2ind([param.M, param.N],ix_z, iy_z); 
        meas_range = param.maxrange;
        % if not hit the border,check this cell
        z_ray = [z_ray; ind_z];
        z = z + dz;
    end
    
    [uz_ray, mm] = unique(z_ray);
    [~, mm_id] = sort(mm);
    uz_ray = uz_ray(mm_id);
    [uz_occ, mmm] = unique(z_occ);
    [~, mmm] = sort(mmm);
    uz_occ = uz_occ(mmm, :);
    
    uz_free = uz_ray;
    if ~isempty(z_occ)
        for i = 1:length(z_occ)
            temp = find(uz_free == uz_occ(i));
            uz_free(temp) = [];
        end
        
    end
end