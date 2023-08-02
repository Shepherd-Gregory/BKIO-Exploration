function MI_update_list = action_CRMI(candidate_list, B, L, ENT0, sensor, param)
%{
    Copyright (C) 2022  Yang Xu (xuyangxtu@foxmail.com)
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
%}
    MI_update_list = [];
    for i = 1:length(candidate_list(:,1))
        candi = candidate_list(i,:);
        L_copy = L;
        temp_x = candi';
        
        % CRMI
        B_copy = B;
        [L_copy_updated, B_copy_updated] = raycasting_crm_simu(temp_x, sensor, B_copy, L_copy, param);
             
        m_copy_updated = compute_expm(B_copy_updated,param);
        [MI_new, ENT_update] = cal_MI(m_copy_updated);
        MI_update = ENT0 - ENT_update;
        MI_update_list = [MI_update_list; MI_update];
        
    end
end