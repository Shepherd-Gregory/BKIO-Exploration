function action_list_new = sample_action_m(vcell, m, pose, param)
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
N_candi = param.Ncandi; % gen N_candi points
% find all free cells and their indices
free_ind = [];
for p=1:length(vcell(:,1))
    if m(vcell(p,1), vcell(p,2))<= 0.5
        free_ind = [free_ind; p];
    end
end
% corrected random sample method
num_candies = length(free_ind);
if num_candies > N_candi
    sam_idx = randperm(num_candies);
    action = vcell(sam_idx(1:N_candi)',:);
else
    action = vcell(free_ind,:);
end
%% add multi-directions for generated N_candi points
action_list = [];
if isempty(action)
    hold on
end
for pp=1:length(action(:,1))
    for yaw = 0:0.125*pi:1.875*pi 
        action_list = [action_list; [action(pp,1), action(pp,2), yaw]];
    end
end
%% mix generated N_candi points
randIndex = randperm(size(action_list,1));
action_list_new = action_list(randIndex,:);
end