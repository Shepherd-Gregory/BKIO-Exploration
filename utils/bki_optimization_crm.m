function [action_best,MI_best,infer_time] = bki_optimization_crm(candidate_list,X_train,Y_train, B, L, ENT0, sensor, param, MI_update_list)
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
% Gaussian process using Matern kernel and params
phi = [1;2.5]; % kernel params: length_scale=1.0, std=2.5
beta = 1; % trade off "exploration" and "exploitation"
n_epoch = param.Nepoch; 

% set sample numbers
num_candies = length(candidate_list(:,1));
action_best = [];
MI_best = [];
per_infer_time = [];

%% traning the GP model
for i = 1:n_epoch
    tic
    Ks = matern_kernel(X_train,candidate_list);
    ybar = Ks * Y_train; 
    kbar = sum(Ks,2); 
    mu = ybar./kbar; % exp of posterior distribution
    mu(mu<0)=0;
    var = 100 ./kbar;  % var of posterior distribution
    std = var.^0.5;
    %  cal UCB function
    acquisition = beta * std(1:num_candies) + mu(1:num_candies);
    % sort
    [max_acq, candidate_with_max_acq_index] = max(acquisition);
    candidate_with_max_acq = candidate_list(candidate_with_max_acq_index,:);
    tmp_infer_time = toc;
    per_infer_time = [per_infer_time, tmp_infer_time];

    flag_not_in = true;
    % if max_acq already in D
    if ismember(candidate_with_max_acq,X_train,'rows')
        flag_not_in = false;
        action_best = [action_best; candidate_with_max_acq];
        idx = find(ismember(X_train, candidate_with_max_acq,'rows'));
        MI_best = [MI_best; Y_train(idx)];
    end
    % add max_acq to D if not in D
    if flag_not_in
        % explicitly eval candidate_with_max_acq CRMI 
		MI_best_epoch = action_CRMI(candidate_with_max_acq, B, L, ENT0, sensor, param);
        action_best = [action_best; candidate_with_max_acq];
        MI_best = [MI_best; MI_best_epoch];
        X_train = [X_train; candidate_with_max_acq];
        Y_train = [Y_train; MI_best_epoch];
    end
end
infer_time = mean(per_infer_time);
end

