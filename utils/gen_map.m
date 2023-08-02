function [true_map, obs_list] = gen_map(M,N,map_type)
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
USE_STRUCT = map_type;
obs_list = [];
%% map boundaries
true_map = zeros(M, N);
true_map(1:M,1) = 1;
true_map(1:M,N) = 1;
true_map(1,1:N) = 1;
true_map(M,1:N) = 1;

if USE_STRUCT
%% obstacles --- simple walls
    % obstacle list
    for i=1:M
        for j=1:N
            % add walls
            if (i>=1 && i<=40 && j>=11 &&j<=13) || (i>=35 && i<=37 && j>=60 && j<=100) || (i>=35 && i<=55 && j>=60 && j<=62) || (j >= 98 && j<=100 && i>=35 && i<=55)...
                    || (i>=21 && i<=60 && j>=33 && j<=35) || (i>=21 && i<=23 && j>=33 && j<=50)...
                    || (i>=21 && i<=23 && j>=61 && j<=120) ||(i>=1 && i<=70 && j==1) ...
                    || (i>=1 && i<=70 && j==120) || (i==70 && j>=1 && j<=120) || (i==1 && j>=1 && j<=120)
                obs_list = [obs_list;i,j];
                true_map(i,j) = 1;
            end
        end
    end
end

