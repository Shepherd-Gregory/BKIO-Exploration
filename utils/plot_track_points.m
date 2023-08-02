function plot_track_points(action_track)
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
last_action = action_track(1,:);
for i = 1:length(action_track(:,1))
    action = action_track(i,:);
    hold on
    scatter(action(2), action(1),30,'r','s','filled');
    plot([action(2), action(2) + 5*sin(action(3))], [action(1), action(1) + 5*cos(action(3))], 'r','LineWidth',2);
    last_action = action;
end
end

