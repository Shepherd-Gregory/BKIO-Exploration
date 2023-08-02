function show_FOV(x,sensor)
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
% This FUNC is for showing the FOV of the robot.
rmax = sensor.maxRange;
FOV = sensor.FOV;

x_max1_v = rmax * cos(FOV/2); % vehicle frame
y_max1_v = rmax * sin(FOV/2);
x_max1_m = x_max1_v * cos(x(3)) - y_max1_v * sin(x(3)) + x(1); % map frame
y_max1_m = x_max1_v * sin(x(3)) + y_max1_v * cos(x(3)) + x(2);
x_max2_v = rmax * cos(FOV/2); % vehicle frame
y_max2_v = rmax * sin(-FOV/2);
x_max2_m = x_max2_v * cos(x(3)) - y_max2_v * sin(x(3)) + x(1); % map frame
y_max2_m = x_max2_v * sin(x(3)) + y_max2_v * cos(x(3)) + x(2);

% figure
line([x(2), y_max1_m],[x(1), x_max1_m],'color','red','linestyle','-','linewidth',2);
hold on
line([x(2), y_max2_m],[x(1), x_max2_m],'color','red','linestyle','-','linewidth',2);

end

