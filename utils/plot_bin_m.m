function plot_bin_m(L)
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
L2show = L;
L2show(L2show > 0) = 1;
L2show(L2show < 0) = 0;
L2show(L2show == 0) = 0.5;
figure
imagesc(1 - L2show)
axis xy
colormap(gray)
colorbar
grid on
end

