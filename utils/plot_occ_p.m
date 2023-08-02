function plot_occ_p(L,m,param)
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
m2show = m;
% OGM
for i = 1:param.M
    for j = 1:param.N
        l_show = L2show(i,j);
        if l_show < 0
            l_show = -5;
        elseif l_show > 0
            l_show = 1;
        else
            l_show = 0;
        end
        m2show(i,j) = exp(l_show)/(1+exp(l_show));
    end
end
figure
imagesc(m2show)
axis xy
colormap(gray)
colorbar
grid on
end

