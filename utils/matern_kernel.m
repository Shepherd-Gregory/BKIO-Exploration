function Ks = matern_kernel(x,x_star)
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

% This function is used for computing matern kernels.

len = 1;
m = length(x_star(:,1));
n = length(x(:,1));
d = zeros(m,n); 
dd = zeros(m,n);
for i = 1:m
    temp_x = x - x_star(i,:);
    for j = 1:n
        d(i,j) = sqrt(3)*sqrt(sum(temp_x(j,:).^2))/len;
    end
end

Ks = (1+d).*exp(-d); 

end