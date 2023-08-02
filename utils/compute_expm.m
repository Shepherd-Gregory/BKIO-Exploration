function exp_m = compute_expm(B,param)
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
exp_m = 0.5 * ones(param.M, param.N);
vec_com = (0.05:0.01*param.res_m:0.95);
for i=1:param.M
    for j=1:param.N
        cell_index = sub2ind([param.M, param.N],i,j);
        exp_m(i,j) = (1/param.res_m)*sum(vec_com.*B(:,cell_index)'); 
    end
end
end

