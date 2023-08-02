function p = beamSensorModel_new(z, measRange, param)
%{
    Copyright (C) 2021  Yang Xu (xuyang94@zju.edu.cn)
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
%}
    %% compute the density
    if z>=0 && z<= param.maxrange   
        p_hit = exp(-0.5 * ((z - measRange)/param.dev)^2) / (sqrt(2*pi) * param.dev);
    else
        p_hit = 0;
    end
    if z>=0 && z<param.maxrange
        p_rand = 1/param.maxrange;
    else
        p_rand = 0;
    end
    if z == param.maxrange
        p_max = 1;
    else
        p_max = 0;
    end
    if z>=0 && z<= measRange
        p_short = param.lambda_short * exp(-param.lambda_short * z);
    else
        p_short = 0;
    end
%    simple  linear combination
    p = param.z_hit * p_hit + param.z_short * p_short + param.z_max * p_max + param.z_rand * p_rand;

end