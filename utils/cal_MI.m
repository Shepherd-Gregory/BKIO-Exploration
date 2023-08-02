function [I, H] = cal_MI(exp_m)
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
% This a simple way to compute MI, but not an accurate one. For more accurate
% MI computation, please see our formally released codes.

    [M, N] = size(exp_m);
    I = zeros(M,N); % initialize MI
    H = 0;
    for i = 1: M * N
        if exp_m(i) == 0 || exp_m(i) == 1
            I(i) = 0;
        else
            I(i) = -(exp_m(i)*log2(exp_m(i)) + (1-exp_m(i))*log2(1-exp_m(i)));
        end
        H = H + I(i);
    end
end