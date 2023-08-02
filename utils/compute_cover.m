function r = compute_cover(P)
    [M,N] = size(P);
    r = 1 - length(find(P==0.5))/(M*N);
end