function n_d = noise_dist(t)

if t < 10
    n_d = 0.06*randn;
else
    n_d = 0.06*randn;
%     n_d = 8+0.06*randn;
%     n_d = 5*(t-10)+0.06*randn;
end
    