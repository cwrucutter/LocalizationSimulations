function [ new ] = resample_low_variance(particles)
%RESAMPLE_LOW_VARIANCE Resamples the particles according to their weights
%   particles: Particle input [x; y; theta; weight]

w = particles(4,:);
[nrows M] = size(particles);
new = zeros(4,M);
r = 1/M*rand(1);
c = w(1);
i = 1;
for m=1:M
    U = r + (m-1)*1/M;
    while U > c
        i = i + 1;
        if i > M
            i = 1;
        end
        c = c + w(i);
    end
    new(:,m) = particles(:,i);
end


end

