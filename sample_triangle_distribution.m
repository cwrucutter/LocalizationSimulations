function [ sample ] = sample_triangle_distribution(var)
%SAMPLE_NORMAL_DISTRIBUTION Generates a random sample from a zero-centered
%distribution with specified var
%   var: variance

r1 = -var + (var+var)*rand(1);
r2 = -var + (var+var)*rand(1);
sample = sqrt(6)/2*(r1+r2);

end

