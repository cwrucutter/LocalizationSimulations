function [ sample ] = sample_normal_distribution(var)
%SAMPLE_NORMAL_DISTRIBUTION Generates a random sample from a zero-centered
%distribution with specified var
%   var: variance

sigma = sqrt(var);
sample = sigma*randn(1);

end

