function [ R ] = RotMatrix( tht )
%ROTMATRIX Returns the rotation matrix for theta
% R: 2dimensional counterclockwise rotation about the origin 
%    Usage: Pnew = R*Pold, where R is 2x2 and Pold is [x;y]

R = [cos(tht) -sin(tht);sin(tht) cos(tht)];

end

