function [ out ] = CoerceAngle( in )
%COERCEANGLE Coerces an angle to -pi to pi

out = mod(in,2*pi);
out(out>pi) = out(out>pi)-2*pi;

end

