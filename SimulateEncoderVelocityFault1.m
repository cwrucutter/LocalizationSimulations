function [ vRoff, vLoff ] = SimulateEncoderVelocityFault1(i,dt)
%SimulateEncoderFault simulates an encoder fault


vRoff = 0;
vLoff = 0;

if (i > 70/dt) && (i <= 80/dt)
    vRoff = 0.1;
end

% if (i > 20/dt) && (i <= 37/dt)
%     vRoff = 0.2;
% end

end

