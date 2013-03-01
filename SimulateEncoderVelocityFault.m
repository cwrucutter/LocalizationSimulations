function [ vRoff, vLoff ] = SimulateEncoderVelocityFault(i,dt)
%SimulateEncoderFault simulates an encoder fault


vRoff = 0;
vLoff = 0;

if (i > 67/dt) && (i <= 87/dt)
    vRoff = 0.1;
end

end

