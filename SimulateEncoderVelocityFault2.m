function [ vRoff, vLoff ] = SimulateEncoderVelocityFault2(i,dt)
%SimulateEncoderFault simulates an encoder fault


vRoff = 0;
vLoff = 0;

if (i > 20/dt) && (i <= 30/dt)
    vRoff = 0.3;
    vLoff = 0.3;
end

end
