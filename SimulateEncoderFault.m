function [ Dr, Dl ] = SimulateEncoderFault(Dr, Dl, i, dt)
%SimulateEncoderFault simulates an encoder fault


if (i > 40/dt) && (i <= 45/dt)
    Dr = Dr + 0.03/50*(i-40/dt);
end
if (i > 45/dt) && (i <= 50/dt)
    Dr = Dr + 0.03;
end
if (i > 50/dt) && (i < 55/dt)
    Dr = Dr + 0.03/50*(55/dt-i);
end


% if (i > 40/dt) && (i <= 50/dt)
%     Dr = Dr + 0.05;
% end


end

