function phaseout = doublePendulumContinuous(input)

% input
% input.phase(phasenumber).state
% input.phase(phasenumber).control
% input.phase(phasenumber).time
% input.phase(phasenumber).parameter
%
% input.auxdata = auxiliary information
%
% output
% phaseout(phasenumber).dynamics
% phaseout(phasenumber).path
% phaseout(phasenumber).integrand

[N, ny] = size(input.phase.state);

phaseout.dynamics = zeros(N, ny);

for it = 1:N
    phaseout.dynamics(it, :) = dynamics(input.phase.state(it, :)', ...
                                        input.phase.control(it, :)', ...
                                        input.auxdata)';
end

end

function xd = dynamics(x, tau, a)

    % TODO there are a lot of constants here that can be moved to auxdata.

    theta1 = x(1);
    theta2 = x(2);
    theta1dot = x(3);
    theta2dot = x(4);

    c1 = cos(theta1);
    c2 = cos(theta2);
    s2 = sin(theta2);
    z1 = a.m2 * a.L1 * a.L2 * c2;
    M12 = a.m2 * a.L2^2 + z1;

    M = [a.m1 * a.L1^2 + a.m2 * (a.L1^2 + a.L2^2) + 2 * z1, M12;
         M12,                                               a.m2 * a.L2^2];
    V = a.m2 * a.L1 * a.L2 * s2 * [theta2dot * (-2 * theta1dot - theta2dot);
                                   theta1dot^2];
    G = a.g * [a.L1 * c1 * (a.m1 + a.m2) + a.m2 * a.L2;
             a.m2 * a.L2 * cos(theta1 + theta2)];

    xd = [theta1dot;
          theta2dot;
          M\(tau - (V + G))];

end
