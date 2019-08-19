function [euler]=q2euler(q)

%     glvs
%     global q;
    euler=[ atan2((2.0 * (q(1) * q(2) + q(3) * q(4))), 1.0 - 2.0 * (q(2) * q(2) + q(3) * q(3)));
            asin(2.0 * (q(1) * q(3) - q(4) * q(2)));
            atan2(2.0 * (q(1) * q(4) + q(2) * q(3)), 1.0 - 2.0 * (q(3) * q(3) + q(4) * q(4)))];

end