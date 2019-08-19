function [q] = from_axis_angle(vec)
    theta = norm(vec);
    if (theta < 1e-10) 
            q(1) = 1.0;
            q(2) = 0;
            q(3) = 0;
            q(4) = 0;
    else
        vec = vec / theta;
        magnitude = sin(theta / 2.0);
        q(1) = cos(theta / 2.0);
        q(2) = vec(1) * magnitude;
        q(3) = vec(2) * magnitude;
        q(4) = vec(3) * magnitude;
    end
end