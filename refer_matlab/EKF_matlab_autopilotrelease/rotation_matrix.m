function [R]=rotation_matrix(q1,q2,q3,q4)

     q3q3 = q3 * q3;
     q3q4 = q3 * q4;
     q2q2 = q2 * q2;
      q2q3 = q2 * q3;
      q2q4 = q2 * q4;
      q1q2 = q1 * q2;
      q1q3 = q1 * q3;
      q1q4 = q1 * q4;
      q4q4 = q4 * q4;

    ax = 1.0-2.0*(q3q3 + q4q4);
    ay = 2.0*(q2q3 - q1q4);
    az = 2.0*(q2q4 + q1q3);
    bx = 2.0*(q2q3 + q1q4);
    by = 1.0-2.0*(q2q2 + q4q4);
    bz = 2.0*(q3q4 - q1q2);
    cx = 2.0*(q2q4 - q1q3);
    cy = 2.0*(q3q4 + q1q2);
    cz = 1.0-2.0*(q2q2 + q3q3);
    R = [ax ay az;
        bx by bz;
        cx cy cz];
end
