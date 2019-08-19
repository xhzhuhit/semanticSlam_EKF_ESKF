function [p]=initialize_quat_covariance(q,rotation_variance_vector)
 q0 = q(1);q1 = q(2);q2 = q(3);q3 = q(4);
    if (q0 < 0)
        q0 =  - q0;
        q1 =  - q1;
        q2 =  - q2;
        q3 =  - q3;
   end
     delta = 2.0 * acos(q0);
    if (abs(delta) > 1e-6)
        scaler = (delta / sin(delta * 0.5));
    else
        scaler = 2.0;
   end
     rotX = scaler * q1;
     rotY = scaler * q2;
     rotZ = scaler * q3;

     t2 = rotX * rotX;
     t4 = rotY * rotY;
     t5 = rotZ * rotZ;
     t6 = t2 + t4 + t5;
    if (t6 > 1e-9)
         t7 = sqrt(t6);
         t8 = t7 * 0.5;
         t3 = sin(t8);
         t9 = t3 * t3;
         t10 = 1.0 / t6;
         t11 = 1.0 / sqrt(t6);
         t12 = cos(t8);
         t13 = 1.0 / (t6^(1.5));
         t14 = t3 * t11;
         t15 = rotX * rotY * t3 * t13;
         t16 = rotX * rotZ * t3 * t13;
         t17 = rotY * rotZ * t3 * t13;
         t18 = t2 * t10 * t12 * 0.5;
         t27 = t2 * t3 * t13;
         t19 = t14 + t18 - t27;
         t23 = rotX * rotY * t10 * t12 * 0.5;
         t28 = t15 - t23;
         t20 = rotY * rotation_variance_vector(2) * t3 * t11 * t28 * 0.5;
         t25 = rotX * rotZ * t10 * t12 * 0.5;
         t31 = t16 - t25;
         t21 = rotZ * rotation_variance_vector(3) * t3 * t11 * t31 * 0.5;
         t22 = t20 + t21 - rotX * rotation_variance_vector(1) * t3 * t11 * t19 * 0.5;
         t24 = t15 - t23;
         t26 = t16 - t25;
         t29 = t4 * t10 * t12 * 0.5;
         t34 = t3 * t4 * t13;
         t30 = t14 + t29 - t34;
         t32 = t5 * t10 * t12 * 0.5;
         t40 = t3 * t5 * t13;
         t33 = t14 + t32 - t40;
         t36 = rotY * rotZ * t10 * t12 * 0.5;
         t39 = t17 - t36;
         t35 = rotZ * rotation_variance_vector(3) * t3 * t11 * t39 * 0.5;
         t37 = t15 - t23;
         t38 = t17 - t36;
         t41 = rotation_variance_vector(1) * (t15 - t23) * (t16 - t25);
         t42 = t41 - rotation_variance_vector(2) * t30 * t39 - rotation_variance_vector(3) * t33 * t39;
         t43 = t16 - t25;
         t44 = t17 - t36;

        p(1,1) = rotation_variance_vector(1) * t2 * t9 * t10 * 0.25 ...
                   + rotation_variance_vector(2) * t4 * t9 * t10 * 0.25 ...
                   + rotation_variance_vector(3) * t5 * t9 * t10 * 0.25;
        p(1,2) = t22;
        p(1,3) = t35 + rotX * rotation_variance_vector(1) * t3 * t11 * (t15 - rotX * rotY * t10 * t12 * 0.5) * 0.5 ...
                   - rotY * rotation_variance_vector(2) * t3 * t11 * t30 * 0.5;
        p(1,4) = rotX * rotation_variance_vector(1) * t3 * t11 * (t16 - rotX * rotZ * t10 * t12 * 0.5) * 0.5 ...
                   + rotY * rotation_variance_vector(2) * t3 * t11 * (t17 - rotY * rotZ * t10 * t12 * 0.5) * 0.5 - rotZ * rotation_variance_vector(3) * t3 * t11 * t33 * 0.5;
        p(2,1) = t22;
        p(2,2) = rotation_variance_vector(1) * (t19 * t19) ...
                   + rotation_variance_vector(2) * (t24 * t24) ...
                   + rotation_variance_vector(3) * (t26 * t26);
        p(2,3) = rotation_variance_vector(3) * (t16 - t25) * (t17 - rotY * rotZ * t10 * t12 * 0.5) ...
                   - rotation_variance_vector(1) * t19 * t28  ...
                   - rotation_variance_vector(2) * t28 * t30;
        p(2,4) = rotation_variance_vector(2) * (t15 - t23) * (t17 - rotY * rotZ * t10 * t12 * 0.5) ...
                   - rotation_variance_vector(1) * t19 * t31  ...
                   - rotation_variance_vector(3) * t31 * t33;
        p(3,1) = t35 - rotY * rotation_variance_vector(2) * t3 * t11 * t30 * 0.5 ...
                   + rotX * rotation_variance_vector(1) * t3 * t11 * (t15 - t23) * 0.5;
        p(3,2) = rotation_variance_vector(3) * (t16 - t25) * (t17 - t36) ...
                   - rotation_variance_vector(1) * t19 * t28  ...
                   - rotation_variance_vector(2) * t28 * t30;
        p(3,3) = rotation_variance_vector(2) * (t30 * t30) ...
                  + rotation_variance_vector(1) * (t37 * t37) ...
                   + rotation_variance_vector(3) * (t38 * t38);
        p(3,4) = t42;
        p(4,1) = rotZ * rotation_variance_vector(3) * t3 * t11 * t33 * ( - 0.5) ...
                   + rotX * rotation_variance_vector(1) * t3 * t11 * (t16 - t25) * 0.5 ...
                   + rotY * rotation_variance_vector(2) * t3 * t11 * (t17 - t36) * 0.5;
        p(4,2) = rotation_variance_vector(2) * (t15 - t23) * (t17 - t36) ...
                   - rotation_variance_vector(1) * t19 * t31 ...
                   - rotation_variance_vector(3) * t31 * t33;
        p(4,3) = t42;
        p(4,4) = rotation_variance_vector(3) * (t33 * t33) ...
                   + rotation_variance_vector(1) * (t43 * t43) ...
                   + rotation_variance_vector(2) * (t44 * t44);
    else
        p(1,1) = 0.0;
        p(1,2) = 0.0;
        p(1,3) = 0.0;
        p(1,4) = 0.0;
        p(2,1) = 0.0;
        p(2,2) = 0.25 * rotation_variance_vector(1);
        p(2,3) = 0.0;
        p(2,4) = 0.0;
        p(3,1) = 0.0;
        p(3,2) = 0.0;
        p(3,3) = 0.25 * rotation_variance_vector(2);
        p(3,4) = 0.0;
        p(4,1) = 0.0;
        p(4,2) = 0.0;
        p(4,3) = 0.0;
        p(4,4) = 0.25 * rotation_variance_vector(3);
    end
end