function [data]=euler2R(euler)
%euler: roll, float pitch, float yaw
roll = euler(1);pitch = euler(2);yaw = euler(3);
		 cp = cos(pitch);
		 sp = sin(pitch);
		 sr = sin(roll);
		 cr = cos(roll);
		 sy = sin(yaw);
		 cy = cos(yaw);

		data(1,1) = cp * cy;
		data(1,2) = (sr * sp * cy) - (cr * sy);
		data(1,3) = (cr * sp * cy) + (sr * sy);
		data(2,1) = cp * sy;
		data(2,2) = (sr * sp * sy) + (cr * cy);
		data(2,3) = (cr * sp * sy) - (sr * cy);
		data(3,1) = -sp;
		data(3,2) = sr * cp;
		data(3,3) = cr * cp;
end