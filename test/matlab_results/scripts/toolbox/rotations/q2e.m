function eul = q2e(q)
%Q2E Convert quaternion to Euler angles
%   EUL = Q2E(Q) converts a unit quaternion rotation into the corresponding 
%   Euler angles. The input, Q, is an N-by-4 matrix containing N quaternions. 
%   Each quaternion represents a 3D rotation and is of the form q = [w x y z], 
%   with a scalar number as the first value. Each element of Q must be a real number.
%   The output, EUL, is an N-by-3 array of Euler rotation angles with each 
%   row representing one Euler angle set. Rotation angles are in radians.
%
%   The rotation sequence is 'ZYX', which is written as roll
%   pitch yaw angles expressed in reference frame.


if size(q,1) == 4  % change to row vector if quaternion written by a column vector 
    quat = q';
else
    quat = q;
end

qw = quat(:,1);
qx = quat(:,2);
qy = quat(:,3);
qz = quat(:,4);

norm = sqrt(qw.*qw + qx.*qx + qy.*qy + qz.*qz);

if norm ~= 1 
    qw = qw./norm;
    qx = qx./norm;
    qy = qy./norm;
    qz = qz./norm;
end

aSinInput = -2*(qx.*qz-qw.*qy);
aSinInput(aSinInput > 1) = 1;
aSinInput(aSinInput < -1) = -1;

yaw = atan2( 2*(qx.*qy+qw.*qz), qw.^2 + qx.^2 - qy.^2 - qz.^2 );
pitch = asin( aSinInput );
roll = atan2( 2*(qy.*qz+qw.*qx), qw.^2 - qx.^2 - qy.^2 + qz.^2 );

eul = [roll pitch yaw]'; % output with column vector

end