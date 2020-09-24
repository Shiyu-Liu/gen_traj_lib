function q = e2q(eul)
%E2Q Convert Euler angles to quaternion.
%   Q = E2Q(EUL) converts Euler angles into the corresponding unit quaternion
%   rotation.
%   The input, EUL, is an N-by-3 array containing N Euler rotation angles with each 
%   row representing one Euler angle set. Rotation angles are in radians.
%   The output, EUL, is an N-by-4 matrix of N unit quaternions. Each quaternion
%   represents a 3D rotation and is of the form q = [w x y z], with a scalar number
%   as the first value. Each element of Q must be a real number.
%
%   The rotation sequence is 'ZYX', which is written as roll
%   pitch yaw angles expressed in reference frame.

if size(eul,1) ~= 3  % change to column vector if Euler angles written by a row vector 
    Eul = eul';
else 
    Eul = eul;
end

roll = Eul(1,:);
pitch = Eul(2,:);
yaw = Eul(3,:);

cy = cos(yaw./2);
sy = sin(yaw./2);
cp = cos(pitch./2);
sp = sin(pitch./2);
cr = cos(roll./2);
sr = sin(roll./2);

q = [   cy.*cp.*cr + sy.*sp.*sr, ...
        cy.*cp.*sr - sy.*sp.*cr, ...
     	sy.*cp.*sr + cy.*sp.*cr, ...
     	sy.*cp.*cr - cy.*sp.*sr]; % output with row vector

end
