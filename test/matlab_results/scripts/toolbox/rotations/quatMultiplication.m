function q_out = quatMultiplication(q1, q2)
% This function calculates the result of the multiplication between two unit
% quaternion. 
% INPUT: q1, q2 are vectors of quaternion 
% OUTPUT: q_out result of the multiplication, in row vector

size_q1 = size(q1,1); size_q2 = size(q2,1);
if size_q1 ~=4
    q1 = q1';
end

if size_q2 ~=4
    q2 = q2';
end


q_out = [ q1(1,:).*q2(1,:) - q1(2,:).*q2(2,:) - q1(3,:).*q2(3,:) - q1(4,:).*q2(4,:); ...
          q1(1,:).*q2(2,:) + q1(2,:).*q2(1,:) + q1(3,:).*q2(4,:) - q1(4,:).*q2(3,:); ...
          q1(1,:).*q2(3,:) - q1(2,:).*q2(4,:) + q1(3,:).*q2(1,:) + q1(4,:).*q2(2,:); ...
          q1(1,:).*q2(4,:) + q1(2,:).*q2(3,:) - q1(3,:).*q2(2,:) + q1(4,:).*q2(1,:);];
q_out = q_out';

end