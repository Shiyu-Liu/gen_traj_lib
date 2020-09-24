function CoeffSpline = ComputeCoeffSpline(q_steps,t_steps)

n_segment = length(t_steps)-1;
n_dof = size(q_steps,2);

CoeffSpline = zeros(4,n_dof,n_segment);

T = zeros(n_segment,1); % time interval for each segment
H = zeros(n_segment, n_dof); % difference of initial and final value for each segment

for i=1:n_segment
    T(i) = t_steps(i+1) - t_steps(i);
    H(i,:) = q_steps(i+1,:) - q_steps(i,:);
end

Vel = zeros(n_segment+1, n_dof); % intermediary velocities

for i=1:n_dof
    Vel(2:n_segment,i) = computeIntVelSpine(T, H(:,i), n_segment);
end

for i=1:n_segment 
    CoeffSpline(1,:,i) = q_steps(i,:);
    CoeffSpline(2,:,i) = Vel(i,:);
    CoeffSpline(3,:,i) = (1/T(i))*(3*H(i,:)/T(i)-2*Vel(i,:)-Vel(i+1,:));
    CoeffSpline(4,:,i) = (1/T(i)^2)*(-2*H(i,:)/T(i)+Vel(i,:) + Vel(i+1,:));   
end

end


function vel = computeIntVelSpine(T, h, n) 

A = zeros(n-1, n-1);
C = zeros(n-1, 1);

for i=1:n-1
    if(i==1)
        A(1,1) = 2*(T(2)+T(1));
        A(1,2) = T(1);
    elseif (i == n-1)
        A(i,i) = 2*(T(i+1)+T(i));
        A(i,i-1) = T(i+1);
    else
        A(i,i) = 2*(T(i+1)+T(i));
        A(i,i-1) = T(i+1);
        A(i,i+1) = T(i);
    end

    C(i) = 3/(T(i)*T(i+1))*(T(i)^2*h(i+1)+T(i+1)^2*h(i));
end

vel = A\C;

end