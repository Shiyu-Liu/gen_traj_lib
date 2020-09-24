%% Trajectory generation in 3d space, orientation represented by unit quaternion

% define waypoints and passing clock
t_steps=[0;1;3;10;];
q1=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
q2=[0.4, 0.3, 0.6, 0.5, 0.35, 0.46];
q3=[1.0, 0.7, 0.9, 0.8, 0.80, 1.5];
q4=[1.5, 1.0, 1.0, 1.6, 1.2, 0.75];
q_steps=[q1;q2;q3;q4];

tech = 0.02; % step time

% traj_type = 'poly7order'; % 'poly3order', 'poly5order', 'poly7order', 'spline'
traj_type = 'spline';

if size(q_steps,1)~= size(t_steps,1)
    error('Size of q_steps and t_steps incoherent.');
end

q_des = []; dq_des = []; ddq_des = [];
qdot_des = [];
qddot_des = [];

CoeffSpline = ComputeCoeffSpline(q_steps,t_steps);

% Traj comp
for i=1:(length(t_steps)-1)
    t_begin=t_steps(i);
    t_end=t_steps(i+1);
    if i==(length(t_steps)-1)
        t_traj=(0:tech:(t_end-t_begin))';
    else
        t_traj=(0:tech:(t_end-t_begin-tech))';
    end
    q_ini=q_steps(i,:);
    q_end=q_steps(i+1,:);
    
    q_traj=zeros(length(t_traj),6);
    qD_traj=zeros(length(t_traj),6);
    qDD_traj=zeros(length(t_traj),6);
    
    if (length(traj_type) == length('spline') && all(traj_type == 'spline')) 
        for k=1:6
            [q_traj(:,k),qD_traj(:,k),qDD_traj(:,k)] = GenTrajSpline(CoeffSpline, i, k, t_traj);
        end
    else
        for k=1:6
            [q_traj(:,k),qD_traj(:,k),qDD_traj(:,k)] = GenTrajPolynomial(q_ini(k),q_end(k),t_traj,t_end-t_begin,traj_type);
        end
    end
    q_des=[q_des;[t_traj+t_begin, q_traj]];
    qdot_des=[qdot_des;qD_traj];
    qddot_des=[qddot_des;qDD_traj];
end

dq_traj = zeros(1,6);
ddq_traj = zeros(1,6);

for i=1:length(q_des(:,1))    
    
    dq_traj(1:3) = qdot_des(i,1:3);
    ddq_traj(1:3) = qddot_des(i,1:3);

    % computation of body-frame angular velocity
    eul = q_des(i,5:7)';
    d_eul = qdot_des(i,4:6)';
    dd_eul = qddot_des(i,4:6)';
    
    T = Mat_T(eul);
    dT = dMat_T(eul,d_eul);
    
    omega = T*d_eul;
    domega = T*dd_eul + dT*d_eul;
    
    dq_traj(4:6) = omega;
    ddq_traj(4:6) = domega;
    
    dq_des = [dq_des; dq_traj];
    ddq_des = [ddq_des; ddq_traj]; 
end

figure(1);
plot(q_des(:,1), q_des(:,2:4));
title('position');
legend('x', 'y', 'z');
figure(2);
plot(q_des(:,1), q_des(:,5:7));
title('orientation');
legend('phi', 'theta', 'psi');

figure(3);
plot(q_des(:,1), dq_des(:,1:3));
title('linear velocity');
legend('vx', 'vy', 'vz');
figure(4);
plot(q_des(:,1), dq_des(:,4:6));
title('angular velocity');
legend('wx', 'wy', 'wz');

figure(5);
plot(q_des(:,1), ddq_des(:,1:3));
title('linear acceleration');
legend('ax', 'ay', 'az');
figure(6);
plot(q_des(:,1), ddq_des(:,4:6));
title('angular acceleration');
legend('awx', 'awy', 'awz');

q_des = q_des(:,2:end);
save('qtraj.txt', 'q_des', '-ascii', '-double', '-tabs');
save('dqtraj.txt', 'dq_des', '-ascii', '-double', '-tabs');
save('ddqtraj.txt', 'ddq_des', '-ascii', '-double', '-tabs');


function T = Mat_T(eul)
% matrix relating euler angle rates to body-frame velocity
phi = eul(1);
theta = eul(2);
% psi = eul(3);

T = [ 1, 0, -sin(theta); ...
      0, cos(phi), sin(phi)*cos(theta);...
      0, -sin(phi), cos(phi)*cos(theta);];

end

function dT = dMat_T(eul, d_eul)
% derivative of matrix T

phi = eul(1);
theta = eul(2);
% psi = eul(3);
dphi = d_eul(1);
dtheta = d_eul(2);
% psi = eul(3);

dT = [0  0  -cos(theta)*dtheta; ...
      0  -sin(phi)*dphi   cos(phi)*dphi*cos(theta)-sin(phi)*sin(theta)*dtheta; ...
      0  -cos(phi)*dphi  -sin(phi)*dphi*cos(theta)-cos(phi)*sin(theta)*dtheta;];

end