%% Trajectory generation in 3d space, orientation represented by unit quaternion

% define waypoints and passing clock
t_steps=[0;1;3;10;];
q1=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0];
q2=[0.4, 0.3, 0.6, 0.9240, 0.3825, 0.0, 0.0];
q3=[1.0, 0.7, 0.9, 0.8224, 0.3605, 0.4395, 0.0224];
q4=[1.5, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0];
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
    
    q_traj=zeros(length(t_traj),7);
    qD_traj=zeros(length(t_traj),7);
    qDD_traj=zeros(length(t_traj),7);
    
    if (length(traj_type) == length('spline') && all(traj_type == 'spline')) 
        for k=1:7
            [q_traj(:,k),qD_traj(:,k),qDD_traj(:,k)] = GenTrajSpline(CoeffSpline, i, k, t_traj);
        end
    else
        for k=1:7
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
    quat = q_des(i,5:8);
    if(norm(quat)~=1)
        quat = quat/norm(quat);
        q_des(i,5:8) = quat;
    end
    quat_conj = quat;
    quat_conj(2:4) = -quat(2:4);
    
    dq_traj(1:3) = qdot_des(i,1:3);
    quat_dot = quatMultiplication(2*quat_conj,qdot_des(i,4:7));
    dq_traj(4:6) = quat_dot(2:4);
    
    ddq_traj(1:3) = qddot_des(i,1:3);
    quat_ddot = quatMultiplication(2*quat_conj,qddot_des(i,4:7));
    ddq_traj(4:6) = quat_ddot(2:4);
    
    dq_des = [dq_des; dq_traj];
    ddq_des = [ddq_des; ddq_traj]; 
end


figure(1);
plot(q_des(:,1), q_des(:,2:4));
title('position');
legend('x', 'y', 'z');
figure(2);
plot(q_des(:,1), q_des(:,5:8));
title('orientation');
legend('q0', 'q1', 'q2', 'q3');

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
q_temp = q_des(:,4);
q_des(:,4:6) = q_des(:,5:7);
q_des(:,7) = q_temp;
save('qtraj.txt', 'q_des', '-ascii', '-double', '-tabs');
save('dqtraj.txt', 'dq_des', '-ascii', '-double', '-tabs');
save('ddqtraj.txt', 'ddq_des', '-ascii', '-double', '-tabs');