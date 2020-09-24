function [q_traj, qD_traj, qDD_traj] = GenTrajSpline(CoeffSpline, index_segment, index_dof, t_traj)
    
coeff = CoeffSpline(:,index_dof,index_segment);
q_traj = coeff(1) + coeff(2)*t_traj + coeff(3)*t_traj.^2 + coeff(4)*t_traj.^3;
qD_traj = coeff(2) + 2*t_traj.*coeff(3) + 3*t_traj.^2*coeff(4);
qDD_traj = 2*coeff(3) + 6*t_traj.*coeff(4);

end
