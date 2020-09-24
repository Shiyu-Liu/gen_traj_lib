function [q_traj, qD_traj, qDD_traj]= GenTrajPolynomial(qi,qe, t_traj, dT, traj_type)

if (all(traj_type == 'poly3order'))
    p = 3*(t_traj./dT).^2 - 2*(t_traj./dT).^3;
    dp = 6*t_traj./dT^2 - 6*t_traj.^2/dT^3;
    ddp = 6/dT^2 - 12*t_traj./dT^3;
    q_traj = qi + (qe-qi).*p;
    qD_traj = (qe-qi).*dp;
    qDD_traj = (qe-qi).*ddp;
    
elseif (all(traj_type == 'poly5order'))
    p = 10*(t_traj./dT).^3 - 15*(t_traj./dT).^4 + 6*(t_traj./dT).^5;
    dp = 30*t_traj.^2/dT^3 - 60*t_traj.^3/dT^4 + 30*t_traj.^4/dT^5;
    ddp = 60*t_traj./dT^3 - 180*t_traj.^2/dT^4 + 120*t_traj.^3/dT^5;
    q_traj = qi + (qe-qi).*p;
    qD_traj = (qe-qi).*dp;
    qDD_traj = (qe-qi).*ddp;
    
elseif (all(traj_type == 'poly7order'))
    p = 35*(t_traj/dT).^4 - 84*(t_traj/dT).^5 + 70*(t_traj/dT).^6 - 20*(t_traj/dT).^7;
    dp = 140*t_traj.^3/dT^4 - 420*t_traj.^4/dT^5 + 420*t_traj.^5/dT^6 - 140*t_traj.^6/dT^7;
    ddp = 420*t_traj.^2/dT^4 - 1680*t_traj.^3/dT^5 + 2100*t_traj.^4/dT^6 - 840*t_traj.^5/dT^7;
    q_traj = qi + (qe-qi).*p;
    qD_traj = (qe-qi).*dp;
    qDD_traj = (qe-qi).*ddp;
    
else
    error('traj_type incorrect');
end

end