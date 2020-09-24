#include "trajectory_3dspace.h"
using namespace GenTrajLib;

//******************** Declaration of Trajectory3dQuat member function *********************//
Trajectory3dQuat::Trajectory3dQuat(TrajectoryType _type) : TrajectoryBase(_type)
{ 
    n_dof = state_size; 
    init_position.setZero();
    init_attitude.setIdentity();
}

void Trajectory3dQuat::getTrajNow(Eigen::VectorXd& _pos, Eigen::VectorXd& _vel, Eigen::VectorXd& _acc, const double _tNow)
{
    Eigen::VectorXd raw_pos; raw_pos.setZero(state_size); // raw position data
    Eigen::VectorXd raw_vel; raw_vel.setZero(state_size); // raw velocity (corresponding to state derivatives)
    Eigen::VectorXd raw_acc; raw_acc.setZero(state_size); // raw acceleration (idem)

    // resize the output variables
    _pos.resize(state_size); 
    _vel.resize(deriv_size);
    _acc.resize(deriv_size);

    getInterpValue(raw_pos, raw_vel, raw_acc, _tNow);
    raw_pos.segment(0,3) += init_position; // add offset related to the initial position

    if(_tNow > t_fin)
    {
        _pos = raw_pos;
        _vel.setZero(deriv_size);
        _acc.setZero(deriv_size);
        return;
    }

    Eigen::Quaterniond quat; quat = Eigen::Vector4d(raw_pos.segment(3,4)); // orientation of the platform represented by quaternion
    Eigen::Quaterniond quat_dot; quat_dot = Eigen::Vector4d(raw_vel.segment(3,4)); // derivatives of quaternion
    Eigen::Quaterniond quat_ddot; quat_ddot = Eigen::Vector4d(raw_acc.segment(3,4));

    if(quat.norm() != 1) // normalize quternion
        quat.normalize();

    /* computation of body-frame angular velocity/acceleration based on quaternion and its derivatives */
    Eigen::Quaterniond quat_conj = quat.conjugate();
    quat_conj.w()*=2.0; quat_conj.vec()*=2.0;
    // quat_dot = 1/2* quat (*) w   =>   w = 2* (quat_conj (*) quat_dot)   (*): represents quaternion multiplication
    Eigen::Quaterniond w = quat_conj*quat_dot; // w = [0; omega] where omega is angular velocity expressed in body frame

    // w_dot = 2*(quat_conj (*) quat_ddot + norm2(quat_dot))
    Eigen::Quaterniond w_dot = quat_conj*quat_ddot;
    w_dot.w() += 2.0*quat_dot.squaredNorm(); // w_dot = [0; omega_dot] angular acceleration
    
    /* outputs: */
    // position:
    _pos << raw_pos.segment(0,3),   // position
            quat.vec(), quat.w();   // orientation (be careful of order -> quaternion = [qx, qy, qz, qw])
    
    // velocity:
    _vel << raw_vel.segment(0,3), w.vec();

    // acceleration:
    _acc << raw_acc.segment(0,3), w_dot.vec();

    return;
}

void Trajectory3dQuat::setInitPose(Eigen::Vector3d _init_pos, Eigen::Quaterniond _init_att)
// modify first waypoint to be the initial pose of the robot
{
    init_position = _init_pos;
    init_attitude = _init_att;
    waypoints.front().segment(4,4) << _init_att.vec(), _init_att.w();
}
//******************************************************************************************//


//******************** Declaration of Trajectory3dEuler member function *********************//
Trajectory3dEuler::Trajectory3dEuler(TrajectoryType _type): TrajectoryBase(_type)
{ 
    n_dof = state_size; 
    init_position.setZero();
    init_attitude.setZero();
}

void Trajectory3dEuler::getTrajNow(Eigen::VectorXd& _pos, Eigen::VectorXd& _vel, Eigen::VectorXd& _acc, const double _tNow)
{
    // resize the output variables
    _pos.setZero(state_size); // raw position data
    _vel.setZero(state_size); // raw velocity (corresponding to state derivatives)
    _acc.setZero(state_size); // raw acceleration (idem)

    // get interpolated value for the current time
    getInterpValue(_pos, _vel, _acc, _tNow);
    _pos.segment(0,3) += init_position; // add offset related to the initial position
    
    if(_tNow > t_fin)
        return;
    
    // computation of body-frame angular velocity
    Eigen::Vector3d eul; // euler angles
    eul = _pos.segment(3,3);
    
    Eigen::Vector3d rates_eul; // rates of euler angle 
    rates_eul = _vel.segment(3,3);

    Eigen::Matrix3d matT; // matrix relating euler angle rates to body-frame velocity
    matT << 1, 0, -sin(eul(1)),                          // T = [1   0      -sin(theta); 
            0,  cos(eul(0)), sin(eul(0))*cos(eul(1)),    //      0  cos(phi) sin(phi)*cos(theta); 
            0, -sin(eul(0)), cos(eul(0))*cos(eul(1));    //      0 -sin(phi) cos(phi)*cos(theta)]
    
    Eigen::Vector3d omega = matT*rates_eul;

    // computation of body-frame angular acceleration
    Eigen::Vector3d acc_eul; // second-order derivatives of euler angle 
    acc_eul = _acc.segment(3,3);

    Eigen::Matrix3d matT_dot; // derivative of matrix T
    matT_dot << 0,  0,   -cos(eul(1))*rates_eul(1),                      
                0,  -sin(eul(0))*rates_eul(0),  cos(eul(0))*cos(eul(1))*rates_eul(0)-sin(eul(0))*sin(eul(1))*rates_eul(1), 
                0,  -cos(eul(0))*rates_eul(0), -sin(eul(0))*cos(eul(1))*rates_eul(0)-cos(eul(0))*sin(eul(1))*rates_eul(1);
    // dT = [0  0  -cos(theta)*dtheta;
    //       0  -sin(phi)*dphi   cos(phi)*dphi*cos(theta)-sin(phi)*sin(theta)*dtheta; 
    //       0  -cos(phi)*dphi  -sin(phi)*dphi*cos(theta)-cos(phi)*sin(theta)*dtheta]

    Eigen::Vector3d omega_dot = matT*acc_eul + matT_dot*rates_eul;

    /* outputs: */
    _vel.segment(3,3) = omega;
    _acc.segment(3,3) = omega_dot;
    return;
}

void Trajectory3dEuler::setInitPose(Eigen::Vector3d _init_pos, Eigen::Vector3d _init_eul)
// modify first waypoint to be the initial pose of the robot
{
    init_position = _init_pos;
    init_attitude = _init_eul;
    waypoints.front().segment(4,3) = _init_eul;
}
//******************************************************************************************//