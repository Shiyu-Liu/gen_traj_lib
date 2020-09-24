/** @file trajectory_3dspace.h
 *  @brief 3D space trajectory generation, with Euler angle and quaternion possibilities for orientation representation
 *
 *  @version: v1.0
 *  @date: 2 september 2020
 */
#pragma once

#include "trajectory_base.h"

namespace GenTrajLib
{

/** 
 *  \brief Derived class from TrajectoryBase for trajectory generation in 3D space, with orientation represented by unit quaternion
 *         Waypoints defined in .txt file, position defiend as desired increments from the initial position, 
 *         Quaternion defined as absolute attitude w.r.t. world frame (take care of quaternion order q=[qx, qy, qz, qw])
 *         Initial attitude will be taken into account by the interpolation if calling setInitPose function.
 */
class Trajectory3dQuat : public TrajectoryBase
{
public:
    static const int state_size = 7; // state size = 7
    static const int deriv_size = 6; // size of derived variables (velocity and acceleration)

    Trajectory3dQuat(TrajectoryType _type);
    ~Trajectory3dQuat(){};

    void getTrajNow(Eigen::VectorXd& _pos, Eigen::VectorXd& _vel, Eigen::VectorXd& _acc, const double _tNow);

    void setInitPose(Eigen::Vector3d _init_pos, Eigen::Quaterniond _init_att);

protected:

    Eigen::Vector3d init_position; // initial position
    Eigen::Quaterniond init_attitude; // initial attitude

};



/** 
 *  \brief Derived class from TrajectoryBase for trajectory generation in 3D space, with orientation represented by roll-pitch-yaw
 *         Euler angles (equivalent to ZYX convention). Euler angles are written in order of phi(x-axis), theta(y-axis), psi(z-axis)
 * 
 */
class Trajectory3dEuler : public TrajectoryBase
{
public:
    static const int state_size = 6; // state size = 6, same for derived type variables

    Trajectory3dEuler(TrajectoryType _type);
    ~Trajectory3dEuler(){};

    void getTrajNow(Eigen::VectorXd& _pos, Eigen::VectorXd& _vel, Eigen::VectorXd& _acc, const double _tNow);

    void setInitPose(Eigen::Vector3d _init_pos, Eigen::Vector3d _init_eul);

protected:

    Eigen::Vector3d init_position; // initial position
    Eigen::Vector3d init_attitude; // initial attitude
    
};

}