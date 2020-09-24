/** @file trajectory_nd.h
 *  @brief Trajectory generation for n-dimensional variables (n>1) in joint space
 *
 *  @version: v1.0
 *  @date: 2 september 2020
 */
#pragma once

#include "trajectory_base.h"

namespace GenTrajLib
{
/** 
 *  \brief Derived class from TrajectoryBase for trajectory generation in joint space, with a given dimension during construction
 *         The trajectory defined in .txt file refers to desired increments for all dof of variables,
 *         i.e. pos_out = init_pos + des_pos, (thus if init_pos = 0, pos_out = des_pos)
 */
class TrajectoryNd : public TrajectoryBase
{
public:

    TrajectoryNd(TrajectoryType _type, const int _size);

    ~TrajectoryNd(){};

    void getTrajNow(Eigen::VectorXd& _pos, Eigen::VectorXd& _vel, Eigen::VectorXd& _acc, const double _tNow);

    void setInitPose(Eigen::VectorXd& _pos);

protected:

    Eigen::VectorXd init_pose; // initial pose
    
};

}