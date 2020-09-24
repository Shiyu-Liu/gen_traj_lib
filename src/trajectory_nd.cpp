#include "trajectory_nd.h"
using namespace GenTrajLib;

TrajectoryNd::TrajectoryNd(TrajectoryType _type, const int _size): TrajectoryBase(_type)
{ 
    n_dof = _size;
    init_pose.setZero(_size);
}

void TrajectoryNd::getTrajNow(Eigen::VectorXd& _pos, Eigen::VectorXd& _vel, Eigen::VectorXd& _acc, const double _tNow)
{
    getInterpValue(_pos, _vel, _acc, _tNow);
    //_pos.segment(0,3) += init_pos.segment(0,3);
}

void TrajectoryNd::setInitPose(Eigen::VectorXd& _pos)
// modify first waypoint to be the initial pose of the robot
{
    init_pose = _pos;
    waypoints.front().segment(1, n_dof) = init_pose;
}