/** @file trajectory_base.h
 *  @brief Base class for trajectory generation
 *
 *  @version: v1.0
 *  @date: 2 september 2020
 */
#pragma once

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

namespace GenTrajLib
{

enum TrajectoryType
{  
    Spline,  // cubic spline
    Poly3order, // 3-order polynomial function 
    Poly5order, // 5-order polynomial function
    Poly7order, // 7-order polynomial function
};

class TrajectoryBase
{
public:
    
    TrajectoryBase(TrajectoryType _type): traj_type(_type)
    {
        initialized = false; 
        interpolated = false;
        n_dof = 0; 
        n_wps = 0;
    };
    ~TrajectoryBase(){};

    bool loadWaypointFromTxtFile(const std::string &file_name, std::string &msgerr);
    
    void setInitTime(double _t0) 
    { 
        if(interpolated)
        {
            t_init = _t0; 
            initialized = true;
            
            t_fin = _t0 + waypoints.back()(0);
        }
    };

    double getFinalTime() {  return t_fin; };

    virtual void getTrajNow(Eigen::VectorXd& _pos, Eigen::VectorXd& _vel, Eigen::VectorXd& _acc, const double _tNow) = 0;
    // pure virtual function to be defined in derived class (abstract base class)

    bool doInterpolation(std::string& msgerr);

protected:

    void getInterpValue(Eigen::VectorXd& _pos, Eigen::VectorXd& _vel, Eigen::VectorXd& _acc, const double _tNow);

    TrajectoryType traj_type;
    std::vector<Eigen::VectorXd> waypoints; // waypoints defined off-line, 
    // time table associated with waypoints, defined in seconds, must be stocked in the first row

    std::vector<Eigen::MatrixXd> coeff; // coefficients of trajectory, for all segments
    // For each segment, a MatrixXd structure stocks the coefficients in row,
    // different rows refer to coefficients of different DOF of variables.
    // If coefficients are identical for all dof of variables (i.e. for polynomial functions), 
    // they are stocked in a VectorXd-type variable.

    bool initialized; 
    bool interpolated;

    double t_init; // initial time, in seconds
    double t_fin; // final time

    int n_dof; // number of DOF to be interpolated 
    int n_wps; // number of waypoints (number of segments equals n_wps-1)

};

}