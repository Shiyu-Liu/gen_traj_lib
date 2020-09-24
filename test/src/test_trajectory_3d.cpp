/** @file test_trajectory_3d.cpp
 *  @brief Google test for trajectory_lib module (comparison with Matlab results)
 *
 *  @version: v1.0
 *  @date: 2 september 2020
 */

#include <string>
#include <fstream>
#include <iostream>
#include <gtest/gtest.h>
#include <ros/package.h> // only used for getpath
#include <eigen3/Eigen/Dense>
#include "trajectory_3dspace.h"

using namespace GenTrajLib;

// global variables to be used
std::string path_G;
std::string path_rel_G = "/test/matlab_results/3DspaceQuat"; 
TrajectoryBase* pTraj_G = NULL;

// function to charge .txt file saved from matlab and convert to eigen matrix 
void getFloatVectorFromString(const std::string& string_line, std::vector<double>& ret)
{
    ret.clear();
    std::istringstream s(string_line);
    double d;
    while (s >> d) 
    {
        ret.push_back(d);
    }
    return;
}

bool parseTxtFileToEigenMat(const std::string& file_name, Eigen::MatrixXd& ret, std::string& msgerr)
{
    std::ifstream fin(file_name.c_str());
    if(!fin.is_open())
    {
        msgerr = "Error opening file: " + file_name;
        return false;
    }

    std::string s_line;
    std::vector<double> row_data;
    std::vector<std::vector<double>> mat_data;
    while(!fin.eof())
    {
        std::getline(fin, s_line); 
        if(s_line.empty()) break; 
        getFloatVectorFromString(s_line, row_data);
        mat_data.push_back(row_data); // suppose that the data of all rows has same size
    }
    fin.close();

    int rows, cols;
    rows = mat_data.size();
    cols = mat_data.begin()->size(); // take the first row to get number of cols

    ret.resize(rows, cols);
    for(int row = 0; row < rows; row++)
    {
        for (int col = 0; col < cols; col++)
        {
            ret(row, col) = mat_data.at(row).at(col);
        }  
    }
    return true;
}


TEST(TestTrajectory, test_trajectory_poly3)
{
    //// Start Load Trajectory ////
    std::cout << "[----------] Start: test_trajectory_poly3"<<std::endl;
    pTraj_G = new Trajectory3dQuat(TrajectoryType::Poly3order);

    Trajectory3dQuat* pTraj = (Trajectory3dQuat*) pTraj_G;

    std::string traj_file = path_G + "/test/traj/test_traj_3dquat.txt";
    
    std::string msgerr;

    if(!pTraj->loadWaypointFromTxtFile(traj_file, msgerr))
    {
        ADD_FAILURE() << msgerr.data();
        return;
    }

    if(!pTraj->doInterpolation(msgerr)) // interpolate trajectory
    {
        ADD_FAILURE() << msgerr.data();
        return;
    }

    double t_init = 153024.51534; // a random number to simulate a ros time
    pTraj->setInitTime(t_init);

    double t_step = 0.02;
    const int index_fin = 501; // trajFPR.waypoints.back()(0)/t_step + 1;

    Eigen::Matrix<double, index_fin, 7> pose_comp;
    Eigen::Matrix<double, index_fin, 6> vel_comp;
    Eigen::Matrix<double, index_fin, 6> acc_comp;

    Eigen::VectorXd pose, vel, acc;

    double t_now = t_init;
    for(int index = 0; index<index_fin; index++)
    {
        t_now = t_init + index*t_step;
        pTraj->getTrajNow(pose, vel, acc, t_now);
        pose_comp.row(index) = pose.transpose();
        vel_comp.row(index) = vel.transpose();
        acc_comp.row(index) = acc.transpose();
    }

    // load matlab results for comparison
    Eigen::MatrixXd ret;
    if(!parseTxtFileToEigenMat(path_G+path_rel_G+"/qtraj_poly3.txt", ret, msgerr))
    {
        ADD_FAILURE() << msgerr.data();
        return;
    }
    Eigen::Matrix<double,index_fin, 7> pose_matlab = ret;
    
    if(!parseTxtFileToEigenMat(path_G+path_rel_G+"/dqtraj_poly3.txt", ret, msgerr))
    {
        ADD_FAILURE() << msgerr.data();
        return;
    }
    Eigen::Matrix<double,index_fin, 6> vel_matlab = ret;

    if(!parseTxtFileToEigenMat(path_G+path_rel_G+"/ddqtraj_poly3.txt", ret, msgerr))
    {
        ADD_FAILURE() << msgerr.data();
        return;
    }
    Eigen::Matrix<double,index_fin, 6> acc_matlab = ret;

    ASSERT_TRUE(pose_comp.isApprox(pose_matlab, 1E-7));
    std::cout << "[----------] Pose trajectory correct" << std::endl;

    ASSERT_TRUE(vel_comp.isApprox(vel_matlab, 1E-7));
    std::cout << "[----------] Velocity trajectory correct" << std::endl;

    ASSERT_TRUE(acc_comp.isApprox(acc_matlab, 1E-7));
    std::cout << "[----------] Acceleration trajectory correct" << std::endl;

    std::cout << "[----------] End: test_trajectory_poly3"<<std::endl;
    delete pTraj_G; pTraj_G = NULL;
}


TEST(TestTrajectory, test_trajectory_poly5)
{
    //// Start Load Trajectory ////
    std::cout << "[----------] Start: test_trajectory_poly5"<<std::endl;

    pTraj_G = new Trajectory3dQuat(TrajectoryType::Poly5order);
    Trajectory3dQuat* pTraj = (Trajectory3dQuat*) pTraj_G;

    std::string traj_file = path_G + "/test/traj/test_traj_3dquat.txt";
    std::string msgerr;

    if(!pTraj->loadWaypointFromTxtFile(traj_file, msgerr))
    {
        ADD_FAILURE() << msgerr.data();
        return;
    }

    if(!pTraj->doInterpolation(msgerr)) // interpolate trajectory
    {
        ADD_FAILURE() << msgerr.data();
        return;
    }

    double t_init = 153024.51534; // a random number to simulate a ros time
    pTraj->setInitTime(t_init);

    double t_step = 0.02;
    const int index_fin = 501; // trajFPR.waypoints.back()(0)/t_step + 1;

    Eigen::Matrix<double, index_fin, 7> pose_comp;
    Eigen::Matrix<double, index_fin, 6> vel_comp;
    Eigen::Matrix<double, index_fin, 6> acc_comp;

    Eigen::VectorXd pose, vel, acc;

    double t_now = t_init;
    for(int index = 0; index<index_fin; index++)
    {
        t_now = t_init + index*t_step;
        pTraj->getTrajNow(pose, vel, acc, t_now);
        pose_comp.row(index) = pose.transpose();
        vel_comp.row(index) = vel.transpose();
        acc_comp.row(index) = acc.transpose();
    }

    // load matlab results for comparison
    Eigen::MatrixXd ret;
    if(!parseTxtFileToEigenMat(path_G+path_rel_G+"/qtraj_poly5.txt", ret, msgerr))
    {
        ADD_FAILURE() << msgerr.data();
        return;
    }
    Eigen::Matrix<double,index_fin, 7> pose_matlab = ret;
    
    if(!parseTxtFileToEigenMat(path_G+path_rel_G+"/dqtraj_poly5.txt", ret, msgerr))
    {
        ADD_FAILURE() << msgerr.data();
        return;
    }
    Eigen::Matrix<double,index_fin, 6> vel_matlab = ret;

    if(!parseTxtFileToEigenMat(path_G+path_rel_G+"/ddqtraj_poly5.txt", ret, msgerr))
    {
        ADD_FAILURE() << msgerr.data();
        return;
    }
    Eigen::Matrix<double,index_fin, 6> acc_matlab = ret;

    ASSERT_TRUE(pose_comp.isApprox(pose_matlab, 1E-7));
    std::cout << "[----------] Pose trajectory correct" << std::endl;

    ASSERT_TRUE(vel_comp.isApprox(vel_matlab, 1E-7));
    std::cout << "[----------] Velocity trajectory correct" << std::endl;

    ASSERT_TRUE(acc_comp.isApprox(acc_matlab, 1E-7));
    std::cout << "[----------] Acceleration trajectory correct" << std::endl;

    std::cout << "[----------] End: test_trajectory_poly5"<<std::endl;
    delete pTraj_G; pTraj_G = NULL;
}


TEST(TestTrajectory, test_trajectory_poly7)
{
    //// Start Load Trajectory ////
    std::cout << "[----------] Start: test_trajectory_poly7"<<std::endl;

    pTraj_G = new Trajectory3dQuat(TrajectoryType::Poly7order);
    Trajectory3dQuat* pTraj = (Trajectory3dQuat*) pTraj_G;

    std::string traj_file = path_G + "/test/traj/test_traj_3dquat.txt";
    std::string msgerr;

    if(!pTraj->loadWaypointFromTxtFile(traj_file, msgerr))
    {
        ADD_FAILURE() << msgerr.data();
        return;
    }

    if(!pTraj->doInterpolation(msgerr)) // interpolate trajectory
    {
        ADD_FAILURE() << msgerr.data();
        return;
    }

     double t_init = 153024.51534; // a random number to simulate a ros time
    pTraj->setInitTime(t_init);

    double t_step = 0.02;
    const int index_fin = 501; // trajFPR.waypoints.back()(0)/t_step + 1;

    Eigen::Matrix<double, index_fin, 7> pose_comp;
    Eigen::Matrix<double, index_fin, 6> vel_comp;
    Eigen::Matrix<double, index_fin, 6> acc_comp;

    Eigen::VectorXd pose, vel, acc;

    double t_now = t_init;
    for(int index = 0; index<index_fin; index++)
    {
        t_now = t_init + index*t_step;
        pTraj->getTrajNow(pose, vel, acc, t_now);
        pose_comp.row(index) = pose.transpose();
        vel_comp.row(index) = vel.transpose();
        acc_comp.row(index) = acc.transpose();
    }

    // load matlab results for comparison
    Eigen::MatrixXd ret;
    if(!parseTxtFileToEigenMat(path_G+path_rel_G+"/qtraj_poly7.txt", ret, msgerr))
    {
        ADD_FAILURE() << msgerr.data();
        return;
    }
    Eigen::Matrix<double,index_fin, 7> pose_matlab = ret;
    
    if(!parseTxtFileToEigenMat(path_G+path_rel_G+"/dqtraj_poly7.txt", ret, msgerr))
    {
        ADD_FAILURE() << msgerr.data();
        return;
    }
    Eigen::Matrix<double,index_fin, 6> vel_matlab = ret;

    if(!parseTxtFileToEigenMat(path_G+path_rel_G+"/ddqtraj_poly7.txt", ret, msgerr))
    {
        ADD_FAILURE() << msgerr.data();
        return;
    }
    Eigen::Matrix<double,index_fin, 6> acc_matlab = ret;

    ASSERT_TRUE(pose_comp.isApprox(pose_matlab, 1E-7));
    std::cout << "[----------] Pose trajectory correct" << std::endl;

    ASSERT_TRUE(vel_comp.isApprox(vel_matlab, 1E-7));
    std::cout << "[----------] Velocity trajectory correct" << std::endl;

    ASSERT_TRUE(acc_comp.isApprox(acc_matlab, 1E-7));
    std::cout << "[----------] Acceleration trajectory correct" << std::endl;

    std::cout << "[----------] End: test_trajectory_poly7"<<std::endl;
    delete pTraj_G; pTraj_G = NULL;
}

TEST(TestTrajectory, test_trajectory_spline)
{
    //// Start Load Trajectory ////
    std::cout << "[----------] Start: test_trajectory_spline"<<std::endl;

    pTraj_G = new Trajectory3dQuat(TrajectoryType::Spline);
    Trajectory3dQuat* pTraj = (Trajectory3dQuat*) pTraj_G;

    std::string traj_file = path_G + "/test/traj/test_traj_3dquat.txt";
    std::string msgerr;

    if(!pTraj->loadWaypointFromTxtFile(traj_file, msgerr))
    {
        ADD_FAILURE() << msgerr.data();
        return;
    }
    
    if(!pTraj->doInterpolation(msgerr)) // interpolate trajectory
    {
        ADD_FAILURE() << msgerr.data();
        return;
    }

    double t_init = 153024.51534; // a random number to simulate a ros time
    pTraj->setInitTime(t_init);

    double t_step = 0.02;
    const int index_fin = 501; // trajFPR.waypoints.back()(0)/t_step + 1;

    Eigen::Matrix<double, index_fin, 7> pose_comp;
    Eigen::Matrix<double, index_fin, 6> vel_comp;
    Eigen::Matrix<double, index_fin, 6> acc_comp;

    Eigen::VectorXd pose, vel, acc;

    double t_now = t_init;
    for(int index = 0; index<index_fin; index++)
    {
        t_now = t_init + index*t_step;
        pTraj->getTrajNow(pose, vel, acc, t_now);
        pose_comp.row(index) = pose.transpose();
        vel_comp.row(index) = vel.transpose();
        acc_comp.row(index) = acc.transpose();
    }

    // load matlab results for comparison
    Eigen::MatrixXd ret;
    if(!parseTxtFileToEigenMat(path_G+path_rel_G+"/qtraj_spline.txt", ret, msgerr))
    {
        ADD_FAILURE() << msgerr.data();
        return;
    }
    Eigen::Matrix<double,index_fin, 7> pose_matlab = ret;
    
    if(!parseTxtFileToEigenMat(path_G+path_rel_G+"/dqtraj_spline.txt", ret, msgerr))
    {
        ADD_FAILURE() << msgerr.data();
        return;
    }
    Eigen::Matrix<double,index_fin, 6> vel_matlab = ret;

    if(!parseTxtFileToEigenMat(path_G+path_rel_G+"/ddqtraj_spline.txt", ret, msgerr))
    {
        ADD_FAILURE() << msgerr.data();
        return;
    }
    Eigen::Matrix<double,index_fin, 6> acc_matlab = ret;

    ASSERT_TRUE(pose_comp.isApprox(pose_matlab, 1E-7));
    std::cout << "[----------] Pose trajectory correct" << std::endl;

    ASSERT_TRUE(vel_comp.isApprox(vel_matlab, 1E-7));
    std::cout << "[----------] Velocity trajectory correct" << std::endl;

    ASSERT_TRUE(acc_comp.isApprox(acc_matlab, 1E-7));
    std::cout << "[----------] Acceleration trajectory correct" << std::endl;

    std::cout << "[----------] End: test_trajectory_spline"<<std::endl;
    delete pTraj_G; pTraj_G = NULL;
}

TEST(TestTrajectory, test_trajectory_euler)
{
    //// Start Load Trajectory ////
    std::cout << "[----------] Start: test_trajectory_Euler"<<std::endl;

    pTraj_G = new Trajectory3dEuler(TrajectoryType::Spline);
    Trajectory3dEuler* pTraj = (Trajectory3dEuler*) pTraj_G;

    std::string traj_file = path_G + "/test/traj/test_traj_3deuler.txt";
    std::string msgerr;

    if(!pTraj->loadWaypointFromTxtFile(traj_file, msgerr))
    {
        ADD_FAILURE() << msgerr.data();
        return;
    }

    if(!pTraj->doInterpolation(msgerr)) // interpolate trajectory
    {
        ADD_FAILURE() << msgerr.data();
        return;
    }

     double t_init = 153024.51534; // a random number to simulate a ros time
    pTraj->setInitTime(t_init);

    double t_step = 0.02;
    const int index_fin = 501; // trajFPR.waypoints.back()(0)/t_step + 1;

    Eigen::Matrix<double, index_fin, 6> pose_comp;
    Eigen::Matrix<double, index_fin, 6> vel_comp;
    Eigen::Matrix<double, index_fin, 6> acc_comp;

    Eigen::VectorXd pose, vel, acc;

    double t_now = t_init;
    for(int index = 0; index<index_fin; index++)
    {
        t_now = t_init + index*t_step;
        pTraj->getTrajNow(pose, vel, acc, t_now);
        pose_comp.row(index) = pose.transpose();
        vel_comp.row(index) = vel.transpose();
        acc_comp.row(index) = acc.transpose();
    }

    // load matlab results for comparison
    Eigen::MatrixXd ret;
    path_rel_G = "/test/matlab_results/3DspaceEuler";
    if(!parseTxtFileToEigenMat(path_G+path_rel_G+"/qtraj_spline.txt", ret, msgerr))
    {
        ADD_FAILURE() << msgerr.data();
        return;
    }
    Eigen::Matrix<double,index_fin, 6> pose_matlab = ret;
    
    if(!parseTxtFileToEigenMat(path_G+path_rel_G+"/dqtraj_spline.txt", ret, msgerr))
    {
        ADD_FAILURE() << msgerr.data();
        return;
    }
    Eigen::Matrix<double,index_fin, 6> vel_matlab = ret;

    if(!parseTxtFileToEigenMat(path_G+path_rel_G+"/ddqtraj_spline.txt", ret, msgerr))
    {
        ADD_FAILURE() << msgerr.data();
        return;
    }
    Eigen::Matrix<double,index_fin, 6> acc_matlab = ret;

    ASSERT_TRUE(pose_comp.isApprox(pose_matlab, 1E-7));
    std::cout << "[----------] Pose trajectory correct" << std::endl;

    ASSERT_TRUE(vel_comp.isApprox(vel_matlab, 1E-7));
    std::cout << "[----------] Velocity trajectory correct" << std::endl;

    ASSERT_TRUE(acc_comp.isApprox(acc_matlab, 1E-7));
    std::cout << "[----------] Acceleration trajectory correct" << std::endl;

    std::cout << "[----------] End: test_trajectory_Euler"<<std::endl;
    delete pTraj_G; pTraj_G = NULL;
}


int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    path_G = ros::package::getPath("gen_traj_lib");

    return RUN_ALL_TESTS();
    return 0;
}