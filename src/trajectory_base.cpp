#include "trajectory_base.h"
#include <fstream>
using namespace GenTrajLib;

/********************** Basic interpolation functions ***********************/
Eigen::Matrix<double, 4, 1> computeCoeff3order(const double _tf)
// 3-order polynomial function: 
// q(t) = q0 + P(t)*(qf-q0) 
// P(t) = p0 + p1*t + p2*t^2 + p3*t^3
{       
    Eigen::Matrix<double, 4, 1> coeff; 
    coeff.setZero();

    coeff(2) = 3.0/(_tf*_tf); // 3/tf^2
    coeff(3) = -2.0/(_tf*_tf*_tf); // -2/tf^3

    return coeff;
}

Eigen::Matrix<double, 6, 1> computeCoeff5order(const double _tf)
// 5-order polynomial function: 
// q(t) = q0 + P(t)*(qf-q0) 
// P(t) = p0 + p1*t + p2*t^2 + p3*t^3 + p4*t^4 + p5*t^5
{       
    Eigen::Matrix<double, 6, 1> coeff; 
    coeff.setZero();

    coeff(3) = 10.0/(_tf*_tf*_tf); // 10/tf^3
    coeff(4) = -15.0/(_tf*_tf*_tf*_tf); // -15/tf^4
    coeff(5) = 6.0/(_tf*_tf*_tf*_tf*_tf); // 6/tf^5

    return coeff;
}

Eigen::Matrix<double, 8, 1> computeCoeff7order(const double _tf)
// 7-order polynomial function: 
// q(t) = q0 + P(t)*(qf-q0) 
// P(t) = p0 + p1*t + p2*t^2 + p3*t^3 + p4*t^4 + p5*t^5 + p6*t^6 + p7*t^7
{
    Eigen::Matrix<double, 8, 1> coeff; 
    coeff.setZero();

    coeff(4) = 35.0/pow(_tf, 4); // 35/tf^4
    coeff(5) = -84.0/pow(_tf, 5); // -84/tf^5
    coeff(6) = 70.0/pow(_tf, 6); // 70/tf^6
    coeff(7) = -20.0/pow(_tf, 7); // -20/tf^7

    return coeff;
}

Eigen::VectorXd computeIntVelSpline(const Eigen::VectorXd& T, const Eigen::VectorXd& H, const int n_segment)
// computation of intermediary velocities of splines knowing time interval and desired values of all segments
/* Cubic splines for trajectory segments passing intermediary points 
*  3-order polynomial function for segment k: q_k(t) = p_k0 + p_k1 *t + p_k2 *t^2 + p_k3 *t^3
*  with q_k(0) = q*_{k-1}, q_k(T_k) = q*_k, qdot_k(0) = v_{k-1}, qdot_k(T_k) = v_k
*  
*  Arguments: 
*  const Eigen::VectorXd& T: time interval for segments
*  const Eigen::VectorXd& H: desired value interval for segments 
*/
{
    Eigen::MatrixXd A; A.resize(n_segment-1, n_segment-1);
    Eigen::VectorXd C; C.resize(n_segment-1);
    // Solve A*x=C, x refers to velocities of intermediary segments

    int index = 0; // index of rows
    for (; index < n_segment-1; index++) // (n_segment-1) rows
    {
        if(index==0) // first row
        {
            A(0,0) = 2*(T(1)+T(0));
            A(0,1) = T(0);
        } 
        else if(index==n_segment-2) // last row
        {
            A(index,index) = 2*(T(index+1)+T(index));
            A(index,index-1) = T(index+1);
        }
        else // other rows
        {
            A(index,index) = 2*(T(index+1) + T(index));
            A(index,index-1) = T(index+1);
            A(index,index+1) = T(index);
        }

        double Tk = T(index); // time interval for the current segment
        double Tkplus1 = T(index+1); // time interval for the next segment
        C(index) = 3.0/(Tk*Tkplus1)*(Tk*Tk*H(index+1) + Tkplus1*Tkplus1*H(index));
    }

    Eigen::RowVectorXd X; X.resize(n_segment-1);
    X = A.inverse()*C;

    return X;
}
/*******************************************************************************/


/************ Protected functions to be used by derived class ******************/

bool TrajectoryBase::doInterpolation(std::string& msgerr)
// Interpolation with specified trajectory type, and the coefficients stocked in TrajBase::coeff
{
    if(n_wps < 2)
    {
        msgerr = "No enough waypoints defined (at least 2)";
        return false;
    }
    if (traj_type == TrajectoryType::Spline)
    {
        /* computation of constants*/
        const int n_segment = n_wps - 1; // number of trajectory segments

        Eigen::VectorXd T; T.resize(n_segment); // time interval for each segment
        Eigen::MatrixXd H; H.resize(n_dof, n_segment); // difference of initial and final value for each segment
            
        int index_segment = 0;
        for (auto it = waypoints.begin(); it != waypoints.end()-1; ++it, ++index_segment)
        // calculation of H,T for all segments
        {
            T(index_segment) = (*(it+1))(0) - (*it)(0);
            H.col(index_segment) = (*(it+1)).segment(1, n_dof) - (*it).segment(1, n_dof);
        }

        /* computation of intermediary velocities*/
        Eigen::MatrixXd Vn; Vn.resize(n_dof, n_segment+1); // velocities at each waypoint for all dof of variables
        Vn.setZero(); // initial and final velocities set to be zero
        
        int index_dof = 0;
        for (; index_dof < n_dof; ++index_dof)
        // calculation of intermediary velocities for all segments and all dof
        {
            Eigen::VectorXd vel = computeIntVelSpline(T, H.row(index_dof), n_segment);
            Vn.block(index_dof, 1, 1, n_segment-1) = vel.transpose();
        }

        /* computation of coefficients*/
        for(index_segment = 0; index_segment < n_segment; ++index_segment)
        // calculation of coefficients for every segment and every dof
        {
            Eigen::MatrixXd coeff_i; coeff_i.resize(n_dof,4);

            coeff_i.col(0) = waypoints.at(index_segment).segment(1,n_dof); // p_k0 = q*_{k-1}
            coeff_i.col(1) = Vn.col(index_segment); // p_k1 = v_{k-1}

            double Tk = T(index_segment);
            coeff_i.col(2) = 1.0/Tk*((3.0/Tk)*H.col(index_segment) - 2*Vn.col(index_segment) - Vn.col(index_segment+1));
            // p_k2 = 1/Tk *(3/Tk*(q*_k - q*_{k-1}) - 2* v_{k-1} - v_k)
            coeff_i.col(3) = 1.0/(Tk*Tk)*((2.0/Tk)*(-H.col(index_segment)) + Vn.col(index_segment) + Vn.col(index_segment+1));
            // p_k3 = 1/(Tk^2)*(2/Tk*(q*_{k-1} - q*_k) + v_{k-1} + v_k)

            coeff.push_back(coeff_i); // stock of coefficients segment-by-segment
        }
        interpolated = true;
    }
    else if (traj_type == TrajectoryType::Poly3order || traj_type == TrajectoryType::Poly5order || traj_type == TrajectoryType::Poly7order)
    {
        for(auto it = waypoints.begin(); it != waypoints.end()-1; ++it) // loop for all segments
        {
            double t0 = (*it)(0); // start time for a segment
            double tf = (*(it+1))(0); // final time for the current segment
            double dt = tf - t0;

            if (traj_type == TrajectoryType::Poly3order)
            {
                Eigen::VectorXd temp = computeCoeff3order(dt);
                coeff.push_back(temp);
                continue;
            }
            if (traj_type == TrajectoryType::Poly5order)
            {
                Eigen::VectorXd temp = computeCoeff5order(dt);
                coeff.push_back(temp);
                continue;
            }
            if (traj_type == TrajectoryType::Poly7order)
            {
                Eigen::VectorXd temp = computeCoeff7order(dt);
                coeff.push_back(temp);
                continue;
            }
        }
        interpolated = true;       
    }    
    else
    {
        msgerr = "Trajectory type error";
        return false;  
    }  
    return true;
}

void TrajectoryBase::getInterpValue(Eigen::VectorXd& _pos, Eigen::VectorXd& _vel, Eigen::VectorXd& _acc, const double _tNow)
// function called at run time to get interpolated value at time _tNow
{
    if(!initialized)
    {
        _pos.setZero(n_dof);
        _vel.setZero(n_dof);
        _acc.setZero(n_dof);
        return;
    }
    if(_tNow > t_fin) // if trajectory finished, set the last waypoint as desired value
    {
        _pos = waypoints.back().segment(1, n_dof);
        _vel.setZero(n_dof);
        _acc.setZero(n_dof);
        return;
    }

    double t_traj = _tNow - t_init;

    static const int n_segment = n_wps-1;
    static int segment_i = 0; // using static variable to be able to keep last semgent index at run time
    
    if(t_traj < waypoints.at(1)(0)) // if current time remains in the first segment, reset segment_i variable
        segment_i = 0;
    
    for (; segment_i < n_segment; segment_i++)
    // test the current time belonging to which segment
    {
        double t0 = waypoints.at(segment_i)(0);
        double tf = waypoints.at(segment_i+1)(0);
        if(segment_i == n_segment-1) // break directly if last segment
            break;
        else
        {
            if(t_traj >= t0 && t_traj < tf)
                break;
        }
    }

    double dT = t_traj - waypoints.at(segment_i)(0); // time period in the current segment (start time as 0)

    switch (traj_type)
    {
    case TrajectoryType::Spline:
    {
        Eigen::MatrixXd coeff_i = coeff.at(segment_i); // coefficient for the current segment
        _pos = coeff_i.col(0) + dT*coeff_i.col(1) + dT*dT*coeff_i.col(2) + dT*dT*dT*coeff_i.col(3);
        _vel = coeff_i.col(1) + 2*dT*coeff_i.col(2) + 3*dT*dT*coeff_i.col(3);
        _acc = 2*coeff_i.col(2) + 6*dT*coeff_i.col(3);
        break;
    }
    case TrajectoryType::Poly3order:
    {
        Eigen::Matrix<double, 4, 1> coeff_i = coeff.at(segment_i); // coefficient for the current segment
        double pt = coeff_i(0) + coeff_i(1)*dT + coeff_i(2)*dT*dT + coeff_i(3)*dT*dT*dT;
        double dpt = coeff_i(1) + 2*coeff_i(2)*dT + 3*coeff_i(3)*dT*dT;
        double ddpt = 2*coeff_i(2) + 6*coeff_i(3)*dT;

        Eigen::VectorXd q0 = waypoints.at(segment_i).segment(1,n_dof);
        Eigen::VectorXd qf = waypoints.at(segment_i+1).segment(1,n_dof); 
        // q0, qf for the current segment

        _pos = q0 + pt*(qf-q0);
        _vel = dpt*(qf-q0);
        _acc = ddpt*(qf-q0);
        break;
    }
    case TrajectoryType::Poly5order:
    {
        Eigen::Matrix<double, 6, 1> coeff_i = coeff.at(segment_i); // coefficient for the current segment
        double pt = coeff_i(0) + coeff_i(1)*dT + coeff_i(2)*dT*dT + coeff_i(3)*dT*dT*dT + coeff_i(4)*pow(dT,4) + coeff_i(5)*pow(dT,5);
        double dpt = coeff_i(1) + 2*coeff_i(2)*dT + 3*coeff_i(3)*dT*dT + 4*coeff_i(4)*pow(dT,3) + 5*coeff_i(5)*pow(dT,4);
        double ddpt = 2*coeff_i(2) + 6*coeff_i(3)*dT + 12*coeff_i(4)*dT*dT + 20*coeff_i(5)*dT*dT*dT;

        Eigen::VectorXd q0 = waypoints.at(segment_i).segment(1,n_dof);
        Eigen::VectorXd qf = waypoints.at(segment_i+1).segment(1,n_dof); 
        // q0, qf for the current segment

        _pos = q0 + pt*(qf-q0);
        _vel = dpt*(qf-q0);
        _acc = ddpt*(qf-q0);
        break;
    }
    case TrajectoryType::Poly7order:
    {
        Eigen::Matrix<double, 8, 1> coeff_i = coeff.at(segment_i); // coefficient for the current segment
        double pt = coeff_i(0) + coeff_i(1)*dT + coeff_i(2)*dT*dT + coeff_i(3)*dT*dT*dT + coeff_i(4)*pow(dT,4) + coeff_i(5)*pow(dT,5) 
                    + coeff_i(6)*pow(dT,6) + coeff_i(7)*pow(dT,7);
        double dpt = coeff_i(1) + 2*coeff_i(2)*dT + 3*coeff_i(3)*dT*dT + 4*coeff_i(4)*pow(dT,3) + 5*coeff_i(5)*pow(dT,4)
                    + 6*coeff_i(6)*pow(dT,5) + 7*coeff_i(7)*pow(dT,6);
        double ddpt = 2*coeff_i(2) + 6*coeff_i(3)*dT + 12*coeff_i(4)*dT*dT + 20*coeff_i(5)*dT*dT*dT + 30*coeff_i(6)*pow(dT,4) + 42*coeff_i(7)*pow(dT,5);

        Eigen::VectorXd q0 = waypoints.at(segment_i).segment(1,n_dof);
        Eigen::VectorXd qf = waypoints.at(segment_i+1).segment(1,n_dof); 
        // q0, qf for the current segment

        _pos = q0 + pt*(qf-q0);
        _vel = dpt*(qf-q0);
        _acc = ddpt*(qf-q0);
        break;
    }
    
    default:
        break;
    }
}
/*******************************************************************************/

/************************** Loading desired waypoints from txt file ****************************/
template <typename T> void getDataFromString(const std::string& string_line, std::vector<T>& ret)
{
    ret.clear();
    std::istringstream s(string_line);
    T d;
    while (s >> d) {
        ret.push_back(d);
    }
    return;
}

bool TrajectoryBase::loadWaypointFromTxtFile(const std::string &file_name, std::string &msgerr)
{
    std::ifstream fin(file_name.c_str());
    if(!fin.is_open())
    {
        msgerr = "Error opening file: " + file_name;
        return false;
    }

    std::string s_line;
    int data_size = n_dof+1; // first column refers to time stamp
    
    while(!fin.eof())
    {
        std::getline(fin, s_line);
        if(s_line.empty()) break; // check empty line at the end

        std::vector<double> data;
        getDataFromString(s_line, data);

        if(data.size() != data_size) // check data_size 
        {
            char msg[100];
            sprintf(msg, "Data size(%d) incoherent (must be %d)", int(data.size()), data_size);
            msgerr = msg;
            return false;
        }
        Eigen::VectorXd wp; wp.resize(data_size);
        for (int i=0; i < data_size; i++) // stock in Eigen vector
            wp(i) = data.at(i);
        
        waypoints.push_back(wp);
        n_wps++;
    }
    fin.close();
    return true;
}
/*******************************************************************************/
