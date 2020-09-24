# gen_traj_lib

## **Description**:
C++ library for trajectory generation embedded in ROS. 

An abstract base class (**TrajectoryBase**) is provided. It is for loading desired waypoints written in .txt file (with a specified format), as well as interpolating data by one of the following methods: 
**cubic spline**, **polynomial order 3**, **polynomial order 5** and **polynomial order 7**. 

Then three options of the derived classes are given: 

- **TrajectoryNd**: Trajectory generation of N-dimensional variables in joint space. 

- **Trajectory3dQuat**: 3D-space trajectory generation, with orientation represented by unit quaternion. 

- **Trajectory3dEuler**: 3D-space trajectory generation, with orientation represented by roll-pitch-yaw Euler angles (equivalent to ZYX convention). 



## **Quick guide**:

- **Examples and documents** (in 'doc/' folder):

   - See 'doc/example_codes.txt' for using instructions of three different derived classes. 

   - Refer to 'doc/trajectory_interpolation.pdf' for details of interpolation methods (cubic spline, polynomial functions of different order).
 
   - References of 3D-space representation of orientation by quaternion or Euler angles can be found in 'doc/3Dspace_representation/' folder.


- **Protocol of waypoints definition**: 

The traveling time for each waypoint must be given in first column, then starting from second column desired waypoints must be written row-by-row. The variables should be split by TAB (one or several is no matter). Example trajectories written in .txt file could be found in 'test/traj/' folder. See 'doc/waypoints_3dspace_euler.png' and 'doc/waypoints_3dspace_quat.png' for the explanation of example trajectories.


- **Matlab scripts and testing** (in 'test/' folder):

   - Matlab scripts of 3d space trajectory generation can be found in 'test/matlab_results/scripts/'. The same methods are verified in Matlab. Go into the Matlab scripts for better understanding of the codes if necessary.

   - Gtest-based test code is stored in 'test/src/'. 3d-space trajectory generation methods for both quaternion and Euler angle representation are validated (for all four trajectory types). N-dimensional trajectory generation is systematically validated since it uses the same interpolation methods without any further processing.


- **Declaration and definition** of classes and methods are in 'include/' and 'src/' folders.


- **Build and import as library**:

   - Run *$catkin build gen_traj_lib* in catkin workspace to build the library (or using another CMake compiler).

   - Then in your CMake-built project, add *find_package(catkin REQUIRED COMPONENTS gen_traj_lib)* in CMakeLists.txt.


## **Detailed Explanation**:

### **Construction of derived class**: 

To use trajectory generation methods, define an object of derived class (abstract base class cannot be instantiated). The trajectory type should be given as argument in the constructor (by the enum **TrajectoryType**). If using the derived class of **TrajectoryNd**, the dimension of the variables must be given as well.


### **Loading waypoints**:
Use **loadWaypointFromTxtFile** method to do that.


### **Interpolating trajectory**: 

Call **doInterpolation** method to interpolate the trajectory based on waypoints loaded. Basically, this function compute simply the coefficients of different segments of trajectory with the given trajectory type. In run time, setting a initial time before going into a loop, and then with a current time given, the desired trajectory will be computed using these coefficients.

### **Setting initial pose**: 

There is a function named **setInitPose** defined in derived classes, allowing to give the initial values of variables before interpolating them. In each derived class, this function is overloaded in a slightly different way. 

   - **TrajectoryNd**: initial values are considerd as an offset. After getting the interpolated values at each timestamp, this offset will be added in the output. So desired waypoints defined in .txt file are relative values with respect to the initial pose.

   - **Trajectory3dQuat**: initial position and attitude are taken as arguments. For position, intial values are considered as an offset, same as above-mentioned. For attitude (defined by unit quaternion), the initial values will replace that of the first waypoint defiend in the first row of .txt file. So attitude waypoints are global values with respect to the world frame (qualisys fixed frame).

   - **Trajectory3dEuler**: same as **Trajectory3dQuat**. The difference is that the attitude is defined by Euler angles. Desired waypoints of Euler angles are global values with respect to the world frame as well.

   **Important**: 

   - Since the attitude waypoints are defined as global values for **Trajectory3dQuat** and **Trajectory3dEuler** objects, if calling **setInitPose** function, **CHECK CAREFULLY** the initial attitude which must be close to your second attitude waypoint (the first one will be replaced by the initial attitude). In other words, desired attitude waypoints should be corehent to the initial attitude of your robot.

   - Setting initial pose is optional, if not doing this, the desired waypoints will become global values with respect to the world frame. So **BE CAREFUL** when defining trajectories.


### **Getting trajectory at a timestamp**:

Call **setInitTime** to set the current timestamp as initial time. Call **getFinalTime** to get the final timestamp at which time the trajectory will finish. Then in control loop, call **getTrajNow** to get desired trajectory at each timestamp. 

Remark: **getTrajNow** function is overloaded in different derived classes, based on the derivation of velocity and acceleration. In **TrajectoryNd**, velocity and acceleration trajectories are computed as the first-order and second-order derivatives of pose trajectory. In **Trajectory3dQuat** and **Trajectory3dEuler**, however, the body-frame angular velocity and acceleration are derived (not the direct derivatives). The transformation from derivatives of attitude variables (rates of quaternion, rates of Euler angles) to the body-frame angular velocity/acceleration is proceeded.
