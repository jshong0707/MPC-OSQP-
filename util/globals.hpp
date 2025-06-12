#ifndef GLOBALS_H_
#define GLOBALS_H_

#include <mujoco/mujoco.h>
#include "F_Kinematics.hpp"
#include "B_Kinematics.hpp"
#include "control_ui.hpp"
#include "Controller.hpp"
#include "Trajectory.hpp"
#include "Actuator.hpp"
#include "Body.hpp"
#include "MPC.hpp"
#include <casadi/casadi.hpp>

// #include <iostream>
#include <eigen-master/Eigen/Core>
#include <eigen-master/Eigen/Dense>
#include <eigen-master/unsupported/Eigen/MatrixFunctions>

// using namespace std;
using namespace Eigen;



// // using namespace casadi;
// // using namespace Ipopt;
// // using namespace ROPTLIB;


#define NDOF_TRUNK 6 // #(DoF) of trunk
#define NDOF_LEG 2   // #(DoF) of leg
#define NUM_LEG 4

/* Data Logging Variables */

extern const double Ts; // sampling period
extern const double g;    // gravitational accel.
#define PI 3.14159265358979323846264
// extern const double PI;








#endif // globals_