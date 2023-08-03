#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

#include<vector>
#include <math.h>
#include<array>
#include <algorithm>
#include <Eigen/Sparse>
#include <random>

using namespace std;
using namespace Eigen;

typedef array<double, 3> MyPose;
typedef vector<MyPose> belief;

// parameters for controller
struct parameters{
    vector<double> obstacle_pos; // center of obstacle
    double obstacle_radius; // radius of obstacle
    double alpha; // CVaR level
    double delta; // Confidence of correcntess of CVaR upper bound
    double b_max; // max h value
    int nx; // x dim
    int nu; // u dim
    double r_robot; // collision radius robot
    double d; // relative displacement of point used for collision detection
    bool input_bounds;
    bool use_orientation_CBF;
};

// Euclidean distance
double euclidean_dist(MyPose pose1, MyPose pose2);

// Sorting operation with permutation
vector<pair<double, int>> sortVec(vector<double> vec);

// Vector Field f
VectorXd f_x(MyPose p);

// input affine mapping g_x
MatrixXd g_x(MyPose p);

// drift term
MatrixXd sigma_x(MyPose p);

#endif // HELPER_FUNCTIONS_H
