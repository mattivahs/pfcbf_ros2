#include "helper_functions.h"

double euclidean_dist(MyPose pose1, MyPose pose2) {
    return sqrt((pose2[0] - pose1[0])*(pose2[0] - pose1[0] )+ (pose2[1] - pose1[1])*(pose2[1] - pose1[1]));
}

// sorting operation
vector<pair<double, int>> sortVec(vector<double> vec){
    vector<pair<double, int>> sorted_vec;

    for (int i=0; i < static_cast<int>(vec.size()); i++){
        sorted_vec.push_back(make_pair(vec[i], i));
    }
    sort(sorted_vec.begin(), sorted_vec.end());
    return sorted_vec;
}

// Unicycle Model
// dx = (f_x(x) + g_x(x)u)dt + sigma_x(x) dW 
VectorXd f_x(MyPose p){
    VectorXd f = VectorXd::Zero(3);
    return f;
}

MatrixXd g_x(MyPose p){
    MatrixXd g = MatrixXd::Zero(3, 2);
    g << cos(p[2]), 0.,
         sin(p[2]), 0.,
         0., 1.;
    return g;
}
// diffusion term
MatrixXd sigma_x(MyPose p){
    MatrixXd sigma = MatrixXd::Zero(3, 2);
    sigma << 0.01, 0.,
             0.01, 0.,
             0., 0.005;
    return sigma;
}

// Single Integrator [x, y, theta] Model with reference control [u,w] given in local frame
VectorXd f_x_SI(MyPose p){
    VectorXd f = VectorXd::Zero(3);
    return f;
}

MatrixXd g_x_SI(MyPose p){
    MatrixXd g = MatrixXd::Zero(3, 2);
    g << cos(p[2]), -sin(p[2]),
         sin(p[2]), cos(p[2]),
         0., 0.;
    return g;
}

MatrixXd sigma_x_SI(MyPose p){
    MatrixXd sigma = MatrixXd::Zero(3, 2);
    sigma << 0.01, 0.,
             0., 0.01,
             0., 0.;
    return sigma;
}

