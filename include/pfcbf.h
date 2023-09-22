#include "helper_functions.h"

#include <osqp/osqp.h>
#include <OsqpEigen/OsqpEigen.h>


class pfcbf {
    private:
        // parameters
        parameters* params;
        //QP solver
        OsqpEigen::Solver solver;
        // Lie derivative
        Eigen::SparseMatrix<double> Lgh;
        // constraints
        VectorXd lbAconstr;
        VectorXd ubAconstr;
        std::mt19937 gen;
        // control inputs
        VectorXd last_u;
        VectorXd u_ref;
    
    public:
        pfcbf(parameters* params_, belief init_belief); // constructor
        belief pf_belief;
        VectorXd get_control(); // generate risk-aware controls
        double evaluate_h_x(MyPose pose); // State CBF
        VectorXd gradient_h_x(MyPose pose); // gradient of state CBF
        MatrixXd hessian_h_x(MyPose pose); // hessian of state CBF
        double evaluate_h_orientation_x(MyPose pose); // State CBF
        VectorXd gradient_h_orientation_x(MyPose pose); // gradient of state CBF
        MatrixXd hessian_h_orientation_x(MyPose pose); // hessian of state CBF
        void updateParticleBelief(belief p_new){
            pf_belief = p_new;
        };
        void propagateParticles();
        void sample_motion_model(MyPose& p, VectorXd u, double dt);
        void set_reference(VectorXd u){
            u_ref = u;
        };
        int NumberOfParticles(){
            return pf_belief.size();
        };
};