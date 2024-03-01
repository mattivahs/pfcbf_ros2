#include "pfcbf.h"
#include<iostream>

pfcbf::pfcbf(parameters* params_, belief init_belief){
    // parameters
    params = params_;
    last_u = VectorXd::Zero(params->nu);
    u_ref = VectorXd::Zero(params->nu);

    // Obtain random seed
    std::random_device rd; // Obtain a random seed from the hardware
    std::mt19937 gen_(rd());
    gen = gen_;

    // initial particle belief
    pf_belief = init_belief;

    // setup QP solver
    // number of optimization variables (control dimension)
    solver.data()->setNumberOfVariables(params->nu);
    solver.settings()->setVerbosity(false);
    // number of constraints (barrier constraint + potentially input saturation)
    if (!params->input_bounds){
        solver.data()->setNumberOfConstraints(1);
    }
    else{
        solver.data()->setNumberOfConstraints(3);
    }
    // initialize gradient and hessian
    Eigen::SparseMatrix<double> Hcost_sparse(params->nu, params->nu);
    Hcost_sparse.setIdentity();

    // Set higher weight on deviation from reference velocity v
    // Hcost_sparse.coeffRef(0, 0) = 300;
    Hcost_sparse.coeffRef(0, 0) = 300;
    VectorXd gCost = VectorXd::Zero(params->nu);
    solver.data()->setHessianMatrix(Hcost_sparse);
    solver.data()->setGradient(gCost);

    // initialize constraint matrix
    Eigen::SparseMatrix<double> Lgh_init;
    if (!params->input_bounds){
        Lgh_init.resize(1, params->nu);
        Lgh_init.coeffRef(0, 0) = 0.;
        Lgh_init.coeffRef(0, 1) = 0.;
        Lgh = Lgh_init;

        VectorXd lbAconstr_(1);
        VectorXd ubAconstr_(1);
        ubAconstr_.setConstant(OSQP_INFTY);
        lbAconstr_(0) = 0.;
        lbAconstr = lbAconstr_;
        ubAconstr = ubAconstr_;
    }
    else{
        Lgh_init.resize(3, params->nu);
        Lgh_init.coeffRef(0, 0) = 0.;
        Lgh_init.coeffRef(0, 1) = 0.;
        Lgh_init.coeffRef(1, 0) = 1.;
        Lgh_init.coeffRef(2, 1) = 1.;
        Lgh = Lgh_init;

        VectorXd lbAconstr_(3);
        VectorXd ubAconstr_(3);
        ubAconstr_(0) = OSQP_INFTY;
        ubAconstr_(1) = 0.22;
        ubAconstr_(2) = 2.84;
        lbAconstr_(0) = 0.;
        lbAconstr_(1) = -0.22;
        lbAconstr_(2) = -2.84;
        lbAconstr = lbAconstr_;
        ubAconstr = ubAconstr_;
    }
    solver.data()->setLinearConstraintsMatrix(Lgh);

    // Constraint infty >= Lgh * u >= (-gamma h - Lfh - 0.5 * tr(.))
    solver.data()->setLowerBound(lbAconstr);
    solver.data()->setUpperBound(ubAconstr);
    cout << "init " << lbAconstr << endl;
    if (solver.initSolver()){
        cout << "Inititalized Solver!" << endl;
    }
    else{
        cout << "Error when initializing Solver!" << endl;
    }
}

// Simple Circular avoid area
double pfcbf::evaluate_h_x(MyPose pose){
    return euclidean_dist(pose, MyPose{params->obstacle_pos[0], params->obstacle_pos[1], 0.}) - (params->obstacle_radius + params->r_robot);
}

// gradient of h_x
VectorXd pfcbf::gradient_h_x(MyPose pose){
    VectorXd gradient(3);
    auto d = euclidean_dist(pose, MyPose{params->obstacle_pos[0], params->obstacle_pos[1], 0.});
    gradient << (pose[0] - params->obstacle_pos[0]) / d, (pose[1] - params->obstacle_pos[1]) / d, 0.; 
    return gradient;
}

// hessian of h_x
MatrixXd pfcbf::hessian_h_x(MyPose pose){
    MatrixXd hessian = MatrixXd::Zero(3, 3);
    auto d = euclidean_dist(pose, MyPose{params->obstacle_pos[0], params->obstacle_pos[1], 0.});
    hessian(0, 0) = (d - pow(pose[0] - params->obstacle_pos[0], 2) / d) / pow(d, 2);
    hessian(1, 1) = (d - pow(pose[1] - params->obstacle_pos[1], 2) / d) / pow(d, 2);
    hessian(0, 1) = - ((pose[0] - params->obstacle_pos[0]) * (pose[1] - params->obstacle_pos[1])) / pow(d, 3);
    hessian(1, 0) = hessian(0, 1);
    return hessian;
}

// Reciprocal Stochastic CBF
// Evaluate B(x)
double pfcbf::evaluate_B_x(MyPose pose){
    double h_x = evaluate_h_x(pose);
    if (h_x == 0) {
        // Avoid division by zero
        return std::numeric_limits<double>::infinity();
    }
    return 1.0 / h_x;
}

// Gradient of B(x)
VectorXd pfcbf::gradient_B_x(MyPose pose){
    double h_x = evaluate_h_x(pose);
    VectorXd gradient_h = gradient_h_x(pose);
    VectorXd gradient_B = VectorXd::Zero(3);

    if (h_x == 0) {
        // Handle gradient for h_x = 0 to avoid division by zero
        gradient_B << std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity();
    } else {
        // Compute gradient of B(x) = 1 / h_x(x)
        gradient_B = -1.0 / (h_x * h_x) * gradient_h;
    }
    return gradient_B;
}

MatrixXd pfcbf::hessian_B_x(MyPose pose){
    double h_x = evaluate_h_x(pose);
    VectorXd gradient_h = gradient_h_x(pose);
    MatrixXd hessian_h = hessian_h_x(pose);
    MatrixXd hessian_B = MatrixXd::Zero(3, 3);

    if (h_x == 0) {
        // Handle hessian for h_x = 0 to avoid division by zero
        hessian_B << std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity();
    } else {
        // Compute hessian of B(x) = 1 / h_x(x)
        hessian_B = (2 * gradient_h * gradient_h.transpose()) / (h_x * h_x * h_x) - hessian_h / (h_x * h_x);
    }
    return hessian_B;
}

// CBF over states for unicycle model: small offset d included to offer gradient information w.r.t theta
double pfcbf::evaluate_h_orientation_x(MyPose x){
    double term1 = x[0] + cos(x[2]) * params->d - params->obstacle_pos[0];
    double term2 = x[1] + sin(x[2]) * params->d - params->obstacle_pos[1];
    return sqrt(term1 * term1 + term2 * term2) - (params->obstacle_radius + params->r_robot);
}

// gradient of orientation CBF
VectorXd pfcbf::gradient_h_orientation_x(MyPose x){
    VectorXd gradient(3);

    double term1 = x[0] + cos(x[2]) * params->d - params->obstacle_pos[0];
    double term2 = x[1] + sin(x[2]) * params->d - params->obstacle_pos[1];
    double common_factor = 1.0 / sqrt(term1 * term1 + term2 * term2);

    gradient(0) = common_factor * term1;
    gradient(1) = common_factor * term2;
    gradient(2) = common_factor * params->d * (-sin(x[2]) * term1 + cos(x[2]) * term2);

    return gradient;
}

// hessian of orientation cbf
MatrixXd pfcbf::hessian_h_orientation_x(MyPose x){
    MatrixXd hessian(x.size(), x.size());

    double term1 = x[0] + cos(x[2]) * params->d - params->obstacle_pos[0];
    double term2 = x[1] + sin(x[2]) * params->d - params->obstacle_pos[1];
    double common_factor = 1.0 / pow(term1 * term1 + term2 * term2, 1.5);

    hessian(0, 0) = common_factor * (term1 * term1 - term2 * term2);
    hessian(0, 1) = common_factor * term1 * term2;
    hessian(0, 2) = common_factor * (-params->d * term1 * sin(x[2]) - params->d * term2 * cos(x[2]));

    hessian(1, 0) = hessian(0, 1);
    hessian(1, 1) = common_factor * (term2 * term2 - term1 * term1);
    hessian(1, 2) = common_factor * (params->d * term1 * cos(x[2]) - params->d * term2 * sin(x[2]));

    hessian(2, 0) = hessian(0, 2);
    hessian(2, 1) = hessian(1, 2);
    hessian(2, 2) = common_factor * (-params->d * params->d * (term1 * cos(x[2]) + term2 * sin(x[2])));

    return hessian;
}

VectorXd pfcbf::get_control(){
    // evaluate h_x for all particles -> yields distribution over h_x
    vector<double> h_samples;
    for (auto x_i:pf_belief){
        if (!params->use_orientation_CBF){
            h_samples.push_back(-evaluate_h_x(x_i));
        }
        else{
            h_samples.push_back(-evaluate_h_orientation_x(x_i));
        }
    }
    // number of particles
    int N = h_samples.size();

    // sort h list in ascending order and keep track of permutation
    auto h_sorted = sortVec(h_samples);
    
    // determine which particles are "active" e.g. don't have a zero factor in front of them (see paper)
    vector<double> active_particles;
    double cvar_bar = params->b_max;

    for (int i = 0; i < N; i++){
        active_particles.push_back(max(double(0.), (double(i+1) / double(N)) - sqrt(log(1/params->delta) / double(2 * N)) - (1 - params->alpha)));
    }
    
    // calculate CVaR upper bound
    for (int i = 0; i < N-1; i++){
        cvar_bar -= (1 / params->alpha) * (h_sorted[i+1].first - h_sorted[i].first) * active_particles[i];
    }
    cvar_bar -= (1 / params->alpha) * (params->b_max - h_sorted.back().first) * active_particles.back();

    cvar_bar *= -1;
    double Lfh = 0.;
    MatrixXd trace_term = MatrixXd::Zero(2, 2);
    MatrixXd trace_term_RSCBF = MatrixXd::Zero(2, 2);

    VectorXd Lgh_ = VectorXd::Zero(2);
    double dhdxi = 0.;

    VectorXd fx;
    MatrixXd gx;
    MatrixXd sx;

    for (int i = 0; i < N; i++){
        if (active_particles[i] != 0.){
            MyPose x_i = pf_belief[h_sorted[i].second];
            if (i == 0){
                dhdxi = - (1 / params->alpha) * active_particles[0];
            }
            else{
                dhdxi = (1 / params->alpha) * (active_particles[i-1] - active_particles[i]);
            }

            // determine which model to use
            if (params->use_SI){
                fx = f_x_SI(x_i);
                gx = g_x_SI(x_i);
                sx = sigma_x_SI(x_i);
            }
            else{
                fx = f_x(x_i);
                gx = g_x(x_i);
                sx = sigma_x(x_i);
            }

            if (params->use_RSCBF){
                Lfh += (1 / (cvar_bar * cvar_bar)) * dhdxi * gradient_h_x(x_i).dot(fx);
                Lgh_ += (1 / (cvar_bar * cvar_bar)) * dhdxi * gradient_h_x(x_i).transpose() * gx;
                trace_term += (1 / (cvar_bar * cvar_bar)) * sx.transpose() * hessian_h_x(x_i) * sx - \
                              (2 / pow(cvar_bar, 3)) * sx.transpose() * (pow(dhdxi, 2) * gradient_h_x(x_i) * gradient_h_x(x_i).transpose()) * sx;
    
            }
            // determine which CBF to use
            if (!params->use_orientation_CBF){
                Lfh += - dhdxi * gradient_h_x(x_i).dot(fx);
                Lgh_ += - dhdxi * gradient_h_x(x_i).transpose() * gx;
                trace_term += - sx.transpose() * hessian_h_x(x_i) * sx;
            }
            else{
                Lfh += - dhdxi * gradient_h_orientation_x(x_i).dot(fx);
                Lgh_ += - dhdxi * gradient_h_orientation_x(x_i).transpose() * gx;
                trace_term += - sx.transpose() * hessian_h_orientation_x(x_i) * sx;
            }
        }
    }
    
    if (params->use_RSCBF){
        // Update Lie derivatives
        Lgh.coeffRef(0, 0) = - Lgh_(0);
        Lgh.coeffRef(0, 1) = - Lgh_(1);
        
        Eigen::SparseMatrix<double> UpdatedConstrMatrix = Lgh_.transpose().sparseView();
        
        // update right hand side of barrier constraint
        lbAconstr(0) = -(0.5 * cvar_bar - Lfh - 0.5 * trace_term.trace() - Lgh_.dot(u_ref));
        cout << cvar_bar << endl;
    }
    else{
        // Update Lie derivatives
        Lgh.coeffRef(0, 0) = Lgh_(0);
        Lgh.coeffRef(0, 1) = Lgh_(1);
        
        Eigen::SparseMatrix<double> UpdatedConstrMatrix = Lgh_.transpose().sparseView();
        
        // update right hand side of barrier constraint
        lbAconstr(0) = - 0.5 * cvar_bar - Lfh - 0.5 * trace_term.trace() - Lgh_.dot(u_ref);
    }
    solver.updateLowerBound(lbAconstr);
    solver.updateLinearConstraintsMatrix(Lgh);
    

    if (solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError){
        cout << "Found Solution" << endl;
        auto solution = solver.getSolution();
        // save last control
        last_u = u_ref + solution;
        return u_ref + solution;
    }
    else{
        return u_ref;
    }

}

// propagate particles through motion model
void pfcbf::propagateParticles(){
    for (auto& p:pf_belief){
        sample_motion_model(p, last_u, double(0.02));
    }
}

// Simple Euler-maryuma scheme for propagating samples
void pfcbf::sample_motion_model(MyPose& p, VectorXd u, double dt){
    VectorXd BrownianIncrement(2);
    std::normal_distribution<double> sample_dist(0., dt);
    BrownianIncrement(0) = sample_dist(gen);
    BrownianIncrement(1) = sample_dist(gen);

    VectorXd fx;
    MatrixXd gx;
    MatrixXd sx;

    if (params->use_SI){
        fx = f_x_SI(p);
        gx = g_x_SI(p);
        sx = sigma_x_SI(p);
    }
    else{
        fx = f_x(p);
        gx = g_x(p);
        sx = sigma_x(p);
    }
    
    // Euler Maryuama method for integrating SDE
    VectorXd dx = (fx + gx * u) * dt + sx * BrownianIncrement;
    p[0] += dx(0);
    p[1] += dx(1);
    p[2] += dx(2);
}
