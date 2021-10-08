/* Trajectory Server v2                        */
/* Author: Lorenzo Gentilini                   */
/* E-Mail: lorenzo.gentilini6@unibo.it         */
/* Date: August 2020                           */
/* File: trajectory_server.cpp                 */

#include <trajectory_server/trajectory_server.hpp>

TrajectoryServer::TrajectoryServer(ros::NodeHandle& nh):
    planPolyTrajectory(nh.advertiseService("/plan_poly_trajectory", &TrajectoryServer::planPoly, this)){

    nh.param("poly/order", polyOrder, 9);
    nh.param("poly/max_velocity", velMaxDefault, 2.5);
    nh.param("poly/max_acceleration", accMaxDefault, 2.5);
    nh.param("poly/derivate_to_optimize", optDerivate, 4);

    nh.param("poly/relative_step_size", initStepSize_rel, 0.1);
    nh.param("poly/max_iterations", maxIterations, 1000);
    nh.param("poly/obj_rel_value", fRelative, 0.05);
    nh.param("poly/obj_abs_value", fAbsolute, -1.0);
    nh.param("poly/state_rel_value", xRelative, -1.0);
    nh.param("poly/state_abs_value", xAbsolute, -1.0);
    nh.param("poly/random_seed", randomSeed, 0.0);

    nh.param("poly/time_gain", timeGain, 1000.0);
    nh.param("poly/soft_constraints_gain", softConstraintsGain, 10.0);
    nh.param("poly/soft_constraints_epsilon", epsilon, 0.1);
}   

// Helper Functions
std::vector<double> TrajectoryServer::setTimes(){
    std::vector<double> times;
    times.resize(wayPoints->pose.size()-1);

    for(uint i = 0; i < wayPoints->pose.size()-1; i++){
        times[i] = wayPoints->getDistance(i, i+1)/(0.5*velMax);
    }

    return times;
}

void TrajectoryServer::linearOptimize(std::vector<double>& times){
    Polynomial* polynomial = new Polynomial(polyOrder);

    Eigen::MatrixXd A, Q, df;
    Eigen::SparseMatrix<double> C;
    A.resize((polyOrder+1)*times.size(), (polyOrder+1)*times.size());
    Q.resize((polyOrder+1)*times.size(), (polyOrder+1)*times.size());

    A.setZero();
    Q.setZero();

    for(uint i = 0; i < times.size(); i++){
        Eigen::MatrixXd Qi, Ai, invAi;

        // Compute Quadratic Cost Matrix
        computeQuadraticCostMatrix(polynomial, times[i], Qi);

        // Compute Mapping Matrix
        computeMappingMatrix(polynomial, times[i], Ai);

        //Update Q and A
        A.block(i*Ai.rows(), i*Ai.cols(), Ai.rows(), Ai.cols()) = Ai;
        Q.block(i*Qi.rows(), i*Qi.cols(), Qi.rows(), Qi.cols()) = Qi;

        if(i != times.size()-1){
            A.block((i+1)*Ai.rows(), i*Ai.cols(), Ai.rows()/2, Ai.cols()) = -Ai.block(Ai.rows()/2, 0, Ai.rows()/2, Ai.cols());
        }
    }

    Eigen::MatrixXd invA = A.fullPivHouseholderQr().inverse();

    // Compute Reordering Matrix
    computeReorderingMatrixAndConstraints(C, df);

    // Compute R Matrix
    Eigen::MatrixXd R = C*invA.transpose()*Q*invA*C.transpose();
    Eigen::MatrixXd Rfp = R.block(0, wayPoints->fixedConstraints.size(),
                                  wayPoints->fixedConstraints.size(), wayPoints->freeConstraints.size());
    Eigen::MatrixXd Rpp = R.block(wayPoints->fixedConstraints.size(), wayPoints->fixedConstraints.size(),
                                  wayPoints->freeConstraints.size(), wayPoints->freeConstraints.size());

    // Optimize
    Eigen::MatrixXd dp = -Rpp.fullPivHouseholderQr().inverse()*Rfp.transpose()*df;

    uint index = 0;
    for(uint i = 0; i < wayPoints->freeConstraints.size(); i++){
        if(wayPoints->freeConstraints[i].value(0) == CONSTR_NULL){
            wayPoints->freeConstraints[i].value = dp.row(index);
            
            for(uint j = 0; j < wayPoints->allConstraints.size(); j++)
                if(wayPoints->freeConstraints[i] == wayPoints->allConstraints[j])
                    wayPoints->allConstraints[j].value = dp.row(index);
            
            index++;
        }
    }


    delete polynomial;
}

void TrajectoryServer::computeQuadraticCostMatrix(Polynomial* polynomial, double time, Eigen::MatrixXd& Qi){
    Qi.resize(polyOrder+1, polyOrder+1);
    Qi.setZero();

    for(uint col = 0; col < polyOrder + 1 - optDerivate; col++){
        for(uint row = 0; row < polyOrder + 1 - optDerivate; row++){
            double exp = (polyOrder - optDerivate)*2 + 1 - row - col;

            Qi(polyOrder - row, polyOrder - col) = polynomial->getBaseCoefficient(optDerivate, polyOrder - row)*
                                                   polynomial->getBaseCoefficient(optDerivate, polyOrder - col)*
                                                   pow(time, exp)*2/exp;

        }
    }
}

void TrajectoryServer::computeMappingMatrix(Polynomial* polynomial, double time, Eigen::MatrixXd& Ai){
    Ai.resize(polyOrder+1, polyOrder+1);
    Ai.setZero();

    // Creates as A = [A(0); A(Ti)]
    for(uint i = 0; i < (polyOrder+1)/2; i++){
        Ai.row(i) = polynomial->getBaseWithTime(i, 0.0);
        Ai.row(i + (polyOrder+1)/2) = polynomial->getBaseWithTime(i, time);
    }
}

void TrajectoryServer::computeReorderingMatrixAndConstraints(Eigen::SparseMatrix<double>& C, Eigen::MatrixXd& df){
    std::vector<Eigen::Triplet<double>> tripleList;

    C.resize(wayPoints->allConstraints.size(), wayPoints->allConstraints.size());
    tripleList.reserve(wayPoints->allConstraints.size());
    df.resize(wayPoints->fixedConstraints.size(), 3);

    for(uint i = 0; i < wayPoints->allConstraints.size(); i++){
        for(uint j = 0; j < wayPoints->fixedConstraints.size(); j++)
            if(wayPoints->allConstraints[i] == wayPoints->fixedConstraints[j]){
                tripleList.emplace_back(Eigen::Triplet<double>(j, i, 1.0));
                df.row(j) =  wayPoints->fixedConstraints[j].value;
            }

        for(uint k = 0; k < wayPoints->freeConstraints.size(); k++)
            if(wayPoints->allConstraints[i] == wayPoints->freeConstraints[k])
                tripleList.emplace_back(Eigen::Triplet<double>(k + wayPoints->fixedConstraints.size(), i, 1.0));
    }

    C.setFromTriplets(tripleList.begin(), tripleList.end());
}

void TrajectoryServer::nonLinearOptimize(std::vector<double>& times){
    uint varNumber = (wayPoints->freeConstraints.size()*3) + times.size();

    // Set Up Initial Solution and Bounds
    std::vector<double> initialSolution, initialStepSize,
                        lowerBounds, upperBounds;

    initialSolution.reserve(varNumber);
    initialStepSize.reserve(varNumber);
    lowerBounds.reserve(varNumber);
    upperBounds.reserve(varNumber);

    for(uint i = 0; i < times.size(); i++){
        initialSolution.push_back(times[i]);
        lowerBounds.push_back(0.0);
        upperBounds.push_back(times[i]);

        if(abs(times[i]) <= std::numeric_limits<double>::lowest()){
            initialStepSize.push_back(1e-13);
        } else{
            initialStepSize.push_back(initStepSize_rel*abs(times[i]));
        }
    }

    for(uint i = 0; i < wayPoints->freeConstraints.size(); i++){
        for(uint j = 0; j < 3; j++){
            initialSolution.push_back(wayPoints->freeConstraints[i].value(j));

            if(wayPoints->freeConstraints[i].derivID == 1){
                lowerBounds.push_back(-velMax);
                upperBounds.push_back(velMax);
            } else if(wayPoints->freeConstraints[i].derivID == 2){
                lowerBounds.push_back(-accMax);
                upperBounds.push_back(accMax);
            } else{
                lowerBounds.push_back(-std::numeric_limits<double>::max());
                upperBounds.push_back(std::numeric_limits<double>::max());
            }

            if(abs(wayPoints->freeConstraints[i].value(j)) <= std::numeric_limits<double>::lowest()){
                initialStepSize.push_back(1e-13);
            } else{
                if(wayPoints->freeConstraints[i].value(j) != 0){
                    initialStepSize.push_back(initStepSize_rel*abs(wayPoints->freeConstraints[i].value(j)));
                } else{
                    initialStepSize.push_back(initStepSize_rel);
                }
            }
        }
    }

    std::cout << "VEL: " << velMax << std::endl;
    std::cout << "ACC: " << accMax << std::endl;
    
    // Define Optimization Object From NLopt Library
    nlopt::opt opt(/*nlopt::LN_COBYLA*/ /*LN_BOBYQA*/ nlopt::LN_SBPLX, varNumber);
    opt.set_min_objective(TrajectoryServer::J, this);
    opt.set_initial_step(initialStepSize);
    opt.set_maxeval(maxIterations);
    opt.set_ftol_rel(fRelative);
    opt.set_ftol_abs(fAbsolute);
    opt.set_xtol_rel(xRelative);
    opt.set_xtol_abs(xAbsolute);
    opt.set_lower_bounds(lowerBounds);
    opt.set_upper_bounds(upperBounds);

    if(randomSeed < 0)
        nlopt_srand_time();
    else
        nlopt_srand(randomSeed);

    double minimumCost;
    try{
        opt.optimize(initialSolution, minimumCost);
        std::cout << "Found Minimum" << std::endl;

        // Update Waypoints & Times
        for(uint i = 0; i < times.size(); i++){
            times[i] = initialSolution[i];
        }

        uint index = 0;
        for(uint i = 0; i < wayPoints->allConstraints.size(); i++){
            for(uint j = 0; j < wayPoints->freeConstraints.size(); j++){
                if(wayPoints->allConstraints[i] == wayPoints->freeConstraints[j]){
                    for(uint k = 0; k < 3; k++){
                        wayPoints->allConstraints[i].value(k) = initialSolution[times.size() + index + k];
                    }

                    index += 3;
                    break;
                }
            }

            if(wayPoints->allConstraints[i].occurID == 1){
                uint ii = i - (polyOrder+1)/2;
                wayPoints->allConstraints[i].value = wayPoints->allConstraints[ii].value;
            }
        }
    }
    catch(std::exception &e){
        std::cout << "Optimization Failed: " << e.what() << std::endl;
    }
}

// Compute Cost
double TrajectoryServer::J(const std::vector<double>& x, std::vector<double>& grad, void* f_data){
    TrajectoryServer* opt = reinterpret_cast<TrajectoryServer*>(f_data);

    double actualCost;
    opt->computeCost(x, grad, actualCost);

    return actualCost;
}

void TrajectoryServer::computeCost(const std::vector<double>& x, std::vector<double>& grad, double& cost){
    Eigen::MatrixXd derivates;
    std::vector<double> times;

    double timeSize = x.size() - wayPoints->freeConstraints.size()*3;
    double derivatesSize = wayPoints->allConstraints.size();

    times.reserve(timeSize);
    derivates.resize(derivatesSize, 3);
    derivates.setZero();

    uint index = 0;
    for(index = 0; index < timeSize; index++){
        times.push_back(x[index]);
    }

    uint row = 0;
    for(uint i = 0; i < wayPoints->allConstraints.size(); i++){
        if(wayPoints->allConstraints[i].occurID != 0){
            derivates.row(row) = derivates.row(row-(optDerivate + 1)); // MODIFIED
            row++;

            continue;
        }

        bool found = false;
        for(uint j = 0; j < wayPoints->freeConstraints.size(); j++){
            if(wayPoints->allConstraints[i] == wayPoints->freeConstraints[j]){
                derivates.row(row) = Eigen::Matrix<double, 3, 1>(x[index], x[index+1], x[index+2]);
                
                index += 3;
                found = true;

                break;
            }
        }

        if(!found){
            derivates.row(row) = wayPoints->allConstraints[i].value;
        }

        row++;
    }

    // Define Polynomials
    std::vector<Polynomial*> polys_x(wayPoints->pose.size() - 1),
                             polys_y(wayPoints->pose.size() - 1),
                             polys_z(wayPoints->pose.size() - 1);

    for(uint i = 0; i < polys_x.size(); i++){
        std::vector<double> startDerivates_x((polyOrder + 1)/2), endDerivates_x((polyOrder + 1)/2),
                            startDerivates_y((polyOrder + 1)/2), endDerivates_y((polyOrder + 1)/2),
                            startDerivates_z((polyOrder + 1)/2), endDerivates_z((polyOrder + 1)/2);

        for(uint j = 0; j < (polyOrder + 1)/2; j++){
            startDerivates_x[j] = derivates(i*(polyOrder + 1) + j, 0);
            startDerivates_y[j] = derivates(i*(polyOrder + 1) + j, 1);
            startDerivates_z[j] = derivates(i*(polyOrder + 1) + j, 2);

            endDerivates_x[j] = derivates(i*(polyOrder + 1) + (polyOrder + 1)/2 + j, 0);
            endDerivates_y[j] = derivates(i*(polyOrder + 1) + (polyOrder + 1)/2 + j, 1);
            endDerivates_z[j] = derivates(i*(polyOrder + 1) + (polyOrder + 1)/2 + j, 2);
        }
        
        polys_x[i] = new Polynomial(polyOrder, 0, times[i], startDerivates_x, endDerivates_x);
        polys_y[i] = new Polynomial(polyOrder, 0, times[i], startDerivates_y, endDerivates_y);
        polys_z[i] = new Polynomial(polyOrder, 0, times[i], startDerivates_z, endDerivates_z);
    }

    // Compute Cost Function
    Eigen::MatrixXd A, Q;

    A.resize((polyOrder+1)*times.size(), (polyOrder+1)*times.size());
    Q.resize((polyOrder+1)*times.size(), (polyOrder+1)*times.size());
    A.setZero();
    Q.setZero();

    for(uint i = 0; i < times.size(); i++){
        Eigen::MatrixXd Qi, Ai, invAi;

        // Compute Quadratic Cost Matrix
        computeQuadraticCostMatrix(polys_x[0], times[i], Qi);

        // Compute Mapping Matrix
        computeMappingMatrix(polys_x[0], times[i], Ai);

        //Update Q and A
        A.block(i*Ai.rows(), i*Ai.cols(), Ai.rows(), Ai.cols()) = Ai;
        Q.block(i*Qi.rows(), i*Qi.cols(), Qi.rows(), Qi.cols()) = Qi;
    }

    Eigen::MatrixXd invA = A.fullPivHouseholderQr().inverse();
    Eigen::MatrixXd matrixTrajectory = derivates.transpose()*invA.transpose()*Q*invA*derivates;

    //ROS_INFO_STREAM("TEST: " << derivates);

    double costTrajectory = 0.0;
    for(uint i = 0; i < matrixTrajectory.rows(); i++)
        for(uint j = 0; j < matrixTrajectory.cols(); j++)
            if(i == j)
                costTrajectory += matrixTrajectory(i, j);

    double totalTime = 0.0, costTime = 0.0;
    for(uint i = 0; i < times.size(); i++)
        totalTime += times[i];

    costTime = timeGain*pow(totalTime, 2);

    // Compute Maximum Velocity and Acceleration
    std::vector<double> vel, acc;
    for(uint i = 0; i < polys_x.size(); i++){
       Eigen::MatrixXd velCoeffs = polys_x[i]->convolve(1, 2) + polys_y[i]->convolve(1, 2) + polys_z[i]->convolve(1, 2);
       Eigen::MatrixXd accCoeffs = polys_x[i]->convolve(2, 3) + polys_y[i]->convolve(2, 3) + polys_z[i]->convolve(2, 3);

        Polynomial velocity(velCoeffs), acceleration(accCoeffs);
        std::pair<double, double> minimum_v, minimum_a, maximum_v, maximum_a;
        velocity.computeMinMax(0, times[i], 0, minimum_v, maximum_v);
        acceleration.computeMinMax(0, times[i], 0, minimum_a, maximum_a);

        vel.push_back(abs(minimum_v.second));
        vel.push_back(abs(maximum_v.second));
        acc.push_back(abs(minimum_a.second));
        acc.push_back(abs(maximum_a.second));
    }

    std::vector<double>::iterator maximumVelocity = std::max_element(vel.begin(), vel.end());
    std::vector<double>::iterator maximumAcceleration = std::max_element(acc.begin(), acc.end());

    double costSoftConstraints = exp((*maximumVelocity - velMax)*(softConstraintsGain/(velMax*epsilon))) +
                                 exp((*maximumAcceleration - accMax)*(softConstraintsGain/(accMax*epsilon)));

    // Total Cost
    cost = costTrajectory + costTime + costSoftConstraints;

    // Delete Objects
    for(uint i = 0; i < polys_x.size(); i++){
        delete polys_x[i];
        delete polys_y[i];
        delete polys_z[i];
    }
}

// Services
bool TrajectoryServer::planPoly(trajectory_server::PlanPolyTraj::Request& request,
                                trajectory_server::PlanPolyTraj::Response& response){
    ros::Time startTime = ros::Time::now();
    if(request.v_max > 0.0){
        velMax = request.v_max;
    }else{
        velMax = velMaxDefault;
    }

    if(request.a_max > 0.0){
        accMax = request.a_max;
    }else{
        accMax = accMaxDefault;
    }
    wayPoints = new WayPoint(request.wpoints.size(), polyOrder);

    // Set Up wayPoints
    for(uint i = 0; i < request.wpoints.size(); i++){
        Eigen::Matrix<double, 3, 1> p(request.wpoints[i].x, request.wpoints[i].y, request.wpoints[i].z);
        wayPoints->pose.push_back(p);

        uint constraintOccurence = 2;
        if(i == 0 || i == request.wpoints.size()-1)
            constraintOccurence = 1;
    
        for(int c = 0; c < constraintOccurence; c++){
            Constraint constraint;
            constraint.pointID = i;
            constraint.derivID = 0;
            constraint.occurID = c;
            constraint.value = p;

            if(c == 1){
                constraint.value = Eigen::Matrix<double, 3, 1>(0.0, 0.0, 0.0);
            }

            wayPoints->allConstraints.push_back(constraint);
            wayPoints->fixedConstraints.push_back(constraint);

            uint k = 1;
            for(uint j = 0; j < request.wpoints[i].constraints.size(); j++){
                Eigen::Matrix<double, 3, 1> derConstraint(request.wpoints[i].constraints[j].x,
                                                          request.wpoints[i].constraints[j].y,
                                                          request.wpoints[i].constraints[j].z);
                constraint.pointID = i;
                constraint.derivID = j+1;
                constraint.occurID = c;
                constraint.value = derConstraint;

                wayPoints->allConstraints.push_back(constraint);
                wayPoints->fixedConstraints.push_back(constraint);

                k++;
            }

            if(k < (polyOrder+1)/2){
                for(uint index = k; index < (polyOrder+1)/2; index++){
                    constraint.pointID = i;
                    constraint.derivID = index;
                    constraint.occurID = c;
                    constraint.value = Eigen::Matrix<double, 3, 1>(CONSTR_NULL, CONSTR_NULL, CONSTR_NULL);
        
                    if(c == 1){
                        constraint.value = Eigen::Matrix<double, 3, 1>(0.0, 0.0, 0.0);
                        wayPoints->fixedConstraints.push_back(constraint);
                    }else{
                        wayPoints->freeConstraints.push_back(constraint);
                    }

                    wayPoints->allConstraints.push_back(constraint);
                }
            }
        }
    }


    // Define Time Vector
    std::vector<double> times = setTimes();

    std::cout << "BEFORE: " << times[0] << " " << std::endl;

    if(wayPoints->freeConstraints.size() > 0){
        // Linear Optimization
        linearOptimize(times);
    }

    // Non-Linear Optimization
    nonLinearOptimize(times);

    std::cout << "AFTER: " << times[0] << std::endl;

    // Build Answer
    for(uint i = 0; i < times.size(); i++)
        response.times.push_back(times[i]);

    for(uint i = 0; i < wayPoints->allConstraints.size(); i += (polyOrder+1)/2){
        trajectory_server::Waypoint msg;
        msg.x = wayPoints->allConstraints[i].value(0);
        msg.y = wayPoints->allConstraints[i].value(1);
        msg.z = wayPoints->allConstraints[i].value(2);

        for(uint j = 1; j < (polyOrder+1)/2; j++){
            trajectory_server::Constraint msg_c;
            msg_c.x = wayPoints->allConstraints[i + j].value(0);
            msg_c.y = wayPoints->allConstraints[i + j].value(1);
            msg_c.z = wayPoints->allConstraints[i + j].value(2);

            msg.constraints.push_back(msg_c);
        }

        response.wpoints.push_back(msg);
    }

    delete wayPoints;

    std::cout << "Optimizer Time: " << (ros::Time::now()-startTime).toSec() << std::endl;
    return true;
}
