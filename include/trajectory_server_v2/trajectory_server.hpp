/* Trajectory Server v2                        */
/* Author: Lorenzo Gentilini                   */
/* E-Mail: lorenzo.gentilini6@unibo.it         */
/* Date: August 2020                           */
/* File: trajectory_server.hpp                 */

#include <ros/ros.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <nlopt.hpp>

#include <poly_lib/poly_lib.hpp>
#include <trajectory_server_v2/PlanPolyTraj.h>
#include <trajectory_server_v2/Waypoint.h>
#include <trajectory_server_v2/Constraint.h>

class Constraint{
    #define CONSTR_NULL std::numeric_limits<double>::max()

    public:
    uint pointID = -1;
    uint derivID = -1;
    uint occurID = -1;

    Eigen::Matrix<double, 3, 1> value;

    // Helper Functions
    inline bool operator==(const Constraint& rhs) const{
        return (pointID == rhs.pointID && derivID == rhs.derivID && occurID == rhs.occurID);
    }

    // Constructor
    Constraint(){;}

    // Destructor
    ~Constraint(){;}
};

class WayPoint{ 
    public:
    std::vector<Eigen::Matrix<double, 3, 1>> pose;

    std::vector<Constraint> allConstraints,
                            fixedConstraints,
                            freeConstraints;

    // Helper Functions
    double getDistance(uint index1, uint index2){
        return (pose[index1] - pose[index2]).norm();
    }

    // Constructor
    WayPoint(uint n, uint order){
        uint m = (order+1)/2;

        pose.reserve(n);
        allConstraints.reserve(n*m - m*2);
        fixedConstraints.reserve(n*m - m*2);
        freeConstraints.reserve(n*m - m*2);
    }

    // Destructor
    ~WayPoint(){;}
};


class TrajectoryServer{
    private:
    //Variables
    int polyOrder           = 0,
        optDerivate         = 0,
        maxIterations       = 0;

    double  velMax              = 0.0,
            accMax              = 0.0,
            velMaxDefault       = 0.0,
            accMaxDefault       = 0.0,
            initStepSize_rel    = 0.0,
            fRelative           = 0.0,
            fAbsolute           = 0.0,
            xRelative           = 0.0,
            xAbsolute           = 0.0,
            randomSeed          = 0.0,
            timeGain            = 0.0,
            softConstraintsGain = 0.0,
            epsilon             = 0.0;

    WayPoint* wayPoints;

    // Services
    ros::ServiceServer planPolyTrajectory;

    // Helper Functions
    std::vector<double> setTimes();
    void linearOptimize(std::vector<double>& times);
    void computeQuadraticCostMatrix(Polynomial* polynomial, double time, Eigen::MatrixXd& Qi);
    void computeMappingMatrix(Polynomial* polynomial, double time, Eigen::MatrixXd& Ai);
    void computeReorderingMatrixAndConstraints(Eigen::SparseMatrix<double>& C, Eigen::MatrixXd& dp);

    void nonLinearOptimize(std::vector<double>& times);
    static double J(const std::vector<double>& x, std::vector<double>& grad, void* f_data);
    void computeCost(const std::vector<double>& x, std::vector<double>& grad, double& cost);

    // Services Functions
    bool planPoly(trajectory_server_v2::PlanPolyTraj::Request& request,
                  trajectory_server_v2::PlanPolyTraj::Response& response);

    public:
    // Constructor
    TrajectoryServer(ros::NodeHandle& nh);

    // Destructor
    ~TrajectoryServer(){;}
};
