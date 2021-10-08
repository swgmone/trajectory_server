/* Polynomial Handler                          */
/* Author: Lorenzo Gentilini                   */
/* E-Mail: lorenzo.gentilini6@unibo.it         */
/* Date: August 2020                           */
/* File: poly_lib.hpp                          */

#ifndef _POLY_H
#define _POLY_H

#include "rpoly/rpoly_ak1.hpp"

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <vector>
#include <iostream>

class Polynomial{
    private:
    // Attributes
    int polyOrder = 0;

    Eigen::VectorXd polyCoeff;
    Eigen::MatrixXd baseCoeff;

    // Helper Functions
    Eigen::MatrixXd computeBaseCoefficients();
    bool computeMinMaxCandidates(double startTime, double endTime, int derivate,
                                 std::vector<double>& candidates);
    bool selectCandidatesFromRoots(double startTime, double endTime, Eigen::VectorXcd roots,
                                   std::vector<double>& candidates);
    bool selectMinMaxFromCandidates(std::vector<double>& candidates, int derivate,
                                    std::pair<double, double>& minimum,
                                    std::pair<double, double>& maximum);

    public:
    // Constructor
    Polynomial(int order);
    Polynomial(Eigen::VectorXd coeffs);
    Polynomial(int order, double startTime, double endTime,
               std::vector<double> startTimeConstraints,
               std::vector<double> endTimeConstraints);

    // Destructor
    ~Polynomial(){;}

    // Setting Functions
    void setCoefficients(Eigen::VectorXd coeffs);

    // Returning Functions
    Eigen::VectorXd getCoefficients(uint derivate);
    double getBaseCoefficient(int derivate, int index);
    Eigen::VectorXd getBaseWithTime(uint derivate, double time);
    double evaluate(uint derivate, double time);
    bool computeMinMax(double startTime, double endTime, int derivate,
                       std::pair<double, double>& minimum,
                       std::pair<double, double>& maximum);
    Eigen::VectorXd convolve(uint derivate, uint kernelDerivate);
    bool getRoots(int derivate, Eigen::VectorXcd& roots);
};

#endif


