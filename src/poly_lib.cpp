/* Polynomial Handler                          */
/* Author: Lorenzo Gentilini                   */
/* E-Mail: lorenzo.gentilini6@unibo.it         */
/* Date: August 2020                           */
/* File: poly_lib.cpp                          */

#include "trajectory_server/poly_lib.hpp"

// Constructor
Polynomial::Polynomial(int order):
    polyCoeff(order + 1){
        
    polyCoeff.setZero();
    polyOrder = order;
    baseCoeff = computeBaseCoefficients();
}

Polynomial::Polynomial(Eigen::VectorXd coeffs):
    polyCoeff(coeffs){ 
    
    polyOrder = polyCoeff.size()-1;
    baseCoeff = computeBaseCoefficients();
}

Polynomial::Polynomial(int order, double startTime, double endTime,
                       std::vector<double> startTimeConstraints,
                       std::vector<double> endTimeConstraints):
                       polyCoeff(order + 1){
    
    polyOrder = order;
    baseCoeff = computeBaseCoefficients();

    Eigen::MatrixXd A, b;
    A.resize(polyOrder+1, polyOrder+1);
    b.resize(polyOrder+1, 1);

    A.setZero();
    b.setZero();
    
    for(uint i = 0; i < (polyOrder+1)/2; i++)
        A.row(i) = getBaseWithTime(i, startTime);

    for(uint i = 0; i < (polyOrder+1)/2; i++)
        A.row(i + (polyOrder+1)/2) = getBaseWithTime(i, endTime);

    for(uint i = 0; i < startTimeConstraints.size(); i++)
        b(i) = startTimeConstraints[i];

    for(uint i = 0; i < endTimeConstraints.size(); i++)
        b(i + (polyOrder+1)/2) = endTimeConstraints[i];

    polyCoeff = A.fullPivHouseholderQr().solve(b);
}

// Helper Functions
void Polynomial::setCoefficients(Eigen::VectorXd coeffs){
    if(coeffs.size() != polyOrder+1){
        std::cout << "Order does't Match !!!" << std::endl;
        return;
    }

    polyCoeff = coeffs;
}

Eigen::MatrixXd Polynomial::computeBaseCoefficients(){
    Eigen::MatrixXd coefficients(polyOrder+1, polyOrder+1);

    coefficients.setZero();
    coefficients.row(0).setOnes();

    int degree = polyOrder;
    int order = degree;
    for(int n = 1; n < polyOrder+1; n++){
        for(int i = degree - order; i < polyOrder+1; i++){
            coefficients(n, i) = (order - degree + i)*coefficients(n - 1, i);
        }
        order--;
    }

    return coefficients;
}

bool Polynomial::computeMinMaxCandidates(double startTime, double endTime, int derivate,
                                         std::vector<double>& candidates){
    Eigen::VectorXcd roots;
    bool success = getRoots(derivate + 1, roots);

    if(!success){
        std::cout << "Not Able to Find Roots !!" << std::endl;
        return false;
    } else if(!selectCandidatesFromRoots(startTime, endTime, roots, candidates)){
        std::cout << "Not Able to Select Candidates !!" << std::endl;
        return false;
    }

    return true;
}

bool Polynomial::getRoots(int derivate, Eigen::VectorXcd& roots){
  return findRootsJenkinsTraub(getCoefficients(derivate), roots);
}

bool Polynomial::selectCandidatesFromRoots(double startTime, double endTime, Eigen::VectorXcd roots,
                                           std::vector<double>& candidates){
    if(startTime > endTime){
        std::cout << "Initial Time Greater Than Final !!" << std::endl;
        return false;
    }

    // Start and End Times are Valid Candidates
    candidates.reserve(roots.size() + 2);
    candidates.push_back(startTime);
    candidates.push_back(endTime);

    for(uint i = 0; i < roots.size(); i++){
        // Consider Only Real Roots
        if(abs(roots(i).imag()) > std::numeric_limits<double>::epsilon())
            continue;

        double candidate = roots(i).real();
        if(candidate > startTime && candidate < endTime)
            candidates.push_back(candidate);
    }

    return true;
}

bool Polynomial::selectMinMaxFromCandidates(std::vector<double>& candidates, int derivate,
                                            std::pair<double, double>& minimum,
                                            std::pair<double, double>& maximum){
    if(candidates.empty()){
        std::cout << "Cannot Find Extrema !!" << std::endl;
        return false;
    }

    minimum.first = candidates[0];
    minimum.second = std::numeric_limits<double>::max();
    maximum.first = candidates[0];
    maximum.second = std::numeric_limits<double>::lowest();

    for(std::vector<double>::iterator iter = candidates.begin();
        iter != candidates.end(); iter++){
        
        double actualValue = evaluate(derivate, *iter);
        if(actualValue < minimum.second){
            minimum.first = *iter;
            minimum.second = actualValue;
        }
        
        if(actualValue > maximum.second){
            maximum.first = *iter;
            maximum.second = actualValue;
        }
    }

    return true;
}

// Returning Functions
Eigen::VectorXd Polynomial::getCoefficients(uint derivate){
    Eigen::VectorXd result;
    result.setZero();

    if(derivate > polyOrder || derivate < 0)
        return result;

    if(derivate == 0)
        return polyCoeff;

    result = baseCoeff.row(derivate).cwiseProduct(polyCoeff.transpose());
    return result;
}

double Polynomial::getBaseCoefficient(int derivate, int index){
    if(derivate > polyOrder || derivate < 0){
        std::cout << "Order Not Compliant !!" << std::endl;
        return 0.0;
    }

    return baseCoeff(derivate, index);
}

Eigen::VectorXd Polynomial::getBaseWithTime(uint derivate, double time){
    Eigen::VectorXd result(polyOrder+1);
    result.setZero();

    if(derivate > polyOrder)
        return result;

    double timePow = time;
    result(derivate) = baseCoeff(derivate, derivate);
    for(int i = derivate+1; i < polyOrder+1; i++){
      result(i) = baseCoeff(derivate, i)*timePow;
      timePow = timePow*time;
    }

    return result;
}

double Polynomial::evaluate(uint derivate, double time){
    if(derivate > polyOrder)
        return 0.0;

    return polyCoeff.dot(getBaseWithTime(derivate, time));
}

bool Polynomial::computeMinMax(double startTime, double endTime, int derivate,
                               std::pair<double, double>& minimum,
                               std::pair<double, double>& maximum){
    
    std::vector<double> candidates;
    if(!computeMinMaxCandidates(startTime, endTime, derivate, candidates))
        return false;
  
    // Evaluate Minimum and Maximum.
    return selectMinMaxFromCandidates(candidates, derivate, minimum, maximum);
}

Eigen::VectorXd Polynomial::convolve(uint derivate, uint kernelDerivate){
    Eigen::VectorXd d = getCoefficients(derivate).tail(polyOrder + 1 - derivate);
    Eigen::VectorXd dd = getCoefficients(kernelDerivate).tail(polyOrder + 1 - kernelDerivate);

    int convolutionDimension = dd.size() + d.size() - 1;

    Eigen::VectorXd convolved = Eigen::VectorXd::Zero(convolutionDimension);
    Eigen::VectorXd kernelReverse = dd.reverse();

    for(uint i = 0; i < convolutionDimension; i++){
        int dataIDX = i - dd.size() + 1;

        int lowerBound = std::max(0, -dataIDX);
        int upperBound = std::min(dd.size(), d.size() - dataIDX);

        for (int kernelIDX = lowerBound; kernelIDX < upperBound; ++kernelIDX) {
            convolved[i] += kernelReverse[kernelIDX]*d[dataIDX + kernelIDX];
        }
    }

  return convolved;
}
