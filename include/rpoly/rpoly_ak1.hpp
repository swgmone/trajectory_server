/* Polynomial Handler                          */
/* Author: Lorenzo Gentilini                   */
/* E-Mail: lorenzo.gentilini6@unibo.it         */
/* Date: August 2020                           */
/* File: rpoly_ak1.hpp                         */

#ifndef _RPOLY_H
#define _RPOLY_H

#include <Eigen/Eigen>
#include <iostream>
#include <fstream>
#include <cctype>
#include <cmath>
#include <cfloat>

int findLastNonZeroCoeff(Eigen::VectorXd coefficients);

bool findRootsJenkinsTraub(Eigen::VectorXd coefficients_increasing,
                           Eigen::VectorXcd& roots);

#endif