// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "QuaternionAverage.hpp"
#include <cmath>
#include "util/Logger.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Meta.h>
#include <Eigen/src/Eigenvalues/EigenSolver.h>

namespace NAV
{
Eigen::Matrix<std::complex<double>, 4, 1> calcQuaternionAverage(std::vector<Eigen::Quaterniond>& quaternions, std::vector<double>& weights, double& maxEigenValue)
{
    auto nQuaternions = quaternions.size();
    Eigen::Matrix4d M = Eigen::Matrix4d::Zero(); // Sum of the weighted dyadic products

    for (size_t i = 0; i < nQuaternions; i++)
    {
        Eigen::Vector4d quat = { quaternions.at(i).coeffs().w(), quaternions.at(i).coeffs().x(), quaternions.at(i).coeffs().y(), quaternions.at(i).coeffs().z() };
        // TODO: Check if w < 1: then there is no rotation and the following calculation could result in a division by 0
        M += weights.at(i) * quat * quat.transpose();
    }
    // TODO: Check whether normalization of M is necessary here

    Eigen::EigenSolver<Eigen::Matrix4d> es(M);
    const auto& eigVals = es.eigenvalues();
    const auto& eigVecs = es.eigenvectors();

    Eigen::Index maxIndex{};
    maxEigenValue = eigVals.real().maxCoeff(&maxIndex);
    auto avgQuat = eigVecs.col(maxIndex);

    if (avgQuat(3).imag() > std::numeric_limits<double>::min()
        || avgQuat(0).imag() > std::numeric_limits<double>::min()
        || avgQuat(1).imag() > std::numeric_limits<double>::min()
        || avgQuat(2).imag() > std::numeric_limits<double>::min())
    {
        LOG_WARN("Imaginary part of at least one coeff of the avgQuat is greater than {} (i.e. 'std::numeric_limits<double>::min()')", std::numeric_limits<double>::min());
    }

    return avgQuat;
}

Eigen::Matrix4d calcQuaternionAverageUncertainty(std::vector<Eigen::Quaterniond>& quaternions, std::vector<double>& weights, std::vector<Eigen::Matrix4d>& sigmaQ, double& maxEigenValue, Eigen::Matrix<std::complex<double>, 4, 1>& eigenVector)
{
    Eigen::Matrix<double, 16, 16> sigmaM = Eigen::Matrix<double, 16, 16>::Zero();

    for (size_t i = 0; i < quaternions.size(); i++)
    {
        Eigen::Matrix<double, 16, 4> jacobian = Eigen::Matrix<double, 16, 4>::Zero();
        jacobian.block<4, 4>(0, 0) = quaternions.at(i).coeffs().w() * Eigen::Matrix4d::Identity();
        jacobian.block<4, 1>(0, 0) += quaternions.at(i).coeffs();
        jacobian.block<4, 4>(4, 0) = quaternions.at(i).coeffs().x() * Eigen::Matrix4d::Identity();
        jacobian.block<4, 1>(4, 1) += quaternions.at(i).coeffs();
        jacobian.block<4, 4>(8, 0) = quaternions.at(i).coeffs().y() * Eigen::Matrix4d::Identity();
        jacobian.block<4, 1>(8, 2) += quaternions.at(i).coeffs();
        jacobian.block<4, 4>(12, 0) = quaternions.at(i).coeffs().z() * Eigen::Matrix4d::Identity();
        jacobian.block<4, 1>(12, 3) += quaternions.at(i).coeffs();

        sigmaM += std::pow(weights.at(i), 2.) * jacobian * sigmaQ.at(i) * jacobian.transpose();
    }

    Eigen::Matrix<double, 4, 16> F_i = Eigen::Matrix<double, 4, 16>::Zero();
    F_i.block<1, 4>(0, 0) = eigenVector.real().transpose();
    F_i.block<1, 4>(1, 4) = eigenVector.real().transpose();
    F_i.block<1, 4>(2, 8) = eigenVector.real().transpose();
    F_i.block<1, 4>(3, 12) = eigenVector.real().transpose();

    return 1 / std::pow(maxEigenValue, 2.) * F_i * sigmaM * F_i.transpose();
}
} // namespace NAV