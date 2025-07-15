// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file AmbiguityResolution.hpp
/// @brief Ambiguity resolution algorithms
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-09-20

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include "util/Eigen.hpp"
#include "util/Logger.hpp"
#include <fmt/format.h>
#include <nlohmann/json.hpp>
using json = nlohmann::json; ///< json namespace

#include "internal/Decorrelation.hpp"
#include "internal/Search.hpp"
#include "internal/Validate.hpp"

#include "Navigation/Math/Math.hpp"

namespace NAV
{

/// Ambiguity resolution strategies
enum class AmbiguityResolutionStrategy : uint8_t
{
    Continuous, ///< Estimate ambiguities every epoch
    FixAndHold, ///< Do not change the ambiguity once it is fixed
    COUNT,      ///< Amount of items in the enum
};

/// @brief Converts the enum to a string
/// @param[in] ambiguityResolutionStrategy Enum value to convert into text
/// @return String representation of the enum
const char* to_string(AmbiguityResolutionStrategy ambiguityResolutionStrategy);

/// @brief Ambiguity resolution algorithms and parameters
struct AmbiguityResolutionParameters
{
    /// @brief Decorrelation algorithms
    enum class DecorrelationAlgorithm : uint8_t
    {
        None,             ///< Do not decorrelate
        Z_Transformation, ///< Z-Transformation
        COUNT,            ///< Amount of items in the enum
    };

    /// @brief Search algorithms
    enum class SearchAlgorithm : uint8_t
    {
        None,                               ///< Disable the search
        IntegerRounding,                    ///< Integer Rounding (IR)
        IntegerBootstrapping,               ///< Integer Bootstrapping (IB)
        IntegerLeastSquaresSearch,          ///< Integer least-squares (ILS) Search (LAMBDA)
        IntegerLeastSquaresSearchAndShrink, ///< Integer least-squares (ILS) Search-and-Shrink (MLAMBDA)
        COUNT,                              ///< Amount of items in the enum
    };

    /// @brief Validation algorithms
    ///
    /// Define the best fitting integer solution \f$ \mathbf{\check{a}} \f$ and second best integer solution as \f$ \mathbf{\check{a}}' \f$ (\cite SpringerHandbookGNSS2017 Springer Handbook GNSS, ch. 23.6.4, eq. 23.79)
    /// \anchor eq-ambRes-val \f{equation}{ \label{eq:eq-ambRes-val}
    /// \begin{aligned}
    ///   \mathbf{\check{a}}  &= \text{arg} \min_{z \in \mathbb{Z}^n} ||\mathbf{\hat{a}} - \mathbf{z}||^2_{\mathbf{Q_{\mathbf{\hat{a}}\mathbf{\hat{a}}}}} \\
    ///   \mathbf{\check{a}}' &= \text{arg} \min_{z \in \mathbb{Z}^n, z \neq \mathbf{\check{a}}} ||\mathbf{\hat{a}} - \mathbf{z}||^2_{\mathbf{Q_{\mathbf{\hat{a}}\mathbf{\hat{a}}}}}
    /// \end{aligned}
    /// \f}
    enum class ValidationAlgorithm : uint8_t
    {
        /// Do not validate the solution (always accept the integer solution, if one is found)
        None,
        /// Accept if (see \cite Verhagen2006 Verhagen 2006, eq. 31, see NAV::Ambiguity::differenceTest)
        /// \anchor eq-ambRes-diff \f{equation}{ \label{eq:eq-ambRes-diff}
        ///   ||\mathbf{\hat{a}} - \mathbf{\check{a}}'||^2_{\mathbf{Q_{\mathbf{\hat{a}}\mathbf{\hat{a}}}}}
        /// - ||\mathbf{\hat{a}} - \mathbf{\check{a}} ||^2_{\mathbf{Q_{\mathbf{\hat{a}}\mathbf{\hat{a}}}}} \ge c
        /// \f}
        DifferenceTest,
        /// Accept if (see \cite SpringerHandbookGNSS2017 Springer Handbook GNSS, ch. 23.6.4, eq. 23.78 or \cite Verhagen2006 Verhagen 2006, eq. 28, 29, see NAV::Ambiguity::ratioTest)
        /// \anchor eq-ambRes-ratio \f{equation}{ \label{eq:eq-ambRes-ratio}
        /// \frac{||\mathbf{\hat{a}} - \mathbf{\check{a}} ||^2_{\mathbf{Q_{\mathbf{\hat{a}}\mathbf{\hat{a}}}}}}
        ///      {||\mathbf{\hat{a}} - \mathbf{\check{a}}'||^2_{\mathbf{Q_{\mathbf{\hat{a}}\mathbf{\hat{a}}}}}} \le \mu
        /// ,\quad 0 < \mu \le 1, \text{given by the user}
        /// \f}
        RatioTestCriticalValue,
        /// Accept if \eqref{eq-ambRes-ratio}, but with \f$ \mu \f$ calculated from given failure rate \f$ P_F \f$ (see \cite Verhagen2013 Verhagen 2013, see NAV::Ambiguity::fixedFailureRateRatioTest)
        RatioTestFailureRate,
        /// Accept if (see \cite Verhagen2006 Verhagen 2006, eq. 35, see NAV::Ambiguity::projectorTest)
        /// \anchor eq-ambRes-proj \f{equation}{ \label{eq:eq-ambRes-proj}
        /// \left| \dfrac{(\mathbf{\check{a}}' - \mathbf{\check{a}})^T \mathbf{Q}_{\mathbf{\hat{a}}}^{-1} (\mathbf{\hat{a}} - \mathbf{\check{a}}) }
        ///              {|| \mathbf{\check{a}}' - \mathbf{\check{a}} ||_{\mathbf{Q}_{\mathbf{\hat{a}}}}} \right| \le \mu
        /// \f}
        /// It projects \f$ \mathbf{\hat{a}} - \mathbf{\check{a}} \f$ orthogonally on the direction of \f$ \mathbf{\check{a}}' - \mathbf{\check{a}} \f$, in the metric of \f$ \mathbf{Q}_{\mathbf{\hat{a}}} \f$
        ProjectorTest,
        /// Amount of items in the enum
        COUNT,
    };

    /// Decorrelation algorithm
    DecorrelationAlgorithm decorrelationAlgorithm = DecorrelationAlgorithm::Z_Transformation;
    /// Search algorithm
    SearchAlgorithm searchAlgorithm = SearchAlgorithm::IntegerLeastSquaresSearchAndShrink;
    /// Validation with Bootstrapped success rate (Bootstrapped failure rate is an upper bound for the ILS failure rate)
    bool validationBootstrappedSuccessRate = true;
    /// Validation algorithm
    ValidationAlgorithm validationAlgorithm = ValidationAlgorithm::RatioTestCriticalValue;

    /// @brief Critical value c for the the difference test
    double validationTestCriticalValueC = 10.0;
    /// @brief Critical value µ for the the ratio and projector test (0, 1]
    double validationTestCriticalValueMu = 1.0 / 3.0;
    /// @brief Failure rate for the ratio test (used to calculate µ)
    double validationRatioTestFailureRate = 0.001;
    /// @brief Attempt partial fixing of ambiguities
    bool partialFixing = false;

    /// Possible failure rates for the look-up tables
    static constexpr std::array<double, 2> allowedFailureRateValues = { { 0.001, 0.01 } };
};

/// @brief Converts the enum to a string
/// @param[in] decorrelationAlgorithm Enum value to convert into text
/// @return String representation of the enum
const char* to_string(AmbiguityResolutionParameters::DecorrelationAlgorithm decorrelationAlgorithm);

/// @brief Converts the enum to a string
/// @param[in] searchAlgorithm Enum value to convert into text
/// @return String representation of the enum
const char* to_string(AmbiguityResolutionParameters::SearchAlgorithm searchAlgorithm);

/// @brief Converts the enum to a string
/// @param[in] searchAlgorithm Enum value to convert into text
/// @return String representation of the enum
const char* to_string_short(AmbiguityResolutionParameters::SearchAlgorithm searchAlgorithm);

/// @brief Converts the enum to a string
/// @param[in] validationAlgorithm Enum value to convert into text
/// @return String representation of the enum
const char* to_string(AmbiguityResolutionParameters::ValidationAlgorithm validationAlgorithm);

/// @brief Shows a ComboBox to select the ambiguity resolution algorithms
/// @param[in] id Unique id for ImGui.
/// @param[in, out] params Reference to the ambiguity resolution parameter struct
/// @param[in] width GUI item width
bool GuiAmbiguityResolution(const char* id, AmbiguityResolutionParameters& params, float width = 310.0F);

/// @brief Possible failures
enum class AmbiguityResolutionFailure : uint8_t
{
    None,              ///< No failure
    NoSearchAlgorithm, ///< No Search algorithm selected
    Decorrelation,     ///< Decorrelation failed
    NoCandidatesFound, ///< No candidates were found with the search
    ValidationFailed,  ///< Validation rejected the result
};

/// @brief Ambiguity resolution result
template<typename Scalar, int nAmb, int nReal>
struct AmbiguityResolutionResult
{
    /// @brief Fixed ambiguity and their squared norm
    struct FixedAmbiguity
    {
        /// @brief Constructor
        /// @param sqnorm Squared norm
        /// @param a Fixed ambiguity vector [cycles]
        template<typename Derived>
        FixedAmbiguity(double sqnorm, const Eigen::MatrixBase<Derived>& a)
            : sqnorm(sqnorm), a(a)
        {}

        double sqnorm;                 ///< Squared norm
        Eigen::Vector<Scalar, nAmb> a; ///< Fixed ambiguity vector [cycles]
    };

    AmbiguityResolutionFailure failure = AmbiguityResolutionFailure::None; ///< Failure mode
    double ambiguityCriticalValueRatio{};                                  ///< Ambiguity Critical Value µ ∈ (0, 1] (R1/R2 ≤ µ)

    size_t nFixed = 0;                      ///< Number of fixed ambiguities (differs from vector size in case of partial fixing)
    std::vector<FixedAmbiguity> fixedAmb;   ///< Sorted vector of fixed ambiguities and their norms
    Eigen::Vector<Scalar, nReal> b;         ///< Fixed non-integer float states (e.g. Pos, Vel, ...)
    Eigen::Matrix<Scalar, nReal, nReal> Qb; ///< Fixed variance/covariance matrix of the non-integer float states
};

/// @brief Tries resolving the ambiguities
/// @param a Float ambiguity vector [cycles]
/// @param Qa Variance/covariance matrix of the ambiguities
/// @param b Non-integer float states (e.g. Pos, Vel, ...)
/// @param Qb Variance/covariance matrix of the non-integer float states
/// @param Qab Upper right part of the variance/covariance matrix (correlation between ambiguities and other states)
/// @param Qba Lower left part of the variance/covariance matrix (correlation between ambiguities and other states)
/// @param params Ambiguity resolution algorithm and parameters
/// @param nameId NameId for debugging
/// @return The result struct if the ambiguities could be fixed and validated
template<typename DerivedA, typename DerivedQa, typename DerivedB, typename DerivedQb, typename DerivedQab, typename DerivedQba>
AmbiguityResolutionResult<typename DerivedA::Scalar, DerivedA::RowsAtCompileTime, DerivedB::RowsAtCompileTime>
    ResolveAmbiguities(const Eigen::MatrixBase<DerivedA>& a, const Eigen::MatrixBase<DerivedQa>& Qa,
                       const Eigen::MatrixBase<DerivedB>& b, const Eigen::MatrixBase<DerivedQb>& Qb,
                       const Eigen::MatrixBase<DerivedQab>& Qab, const Eigen::MatrixBase<DerivedQba>& Qba,
                       const AmbiguityResolutionParameters& params,
                       [[maybe_unused]] const std::string& nameId)
{
    using DecorrelationAlgorithm = AmbiguityResolutionParameters::DecorrelationAlgorithm;
    using SearchAlgorithm = AmbiguityResolutionParameters::SearchAlgorithm;
    using ValidationAlgorithm = AmbiguityResolutionParameters::ValidationAlgorithm;

    using Eigen::seq, Eigen::last;

    AmbiguityResolutionResult<typename DerivedA::Scalar, DerivedA::RowsAtCompileTime, DerivedB::RowsAtCompileTime> result;

    if (params.searchAlgorithm == SearchAlgorithm::None) // If we do not search, do not do any work like decorrelation or validation
    {
        result.failure = AmbiguityResolutionFailure::NoSearchAlgorithm;
        return result;
    }

    LOG_DATA("{}: Qa = \n{}", nameId, Eigen::MatrixXd(Qa));
    LOG_DATA("{}: a = {}", nameId, a.transpose());
    LOG_DATA("{}: Qb = \n{}", nameId, Eigen::MatrixXd(Qb));
    LOG_DATA("{}: b = {}", nameId, b.transpose());
    LOG_DATA("{}: Qab = \n{}", nameId, Eigen::MatrixXd(Qab));
    LOG_DATA("{}: Qba = \n{}", nameId, Eigen::MatrixXd(Qba));

    // Avoid integer overflows by reducing the ambiguities between -1.0 and +1.0
    Eigen::VectorXd ambIntPart = a.template cast<int>().template cast<double>();
    Eigen::VectorXd ambFracPart = a - ambIntPart;
    LOG_DATA("{}: ambFracPart = {}", nameId, ambFracPart.transpose());

    typename DerivedQa::PlainObject Qz; // Decorrelated ambiguity covariance matrix
    typename DerivedQa::PlainObject Z;  // Decorrelation transformation matrix
    typename DerivedQa::PlainObject L;  // Lower-triangular matrix from the L^T * D * L decomposition of Q
    Eigen::VectorXd D;                  // Diagonal entries from the L^T * D * L decomposition of Q
    Eigen::VectorXd z;                  // Decorrelated float ambiguity vector [cycles]

    switch (params.decorrelationAlgorithm)
    {
    case DecorrelationAlgorithm::Z_Transformation:
        if (auto decorrelated_ztrafo = Ambiguity::decorrelate_ztrafo(ambFracPart, Qa))
        {
            std::tie(Qz, Z, L, D, z) = *decorrelated_ztrafo;
        }
        else
        {
            LOG_DEBUG("{}: Decorrelation failed", nameId);
            result.failure = AmbiguityResolutionFailure::Decorrelation;
            return result;
        }
        break;
    case DecorrelationAlgorithm::None:
    case DecorrelationAlgorithm::COUNT:
        Qz = Qa;
        z = a;
        if (auto ltdl_decomp = math::LtDLdecomp_choleskyFact(Qa))
        {
            std::tie(L, D) = *ltdl_decomp;
        }
        else
        {
            LOG_DEBUG("{}: Decorrelation failed", nameId);
        }
        break;
    }
    LOG_DATA("{}: z = {}", nameId, z.transpose());
    LOG_DATA("{}: Qz = \n{}", nameId, Eigen::MatrixXd(Qz));
    LOG_DATA("{}: Z = \n{}", nameId, Z);
    LOG_DATA("{}: L = \n{}", nameId, L);
    LOG_DATA("{}: D = {}", nameId, D.transpose());

    auto n = z.rows();
    int k = 0;
    // if (params.partialFixing) // Partial fixing is done by giving a subset to this function. So not relevant here
    // {
    //     k = -1;
    //     double P0 = 0.995;
    //     double Ps = 0.0;
    //     do // Decorrelated ambiguities are sorted by standard deviation. Largest entry in D is first entry. So remove from top
    //     {
    //         k++;
    //         Ps = Ambiguity::successRateBootstrapping(D(seq(k, last)));
    //         LOG_DATA("{}: Ps(k = {}) = {}", nameId, k, Ps);
    //     } while (Ps < P0 && k < n - 1);
    //     if (Ps < P0) { return {}; }
    // }
    if (k != 0)
    {
        LOG_TRACE("{}: Doing partial ambiguity fixing for only {} of {} ambiguities", nameId, n - k, n);
        LOG_DATA("{}: z = {}", nameId, z(seq(k, last)).transpose());
        LOG_DATA("{}: Qz = \n{}", nameId, Eigen::MatrixXd(Qz(seq(k, last), seq(k, last))));
        LOG_DATA("{}: L = \n{}", nameId, L(seq(k, last), seq(k, last)));
        LOG_DATA("{}: D = {}", nameId, D(seq(k, last)).transpose());
    }

    Eigen::MatrixXd cands;
    Eigen::VectorXd sqnorm;
    int numCandidates = 2;

    switch (params.searchAlgorithm)
    {
    case SearchAlgorithm::IntegerRounding:
        cands = Ambiguity::integerSearchRounding(z(seq(k, last)));
        sqnorm = Eigen::VectorXd::Constant(1, 1.0);
        break;
    case SearchAlgorithm::IntegerBootstrapping:
        cands = Ambiguity::integerSearchBootstrapping(z(seq(k, last)), Qz(seq(k, last), seq(k, last)));
        sqnorm = Eigen::VectorXd::Constant(1, 1.0);
        break;
    case SearchAlgorithm::IntegerLeastSquaresSearch:
        std::tie(cands, sqnorm) = Ambiguity::integerLeastSquaresSearch(z(seq(k, last)), Qz(seq(k, last), seq(k, last)),
                                                                       L(seq(k, last), seq(k, last)), D(seq(k, last)), numCandidates);
        break;
    case SearchAlgorithm::IntegerLeastSquaresSearchAndShrink:
        std::tie(cands, sqnorm) = Ambiguity::integerLeastSquaresSearchAndShrink(z(seq(k, last)), L(seq(k, last), seq(k, last)), D(seq(k, last)), numCandidates);
        break;
    case SearchAlgorithm::None:
    case SearchAlgorithm::COUNT:
        break;
    }

#if LOG_LEVEL <= LOG_LEVEL_DATA
    if (k != 0)
    {
        Eigen::MatrixXd print(cands.cols(), cands.rows() + 1);
        for (Eigen::Index i = 0; i < cands.cols(); i++)
        {
            print(i, 0) = sqnorm(i);
            print(i, Eigen::seq(1, Eigen::last)) = cands.col(i).transpose();
        }
        LOG_DATA("{}: sqnorm, cand (1 candidate each row)\n{}", nameId, print);
    }
#endif

    if (cands.cols() == 0)
    {
        LOG_DATA("{}: No candidates found ", nameId);
        result.failure = AmbiguityResolutionFailure::NoCandidatesFound;
        return result;
    }

    // Partial fixing is done by giving a subset to this function. So not relevant here
    //     if (params.partialFixing) // Adjust first k-1 ambiguities based on correlation with the fixed ambiguities
    //     {
    //         LOG_DATA("{}: Adjusting first k-1 ambiguities based on correlation with the fixed ambiguities", nameId);
    //         Eigen::MatrixXd zfixed = Eigen::MatrixXd::Zero(n, numCandidates);
    //         Eigen::MatrixXd QP = Qz(seq(0, k - 1), seq(k, last)) * Qz(seq(k, last), seq(k, last)).inverse();
    //         for (int i = 0; i < numCandidates; i++)
    //         {
    //             zfixed(seq(0, k - 1), i) = z(seq(0, k - 1)) - QP * (z(seq(k, last)) - cands(Eigen::all, i));
    //         }
    //         zfixed(seq(k, last), Eigen::all) = cands;
    //         cands = zfixed;

    // #if LOG_LEVEL <= LOG_LEVEL_DATA
    //         {
    //             Eigen::MatrixXd print(cands.cols(), cands.rows() + 1);
    //             for (Eigen::Index i = 0; i < cands.cols(); i++)
    //             {
    //                 print(i, 0) = sqnorm(i);
    //                 print(i, Eigen::seq(1, Eigen::last)) = cands.col(i).transpose();
    //             }
    //             LOG_DATA("{}: sqnorm, cand (1 candidate each row)\n{}", nameId, print);
    //         }
    // #endif
    //     }

    for (Eigen::Index i = 0; i < cands.cols(); i++)
    {
        if (params.decorrelationAlgorithm != DecorrelationAlgorithm::None)
        {
            // Back transformation
            cands.col(i) = Z.transpose().inverse() * cands.col(i);

            // Z is an integer preserving transformation
            // If cands is only partially fixed, then the output wont have integers at all
        }
        // Reapply the integer part from before
        cands.col(i) += ambIntPart;
    }

#if LOG_LEVEL <= LOG_LEVEL_DATA
    {
        Eigen::MatrixXd print(cands.cols(), cands.rows() + 1);
        for (Eigen::Index i = 0; i < cands.cols(); i++)
        {
            print(i, 0) = sqnorm(i);
            print(i, Eigen::seq(1, Eigen::last)) = cands.col(i).transpose();
        }
        LOG_DATA("{}: Back transformed results - sqnorm, cand (1 candidate each row){}\n{}", nameId,
                 k != 0 ? " (not integer, because only partial fix)" : "", print);
    }
#endif

    if (cands.cols() > 1) // If we found only one candidate, the second one was very unlikely. Therefore we can accept this one without testing
    {
        result.ambiguityCriticalValueRatio = sqnorm(0) / sqnorm(1);
        switch (params.validationAlgorithm)
        {
        case ValidationAlgorithm::DifferenceTest:
            if (!Ambiguity::differenceTest(sqnorm(0), sqnorm(1), params.validationTestCriticalValueC))
            {
                result.failure = AmbiguityResolutionFailure::ValidationFailed;
                return result;
            }
            break;
        case ValidationAlgorithm::RatioTestCriticalValue:
            if (!Ambiguity::ratioTest(sqnorm(0), sqnorm(1), params.validationTestCriticalValueMu))
            {
                result.failure = AmbiguityResolutionFailure::ValidationFailed;
                return result;
            }
            break;
        case ValidationAlgorithm::RatioTestFailureRate:
            if (!Ambiguity::fixedFailureRateRatioTest(params.validationRatioTestFailureRate, sqnorm(0), sqnorm(1),
                                                      static_cast<size_t>(n - k), D, params.validationBootstrappedSuccessRate))
            {
                result.failure = AmbiguityResolutionFailure::ValidationFailed;
                return result;
            }
            break;
        case ValidationAlgorithm::ProjectorTest:
            if (!Ambiguity::projectorTest(cands.col(0), cands.col(1), a(seq(k, last)), Qa(seq(k, last), seq(k, last)),
                                          params.validationTestCriticalValueMu))
            {
                result.failure = AmbiguityResolutionFailure::ValidationFailed;
                return result;
            }
            break;
        case ValidationAlgorithm::None:
        case ValidationAlgorithm::COUNT:
            break;
        }
    }

    result.b = b - Qba * Qa.inverse() * (a - cands.col(0));
    result.Qb = Qb - Qba * Qa.inverse() * Qab;

    for (Eigen::Index i = 0; i < cands.cols(); i++)
    {
        result.fixedAmb.emplace_back(sqnorm(i), cands.col(i));
    }
    result.nFixed = static_cast<size_t>(n - k);

    return result;
}

/// @brief Converts the provided object into json
/// @param[out] j Json object which gets filled with the info
/// @param[in] obj Object to convert into json
void to_json(json& j, const AmbiguityResolutionParameters& obj);
/// @brief Converts the provided json object into a node object
/// @param[in] j Json object with the needed values
/// @param[out] obj Object to fill from the json
void from_json(const json& j, AmbiguityResolutionParameters& obj);

} // namespace NAV
