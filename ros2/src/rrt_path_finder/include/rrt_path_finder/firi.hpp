/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

/* This is an old version of FIRI for temporary usage here. */

#ifndef FIRI_HPP
#define FIRI_HPP

#include "lbfgs.hpp"
#include "sdlp.hpp"

#include <Eigen/Eigen>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cfloat>
#include <cmath>
#include <vector>

namespace firi
{

    inline void chol3d(const Eigen::Matrix3d &A,
                       Eigen::Matrix3d &L)
    {
        L(0, 0) = sqrt(A(0, 0));
        L(0, 1) = 0.0;
        L(0, 2) = 0.0;
        L(1, 0) = 0.5 * (A(0, 1) + A(1, 0)) / L(0, 0);
        L(1, 1) = sqrt(A(1, 1) - L(1, 0) * L(1, 0));
        L(1, 2) = 0.0;
        L(2, 0) = 0.5 * (A(0, 2) + A(2, 0)) / L(0, 0);
        L(2, 1) = (0.5 * (A(1, 2) + A(2, 1)) - L(2, 0) * L(1, 0)) / L(1, 1);
        L(2, 2) = sqrt(A(2, 2) - L(2, 0) * L(2, 0) - L(2, 1) * L(2, 1));
        return;
    }

    inline bool smoothedL1(const double &mu,
                           const double &x,
                           double &f,
                           double &df)
    {
        if (x < 0.0)
        {
            return false;
        }
        else if (x > mu)
        {
            f = x - 0.5 * mu;
            df = 1.0;
            return true;
        }
        else
        {
            const double xdmu = x / mu;
            const double sqrxdmu = xdmu * xdmu;
            const double mumxd2 = mu - 0.5 * x;
            f = mumxd2 * sqrxdmu * xdmu;
            df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
            return true;
        }
    }

    inline double costMVIE(void *data,
                           const Eigen::VectorXd &x,
                           Eigen::VectorXd &grad)
    {
        const int64_t *pM = (int64_t *)data;
        const double *pSmoothEps = (double *)(pM + 1);
        const double *pPenaltyWt = pSmoothEps + 1;
        const double *pA = pPenaltyWt + 1;

        const int M = *pM;
        const double smoothEps = *pSmoothEps;
        const double penaltyWt = *pPenaltyWt;
        Eigen::Map<const Eigen::MatrixX3d> A(pA, M, 3);
        Eigen::Map<const Eigen::Vector3d> p(x.data());
        Eigen::Map<const Eigen::Vector3d> rtd(x.data() + 3);
        Eigen::Map<const Eigen::Vector3d> cde(x.data() + 6);
        Eigen::Map<Eigen::Vector3d> gdp(grad.data());
        Eigen::Map<Eigen::Vector3d> gdrtd(grad.data() + 3);
        Eigen::Map<Eigen::Vector3d> gdcde(grad.data() + 6);

        double cost = 0;
        gdp.setZero();
        gdrtd.setZero();
        gdcde.setZero();

        Eigen::Matrix3d L;
        L(0, 0) = rtd(0) * rtd(0) + DBL_EPSILON;
        L(0, 1) = 0.0;
        L(0, 2) = 0.0;
        L(1, 0) = cde(0);
        L(1, 1) = rtd(1) * rtd(1) + DBL_EPSILON;
        L(1, 2) = 0.0;
        L(2, 0) = cde(2);
        L(2, 1) = cde(1);
        L(2, 2) = rtd(2) * rtd(2) + DBL_EPSILON;

        const Eigen::MatrixX3d AL = A * L;
        const Eigen::VectorXd normAL = AL.rowwise().norm();
        const Eigen::Matrix3Xd adjNormAL = (AL.array().colwise() / normAL.array()).transpose();
        const Eigen::VectorXd consViola = (normAL + A * p).array() - 1.0;

        double c, dc;
        Eigen::Vector3d vec;
        for (int i = 0; i < M; ++i)
        {
            if (smoothedL1(smoothEps, consViola(i), c, dc))
            {
                cost += c;
                vec = dc * A.row(i).transpose();
                gdp += vec;
                gdrtd += adjNormAL.col(i).cwiseProduct(vec);
                gdcde(0) += adjNormAL(0, i) * vec(1);
                gdcde(1) += adjNormAL(1, i) * vec(2);
                gdcde(2) += adjNormAL(0, i) * vec(2);
            }
        }
        cost *= penaltyWt;
        gdp *= penaltyWt;
        gdrtd *= penaltyWt;
        gdcde *= penaltyWt;

        cost -= log(L(0, 0)) + log(L(1, 1)) + log(L(2, 2));
        gdrtd(0) -= 1.0 / L(0, 0);
        gdrtd(1) -= 1.0 / L(1, 1);
        gdrtd(2) -= 1.0 / L(2, 2);

        gdrtd(0) *= 2.0 * rtd(0);
        gdrtd(1) *= 2.0 * rtd(1);
        gdrtd(2) *= 2.0 * rtd(2);

        return cost;
    }

    // Each row of hPoly is defined by h0, h1, h2, h3 as
    // h0*x + h1*y + h2*z + h3 <= 0
    // R, p, r are ALWAYS taken as the initial guess
    // R is also assumed to be a rotation matrix
    inline bool maxVolInsEllipsoid(const Eigen::MatrixX4d &hPoly,
                                   Eigen::Matrix3d &R,
                                   Eigen::Vector3d &p,
                                   Eigen::Vector3d &r)
    {
        // Find the deepest interior point
        const int M = hPoly.rows();
        Eigen::MatrixX4d Alp(M, 4);
        Eigen::VectorXd blp(M);
        Eigen::Vector4d clp, xlp;
        const Eigen::ArrayXd hNorm = hPoly.leftCols<3>().rowwise().norm();
        Alp.leftCols<3>() = hPoly.leftCols<3>().array().colwise() / hNorm;
        Alp.rightCols<1>().setConstant(1.0);
        blp = -hPoly.rightCols<1>().array() / hNorm;
        clp.setZero();
        clp(3) = -1.0;
        const double maxdepth = -sdlp::linprog<4>(clp, Alp, blp, xlp);
        if (!(maxdepth > 0.0) || std::isinf(maxdepth))
        {
            return false;
        }
        const Eigen::Vector3d interior = xlp.head<3>();

        // Prepare the data for MVIE optimization
        uint8_t *optData = new uint8_t[sizeof(int64_t) + (2 + 3 * M) * sizeof(double)];
        int64_t *pM = (int64_t *)optData;
        double *pSmoothEps = (double *)(pM + 1);
        double *pPenaltyWt = pSmoothEps + 1;
        double *pA = pPenaltyWt + 1;

        *pM = M;
        Eigen::Map<Eigen::MatrixX3d> A(pA, M, 3);
        A = Alp.leftCols<3>().array().colwise() /
            (blp - Alp.leftCols<3>() * interior).array();

        Eigen::VectorXd x(9);
        const Eigen::Matrix3d Q = R * (r.cwiseProduct(r)).asDiagonal() * R.transpose();
        Eigen::Matrix3d L;
        chol3d(Q, L);

        x.head<3>() = p - interior;
        x(3) = sqrt(L(0, 0));
        x(4) = sqrt(L(1, 1));
        x(5) = sqrt(L(2, 2));
        x(6) = L(1, 0);
        x(7) = L(2, 1);
        x(8) = L(2, 0);

        double minCost;
        lbfgs::lbfgs_parameter_t paramsMVIE;
        paramsMVIE.mem_size = 18;
        paramsMVIE.g_epsilon = 0.0;
        paramsMVIE.min_step = 1.0e-32;
        paramsMVIE.past = 3;
        paramsMVIE.delta = 1.0e-7;
        *pSmoothEps = 1.0e-2;
        *pPenaltyWt = 1.0e+3;

        int ret = lbfgs::lbfgs_optimize(x,
                                        minCost,
                                        &costMVIE,
                                        nullptr,
                                        nullptr,
                                        optData,
                                        paramsMVIE);

        if (ret < 0)
        {
            printf("FIRI WARNING: %s\n", lbfgs::lbfgs_strerror(ret));
        }

        p = x.head<3>() + interior;
        L(0, 0) = x(3) * x(3);
        L(0, 1) = 0.0;
        L(0, 2) = 0.0;
        L(1, 0) = x(6);
        L(1, 1) = x(4) * x(4);
        L(1, 2) = 0.0;
        L(2, 0) = x(8);
        L(2, 1) = x(7);
        L(2, 2) = x(5) * x(5);
        Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::FullPivHouseholderQRPreconditioner> svd(L, Eigen::ComputeFullU);
        const Eigen::Matrix3d U = svd.matrixU();
        const Eigen::Vector3d S = svd.singularValues();
        if (U.determinant() < 0.0)
        {
            R.col(0) = U.col(1);
            R.col(1) = U.col(0);
            R.col(2) = U.col(2);
            r(0) = S(1);
            r(1) = S(0);
            r(2) = S(2);
        }
        else
        {
            R = U;
            r = S;
        }

        delete[] optData;

        return ret >= 0;
    }

    inline bool firi(const Eigen::MatrixX4d &bd,
                     const Eigen::Matrix3Xd &pc,
                     const Eigen::Vector3d &a,
                     const Eigen::Vector3d &b,
                     Eigen::MatrixX4d &hPoly,
                     const int iterations = 4,
                     const double epsilon = 1.0e-6)
    {
        const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
        const Eigen::Vector4d bh(b(0), b(1), b(2), 1.0);

        if ((bd * ah).maxCoeff() > 0.0 ||
            (bd * bh).maxCoeff() > 0.0)
        {
            return false;
        }

        const int M = bd.rows();
        const int N = pc.cols();

        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        Eigen::Vector3d p = 0.5 * (a + b);
        Eigen::Vector3d r = Eigen::Vector3d::Ones();
        Eigen::MatrixX4d forwardH(M + N, 4);
        int nH = 0;

        for (int loop = 0; loop < iterations; ++loop)
        {
            const Eigen::Matrix3d forward = r.cwiseInverse().asDiagonal() * R.transpose();
            const Eigen::Matrix3d backward = R * r.asDiagonal();
            const Eigen::MatrixX3d forwardB = bd.leftCols<3>() * backward;
            const Eigen::VectorXd forwardD = bd.rightCols<1>() + bd.leftCols<3>() * p;
            const Eigen::Matrix3Xd forwardPC = forward * (pc.colwise() - p);
            const Eigen::Vector3d fwd_a = forward * (a - p);
            const Eigen::Vector3d fwd_b = forward * (b - p);

            const Eigen::VectorXd distDs = forwardD.cwiseAbs().cwiseQuotient(forwardB.rowwise().norm());
            Eigen::MatrixX4d tangents(N, 4);
            Eigen::VectorXd distRs(N);

            for (int i = 0; i < N; i++)
            {
                distRs(i) = forwardPC.col(i).norm();
                tangents(i, 3) = -distRs(i);
                tangents.block<1, 3>(i, 0) = forwardPC.col(i).transpose() / distRs(i);
                if (tangents.block<1, 3>(i, 0).dot(fwd_a) + tangents(i, 3) > epsilon)
                {
                    const Eigen::Vector3d delta = forwardPC.col(i) - fwd_a;
                    tangents.block<1, 3>(i, 0) = fwd_a - (delta.dot(fwd_a) / delta.squaredNorm()) * delta;
                    distRs(i) = tangents.block<1, 3>(i, 0).norm();
                    tangents(i, 3) = -distRs(i);
                    tangents.block<1, 3>(i, 0) /= distRs(i);
                }
                if (tangents.block<1, 3>(i, 0).dot(fwd_b) + tangents(i, 3) > epsilon)
                {
                    const Eigen::Vector3d delta = forwardPC.col(i) - fwd_b;
                    tangents.block<1, 3>(i, 0) = fwd_b - (delta.dot(fwd_b) / delta.squaredNorm()) * delta;
                    distRs(i) = tangents.block<1, 3>(i, 0).norm();
                    tangents(i, 3) = -distRs(i);
                    tangents.block<1, 3>(i, 0) /= distRs(i);
                }
                if (tangents.block<1, 3>(i, 0).dot(fwd_a) + tangents(i, 3) > epsilon)
                {
                    tangents.block<1, 3>(i, 0) = (fwd_a - forwardPC.col(i)).cross(fwd_b - forwardPC.col(i)).normalized();
                    tangents(i, 3) = -tangents.block<1, 3>(i, 0).dot(fwd_a);
                    tangents.row(i) *= tangents(i, 3) > 0.0 ? -1.0 : 1.0;
                }
            }

            Eigen::Matrix<uint8_t, -1, 1> bdFlags = Eigen::Matrix<uint8_t, -1, 1>::Constant(M, 1);
            Eigen::Matrix<uint8_t, -1, 1> pcFlags = Eigen::Matrix<uint8_t, -1, 1>::Constant(N, 1);

            nH = 0;

            bool completed = false;
            int bdMinId = 0, pcMinId = 0;
            double minSqrD = distDs.minCoeff(&bdMinId);
            double minSqrR = INFINITY;
            if (distRs.size() != 0)
            {
                minSqrR = distRs.minCoeff(&pcMinId);
            }
            for (int i = 0; !completed && i < (M + N); ++i)
            {
                if (minSqrD < minSqrR)
                {
                    forwardH.block<1, 3>(nH, 0) = forwardB.row(bdMinId);
                    forwardH(nH, 3) = forwardD(bdMinId);
                    bdFlags(bdMinId) = 0;
                }
                else
                {
                    forwardH.row(nH) = tangents.row(pcMinId);
                    pcFlags(pcMinId) = 0;
                }

                completed = true;
                minSqrD = INFINITY;
                for (int j = 0; j < M; ++j)
                {
                    if (bdFlags(j))
                    {
                        completed = false;
                        if (minSqrD > distDs(j))
                        {
                            bdMinId = j;
                            minSqrD = distDs(j);
                        }
                    }
                }
                minSqrR = INFINITY;
                for (int j = 0; j < N; ++j)
                {
                    if (pcFlags(j))
                    {
                        if (forwardH.block<1, 3>(nH, 0).dot(forwardPC.col(j)) + forwardH(nH, 3) > -epsilon)
                        {
                            pcFlags(j) = 0;
                        }
                        else
                        {
                            completed = false;
                            if (minSqrR > distRs(j))
                            {
                                pcMinId = j;
                                minSqrR = distRs(j);
                            }
                        }
                    }
                }
                ++nH;
            }

            hPoly.resize(nH, 4);
            for (int i = 0; i < nH; ++i)
            {
                hPoly.block<1, 3>(i, 0) = forwardH.block<1, 3>(i, 0) * forward;
                hPoly(i, 3) = forwardH(i, 3) - hPoly.block<1, 3>(i, 0).dot(p);
            }

            if (loop == iterations - 1)
            {
                break;
            }

            maxVolInsEllipsoid(hPoly, R, p, r);
        }

        return true;
    }

}

#endif
