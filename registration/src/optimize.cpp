#include "optimize.h"
#include <Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/unsupported/Eigen/src/NonLinearOptimization/LevenbergMarquardt.h>
#include <eigen3/unsupported/Eigen/src/NumericalDiff/NumericalDiff.h>
using namespace std;
using namespace Eigen;

/**
 * @brief calculate loss(tgt pointcloud and transformed point cloud)
 *
 * @param trans trans[0],trans[1],trans[2] ：the angel rotaiton by x,y,z
 * t=[trans[3],trans[4],trans[5]
 * @param loss
 * @return int return -1  while get exception else get 0
 */

int optimize::operator()(const Eigen::VectorXd &trans,
                         Eigen::VectorXd &loss) const
{

    try
    {
        Eigen::Matrix3d rotationMatrix;
        Eigen::Quaterniond res =
            Eigen::AngleAxisd(trans[0], Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(trans[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(trans[2], Eigen::Vector3d::UnitZ());
        rotationMatrix = res.toRotationMatrix();
        Eigen::Vector3d transMatrix =
            Eigen::Vector3d(trans[3], trans[4], trans[5]);

        loss.resize(this->srcPoint.size());

        auto getloss =
            [&rotationMatrix, &transMatrix](const Eigen::Vector3d &src,
                                            const Eigen::Vector3d &tgt)
        {
            auto transVec = rotationMatrix * src + transMatrix;
            return (tgt - transVec).norm();
        };

        for (int i = 0; i < srcPoint.size(); i++)
        {
            Eigen::Vector3d srcVec(srcPoint[i].x, srcPoint[i].y, srcPoint[i].z);
            Eigen::Vector3d tgtVec(tgtPoint[i].x, tgtPoint[i].y, tgtPoint[i].z);
            loss[i] = getloss(srcVec, tgtVec);
        }
    }
    catch (...)
    {
        return -1;
    }
    return 0;
};

// Eigen::VectorXd LM::calculatedelta()
// {

//     int iter = 0;
//     double preloss, curloss;
//     preloss = std::numeric_limits<double>::min();
//     while (iter < this->maxIter)
//     {
//         std::cout << "iter:" << iter << std::endl;
//         Eigen::VectorXd temploss(op->srcSize());
//         op->operator()(this->trans, temploss);
//         curloss = temploss.norm();
//         if (std::abs(curloss - preloss) < minLossThreadshold)
//         {
//             break;
//         }
//         preloss = curloss;

//         auto jacobianMatrix = computeJacobian();
//         auto heissanMatrix = jacobianMatrix.transpose() * jacobianMatrix;
//         auto jabobianmulloss = jacobianMatrix.transpose() * temploss;

//         Eigen::MatrixXd tempSumLoss =
//             heissanMatrix +
//             lamda * (Eigen::MatrixXd::Identity(heissanMatrix.rows(),
//                                                heissanMatrix.cols()));
//         // auto deltaMatrix =
//         //     tempSumLoss.colPivHouseholderQr().solve(-jabobianmulloss);
//         std::cout << "************" << std::endl;
//         Eigen::VectorXd deltaMatrix =
//             tempSumLoss.colPivHouseholderQr().solve(-jabobianmulloss);
//         deltaMatrix.resize(6);
//         this->trans.resize(6);
//         std::cout << "dsgfdsfdsfds:" << deltaMatrix.rows() << " "
//                   << deltaMatrix.cols() << std::endl;
//         Eigen::VectorXd transNew(6);
//         // transNew.setZero();

//         std::cout << "row of trans: " << trans.rows() << "cols of trans"
//                   << trans.cols() << std::endl;
//         std::cout << "row of delta: " << deltaMatrix.rows()
//                   << "cols of delta:" << deltaMatrix.cols() << std::endl;
//         std::cout << "row of delta: " << transNew.rows()
//                   << "cols of delta:" << transNew.cols() << std::endl;
//         for (int i = 0; i < 6; i++)
//         {
//             std::cout << " transNew: " << transNew[i] << std::endl;
//             std::cout << " tras: " << this->trans[i] << std::endl;
//             std::cout << "deltaMatrix: " << deltaMatrix[i] << " " <<
//             std::endl; transNew[i] = deltaMatrix[i] + this->trans[i];
//         }
//         // transNew= deltaMatrix.col(0) + this->trans;

//         Eigen::VectorXd temploss2;
//         op->operator()(transNew, temploss2);
//         if (temploss2.norm() < curloss)
//         {
//             this->trans = transNew;
//             lamda *= 0.5;
//         }
//         else
//         {
//             lamda *= 5;
//             continue;
//         }
//         iter++;
//     }
//     return this->trans;
// }
