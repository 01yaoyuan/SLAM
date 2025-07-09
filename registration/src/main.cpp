#include "optimize.h"
#include <Eigen/Geometry>
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/unsupported/Eigen/src/NonLinearOptimization/LevenbergMarquardt.h>
#include <eigen3/unsupported/Eigen/src/NumericalDiff/NumericalDiff.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <vector>
using namespace std;
using namespace pcl;
using namespace Eigen;
#define PI 3.1415926
int main()
{
    std::cout << "tst";
    // std::vector<double> trans = {PI / 4, 0, 0};
    Eigen::VectorXd trans(6);
    // trans.resize(6);
    trans << PI / 4, 0, 0, 1, 1, 1;
    Eigen::Quaterniond res = Eigen::Quaterniond(
        Eigen::AngleAxisd(trans[0], Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(trans[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(trans[2], Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d rotarion = res.toRotationMatrix();
    std::cout << rotarion << endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudtransformed(
        new pcl::PointCloud<pcl::PointXYZ>);

    std::string pcdfilePath = "../bunny.pcd";
    std::string outputfilePath = "../bunnttransformed.pcd";

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdfilePath, *pointCloud) == -1)
    {
        std::cout << "Failed to load pcd file" << std::endl;
    }

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    double theta = PI / 4;
    transform << 1, 0, 0, 1, 0, cos(theta), -sin(theta), 1, 0, sin(theta),
        cos(theta), 1, 0, 0, 0, 1;
    ;

    pcl::transformPointCloud(*pointCloud, *pointCloudtransformed, transform);

    // if (pcl::io::savePCDFileASCII(outputfilePath, *pointCloudtransformed) ==
    // -1)
    // {
    //     std::cout << "Failed to save pcd file" << std::endl;
    //     return 0;
    // }
    // // 创建迭代最近点（ICP）实例
    // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // icp.setInputSource(pointCloud); // 将 cloud_in 设置为输入点云
    // icp.setInputTarget(pointCloudtransformed); // 将 cloud_out 设置为目标点云

    // // 执行 ICP 算法并将结果存储到 Final 点云中
    // pcl::PointCloud<pcl::PointXYZ> Final;
    // icp.align(Final);

    // // 输出 ICP 算法是否收敛以及得分（匹配度）
    // std::cout << "has converged:" << icp.hasConverged()
    //           << " score: " << icp.getFitnessScore() << std::endl;

    // // 输出最终变换矩阵
    // const pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4
    //     &matrix = icp.getFinalTransformation();
    // std::cout << "最终变换矩阵：" << std::endl << matrix << std::endl;

    std::vector<Point> src, tgt;

    for (auto &&t : *pointCloud)
    {
        src.emplace_back(Point(t.x, t.y, t.z));
    }

    for (auto &&t : *pointCloudtransformed)
    {
        tgt.emplace_back(Point(t.x, t.y, t.z));
    }

    std::cout << "test" << std::endl;
    if (!src.empty() && !tgt.empty())
    {
        optimize op(src, tgt);
        std::cout << "consttuct";
        // 确保源点和目标点的数量相同
        if (src.size() != tgt.size())
        {
            std::cerr << "Source and target point clouds have different sizes."
                      << std::endl;
            return -1;
        }
        Eigen::VectorXd transVec(6);
        transVec.setZero();
        std::cout << "dirff===";
        // transVec[0] = PI / 4;
        // LM *test = new LM(op, trans);
        Eigen::NumericalDiff<optimize> diff(op);
        std::cout << "lm";
        Eigen::LevenbergMarquardt<Eigen::NumericalDiff<optimize>, double> lm(
            diff);
        std::cout << "min";
        int ret = lm.minimize(transVec);
        std::cout << "result status" << ret << std::endl;
        std::cout << "res trans" << transVec << std::endl;
    }
    else
    {
        std::cout << "error";
        return -1;
    }

    // auto transMatrix = test->calculatedelta();
    // for (auto &&item : transMatrix)
    // {
    //     std::cout << item << std::endl;
    // }

    // Eigen::Matrix3d rotationMatrix;
    // rotationMatrix =
    //     Eigen::AngleAxisd(transMatrix[0], Eigen::Vector3d::UnitX()) *
    //     Eigen::AngleAxisd(transMatrix[1], Eigen::Vector3d::UnitY()) *
    //     Eigen::AngleAxisd(transMatrix[2], Eigen::Vector3d::UnitZ());

    // std::cout << rotationMatrix << std::endl;

    // auto temp = test;
    // test = nullptr;
    // free(temp);

    return 0;
}