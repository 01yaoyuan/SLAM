#ifndef OPTIMIZE_H_INCLUDED
#define OPTIMIZE_H_INCLUDED
#pragma once
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/unsupported/Eigen/NonLinearOptimization>
#include <eigen3/unsupported/Eigen/NumericalDiff>
#include <iostream>
#include <memory>
#include <vector>

using namespace std;
using namespace Eigen;

// using namespace PCL;
// select a kit for registration
struct Point
{
    Point();
    Point(double a, double b, double c) : x(a), y(b), z(c){};
    ~Point() = default;
    double x;
    double y;
    double z;
};

class optimize
{

  public:
    // typedef double Scalar;
    typedef double Scalar;             // 明确指定Scalar类型
    typedef Eigen::VectorXd InputType; // 输入向量类型
    typedef Eigen::VectorXd OutputType;
    typedef Eigen::VectorXd ValueType;
    typedef Eigen::MatrixXd JacobianType;
    enum
    {
        InputsAtCompileTime = Eigen::Dynamic,
        ValuesAtCompileTime = Eigen::Dynamic
    };
    optimize();
    optimize(const std::vector<Point> &src, const std::vector<Point> &tgt)
        : srcPoint(src), tgtPoint(tgt){};
    ~optimize() = default;
    int operator()(const Eigen::VectorXd &trans, Eigen::VectorXd &loss) const;
    // int srcSize();

    int inputs() const { return 6; }                     // 输入变量数量
    int values() const { return this->srcPoint.size(); } // 输出值数量

  private:
    std::vector<Point> srcPoint;
    std::vector<Point> tgtPoint;
};

// class LM
// {
//   public:
//     LM();
//     LM(const optimize &op, Eigen::VectorXd &matrix, int iter, double lamda,
//        double mintlossthreashold)
//         : op(std::make_shared<optimize>(op)), trans(matrix), maxIter(iter),
//           lamda(lamda), minLossThreadshold(mintlossthreashold){};
//     LM(const optimize &op, Eigen::VectorXd &trans)
//         : op(std::make_shared<optimize>(op)), trans(trans){};
//     Eigen::VectorXd calculatedelta();

//     /**
//      * @brief 计算未转置的雅可比矩阵（n*6）
//      *  计算出来的雅可比矩阵的转置矩阵（6*n）为关于delta的一阶导数；
//      * @return Eigen::MatrixXd
//      */
//     inline Eigen::MatrixXd computeJacobian()
//     {
//         double epsilon = 1e-5;
//         Eigen::MatrixXd Jacobian(op->srcSize(), trans.rows());

//         Eigen::VectorXd tempTrans, transplus, transsub;
//         Eigen::VectorXd transplusloss, transubloss;

//         for (int i = 0; i < trans.size(); i++)
//         {
//             tempTrans = trans;
//             Eigen::VectorXd tempplus(tempTrans.rows());
//             tempplus.setConstant(epsilon);

//             transplus = tempTrans + tempplus;
//             transsub = tempTrans - tempplus;

//             auto lossplus = op->operator()(transplus, transplusloss);

//             auto losssub = op->operator()(transsub, transubloss);
//             if (losssub == 0 && lossplus == 0)
//             {
//                 std::cout << "Get loss succeend" << std::endl;
//             }
//             else
//             {
//                 std::cout << "Failed to get loss" << std::endl;
//             }
//             Jacobian.col(i) = (transplusloss - transubloss) / (2 * epsilon);
//         }
//         return Jacobian;
//     }

//     ~LM() = default;

//   private:
//     double lamda = 0.01;
//     int maxIter = 100;
//     std::shared_ptr<optimize> op;
//     Eigen::VectorXd trans;
//     double minLossThreadshold = 1e-5;
// };

#endif // OPTIMIZE_H_INCLUDED
