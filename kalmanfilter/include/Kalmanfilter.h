
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include <string>
#include <vector>

using namespace std;
using namespace Eigen;
/**
 * @brief 定义卡尔曼滤波类，在类中添加预测更新成员函数；
 *
 */
class Kalmanfileter
{

  public:
    Kalmanfileter(const int &stateDim = 0, const int &measureDim = 0,
                  const int &controlDim = 0);
    ~Kalmanfileter() = default;
    void init(Eigen::VectorXd &x, Eigen::MatrixXd &p, Eigen::MatrixXd &R,
              Eigen::MatrixXd &Q);
    Eigen::VectorXd &predict(Eigen::MatrixXd &A);
    Eigen::VectorXd &predirct(Eigen::MatrixXd &A, Eigen::MatrixXd &B,
                              Eigen::MatrixXd &u);
    void update(Eigen::MatrixXd &H, Eigen::VectorXd &z);

  private:
    int stateDimension;   //状态维度；
    int measureDimension; //测量维度；
    int controlDimension; //控制维度；

    Eigen::VectorXd x; //记录状态方程；
    Eigen::VectorXd z; //记录观测方程；
    Eigen::VectorXd u; //记录控制方程；

    Eigen::MatrixXd A, B, P, Q, H, R;
};
