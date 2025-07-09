#include "../include/Kalmanfilter.h"
#include <eigen3/Eigen/src/Core/Matrix.h>
Kalmanfileter::Kalmanfileter(const int &stateDim, const int &measureDim,
                             const int &controlDim)
    : stateDimension(stateDim), measureDimension(measureDim),
      controlDimension(controlDim)
{
    this->x.resize(stateDimension);
    this->x.setZero();

    this->A.resize(stateDimension, stateDimension);
    this->A.setIdentity();

    this->u.resize(controlDimension);
    this->u.transpose();
    u.setZero();

    this->B.resize(stateDimension, controlDimension);
    this->B.setZero();

    //误差协方差矩阵；
    this->P.resize(stateDimension, stateDimension);
    this->P.setIdentity();

    this->H.resize(measureDimension, measureDimension);
    this->H.setZero();

    //过程噪声
    this->Q.resize(stateDimension, stateDimension);
    this->Q.setZero();

    //观测噪声；
    this->R.resize(measureDimension, measureDimension);
    this->R.setZero();
}

void Kalmanfileter::init(Eigen::VectorXd &x, Eigen::MatrixXd &p,
                         Eigen::MatrixXd &R, Eigen::MatrixXd &Q)
{
    this->x = x;
    this->P = p;
    this->R = R;
    this->Q = Q;
}

Eigen::VectorXd &Kalmanfileter::predict(Eigen::MatrixXd &A)
{
    this->A = A;
    this->x = A * this->x;
    this->P = this->A * this->P * this->A.transpose();
    return this->x;
}

Eigen::VectorXd &Kalmanfileter::predirct(Eigen::MatrixXd &A, Eigen::MatrixXd &B,
                                         Eigen::MatrixXd &u)
{
    this->A = A;
    this->B = B;
    this->u = u;
    this->x = this->A * this->x + this->B * this->u;
    this->P = this->A * this->P * this->A.transpose() + this->Q;
    return this->x;
}

void Kalmanfileter::update(Eigen::MatrixXd &H, Eigen::VectorXd &z)
{
    this->H = H;
    Eigen::MatrixXd cov = this->H * this->P * this->H.transpose() + this->R;
    Eigen::MatrixXd kalman = this->P * this->H.transpose() * cov.transpose();
    this->z = this->H * this->x;
    this->x = this->x + kalman * (z - this->z);
    Eigen::MatrixXd Iden =
        Eigen::MatrixXd::Identity(stateDimension, stateDimension);
    this->P = (Iden - kalman * this->H) * this->P;
}
