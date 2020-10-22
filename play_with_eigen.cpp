#include <iostream>
#include <Eigen/Dense>

#include "Eigen/Eigen"  // AFTER GRIDMAP!
#include <Eigen/Core>
#include <unsupported/Eigen/Splines>


#include <chrono>

#include "spline.h"

class SplineFunction {

  public:
              
      // The spline is used to interpolate antenna gain values, as we only have the graphs
      SplineFunction()
      {}

      SplineFunction(Eigen::VectorXd const &x_vec, Eigen::VectorXd const &y_vec)
        : x_min(x_vec.minCoeff()),
          x_max(x_vec.maxCoeff()),
          y_min(y_vec.minCoeff()),
          y_max(y_vec.maxCoeff()),
          // Spline fitting here. X values are scaled down to [0, 1] for this.
          spline_(Eigen::SplineFitting<Eigen::Spline<double, 1>>::Interpolate(y_vec.transpose(), std::min<int>(x_vec.rows() - 1, 6), scaled_values(x_vec)))  // No more than cubic spline, but accept short vectors.
      {}

      // x values need to be scaled down in extraction as well.
      double interpDeg(double x) const {      
        double y;
        y = spline_(scaled_value(x))(0);

        // interpolation may produce values bigger and lower than our limits ...          
        y = std::max(std::min(y, y_max), y_min );
        return y;
      }

      double interpRad(double x) const {
          return interpDeg(x*180.0/M_PI); 
      }

      // Helpers to scale X values down to [0, 1]
      double scaled_value(double x) const {
        return (x - x_min) / (x_max - x_min);
      }

    private: 
      Eigen::RowVectorXd scaled_values(Eigen::VectorXd const &x_vec) const {
        return x_vec.unaryExpr([this](double x) { return scaled_value(x); }).transpose();
      }

      double x_min;
      double x_max;
      double y_min;
      double y_max;

      // Spline of one-dimensional "points."
      Eigen::Spline<double, 1> spline_;
};

/////////////////////////////



// quick build:
// g++ -I /usr/include/eigen3/ play_with_eigen.cpp -o play_with_eigen -std=c++11

using namespace std::placeholders;
using Eigen::MatrixXd;

const double C = 299792458.0;
const double SENSITIVITY = -115; // dB    

const double TAG_LOSSES = -4.8;

const double LOSS_CONSTANT = 147.55;
const double freq= 865e6; 
const double lambda =  C/freq;
const double ANTENNA_LOSSES_LIST [25] = {  -22.6, -25.2, -25, -20.2, -17.6, -15.6, -14, -11.2, -7.8, -5.2, -2.4, -0.6, 0, -0.6, -2.4, -5.2, -8.8, -12.2, -16.4, -19.2, -20.8, -24.4, -28.2, -24, -22.6};
const double ANTENNA_ANGLES_LIST [25] = {-180.0, -165.0, -150.0, -135.0, -120.0, -105.0, -90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 105.0, 120.0, 135.0, 150.0, 165.0, 180.0};

//! Generates a mesh, just like Matlab's meshgrid
//  Template specialization for column vectors (Eigen::VectorXd)
//  in : x, y column vectors 
//       X, Y matrices, used to save the mesh
template <typename Scalar>
void meshgrid(const Eigen::Matrix<Scalar, -1, 1>& x, 
              const Eigen::Matrix<Scalar, -1, 1>& y,
              Eigen::Matrix<Scalar, -1, -1>& X,
              Eigen::Matrix<Scalar, -1, -1>& Y) {
  const long nx = x.size(), ny = y.size();
  X.resize(ny, nx);
  Y.resize(ny, nx);
  for (long i = 0; i < ny; ++i) {
    X.row(i) = x.transpose();
  }

  // for (long j = 0; j < nx; ++j) {
  //   Y.col(j) = y;
  // }
  for (long j = 0; j < nx; ++j) {
    Y.col(j) = y.reverse();
  }
}


//! Generates a mesh, just like Matlab's meshgrid
//  Template specialization for row vectors (Eigen::RowVectorXd)
//  in : x, y row vectors 
//       X, Y matrices, used to save the mesh
template <typename Scalar>
void meshgrid(const Eigen::Matrix<Scalar, 1, -1>& x, 
              const Eigen::Matrix<Scalar, 1, -1>& y,
              Eigen::Matrix<Scalar, -1, -1>& X,
              Eigen::Matrix<Scalar, -1, -1>& Y) {
  Eigen::Matrix<Scalar, -1, 1> xt = x.transpose(),
                               yt = y.transpose();
  meshgrid(xt, yt, X, Y);
}


typedef Eigen::VectorXd Vec;
typedef Eigen::MatrixXd Mat;

int main(int argc, char **argv)
{
    int Nx, Ny, x_min, x_max, y_min, y_max;
    double x_m, y_m;
    Mat X, Y, R, A, propL, antL,totalLoss, rxPower;
    Vec x,y;

    double txPower = 0; //dB

    

    /////////////////////////        // build spline to interpolate antenna gains;
    std::vector<double> xVec(ANTENNA_ANGLES_LIST, ANTENNA_ANGLES_LIST + 25);
    Eigen::VectorXd xvals = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(xVec.data(), xVec.size());
    std::vector<double> yVec(ANTENNA_LOSSES_LIST, ANTENNA_LOSSES_LIST + 25);
    Eigen::VectorXd yvals= Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(yVec.data(), yVec.size());
    SplineFunction _antenna_gains= SplineFunction(xvals, yvals);

    
    double a0[xvals.size()];
    double b0[yvals.size()];
    std::vector<double> tmp_X(xvals.size()), tmp_Y(yvals.size());
    for(int i=0; i<xvals.size(); i++){
      a0[i] = xvals[i];
      b0[i] = yvals[i];
      tmp_X[i] = xvals[i];
      tmp_Y[i] = yvals[i];
    }
    int size = xvals.size();
    ////////////////////////

    tk::spline new_spline;
    new_spline.set_points(tmp_X, tmp_Y);

    ////////////////////////

    Nx = 5;
    Ny = 8;
    x_min = -1;
    x_max = 1;
    y_min = -2;
    y_max = 2;
    x_m = 0.5;
    y_m = 1.2;

    x = Vec::LinSpaced(Nx, x_min, x_max);
    y = Vec::LinSpaced(Ny, y_min, y_max);

    std::cout << "x" << std::endl;
    std::cout << x << std::endl;
    std::cout << "y" << std::endl;
    std::cout << y << std::endl;

    // create X,Y meshgrids
    meshgrid(x, y, X, Y);

    // distance to point m
    X = X.array() - x_m;
    Y = Y.array() - y_m;

    std::cout << "X" << std::endl;
    std::cout << X << std::endl;
    std::cout << "Y" << std::endl;
    std::cout << Y << std::endl;

    // create R,Ang matrixes
    R = (X.array().square() + Y.array().square()).array().sqrt();
    A = Y.binaryExpr(X, std::ptr_fun(atan2));

    std::cout << "R" << std::endl;
    std::cout << R << std::endl;

    std::cout << "A" << std::endl;
    std::cout << (A*180.0/3.141592) << std::endl;

    // Create a propagation matrix without taking obstacles        
    
    auto funtor = std::bind(&SplineFunction::interpRad, _antenna_gains, _1) ;


    // double tmp = 5;
    // std::cout << "1: " << _antenna_gains.interpRad(tmp) << std::endl;
    
    // std::cout << "Value: " << tmp << std::endl;
    // // tmp = _antenna_gains.scaled_value(tmp * 180/M_PI);
    // std::cout << "Scaled value: " << tmp << std::endl;
    // std::cout << "Spline: " << tmp << std::endl;
    // tmp = std::max(std::min(tmp, (double) yvals.maxCoeff()), (double)yvals.minCoeff() );
    // std::cout << "2: " << tmp << std::endl;
    
    auto start = std::chrono::high_resolution_clock::now();
    // A = A * 180/M_PI;
    // double tmp;
    // for (int i=0; i < A.size(); i++){
    //   tmp = A(i) * 180/M_PI;
    //   tmp = new_spline(tmp);
    //   tmp = std::max(std::min(tmp, (double) yvals.maxCoeff()), (double)yvals.minCoeff() );
    //   A(i) = tmp;
    // }
    // antL =  TAG_LOSSES + A.array();
    
    antL =  TAG_LOSSES + A.unaryExpr( funtor ).array();    
    auto stop = std::chrono::high_resolution_clock::now();
    double totalTime = std::chrono::duration<double,std::milli> ( stop - start ).count();
    std::cout << "Execution time[s]: " << totalTime << std::endl;
    
    
    std::cout << "antL" << std::endl;
    std::cout << antL << std::endl;
    exit(0);

    propL = LOSS_CONSTANT - (20.0 * (R * freq).unaryExpr(std::ptr_fun(log10))).array() ;
    std::cout << "propL" << std::endl;
    std::cout << propL << std::endl;

    // signal goes from antenna to tag and comes back again, so we double the losses
    totalLoss =  2*antL + 2*propL;
    std::cout << "totalLoss" << std::endl;
    std::cout << totalLoss << std::endl;
    
    rxPower = txPower + totalLoss.array(); 

    // this should remove points where friis is not applicable
    rxPower = (R.array()>2*lambda).select(rxPower,SENSITIVITY); 
    // this should remove points where received power is too low
    rxPower = (rxPower.array()>SENSITIVITY).select(rxPower,SENSITIVITY); 

    std::cout << "rxPower" << std::endl;
    std::cout << rxPower << std::endl;



}