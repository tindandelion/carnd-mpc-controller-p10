
#include "ReferenceTrajectory.hpp"

using Eigen::MatrixXd;

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
VectorXd polyfit(VectorXd xvals, VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }
  
  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }
  
  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

void ReferenceTrajectory::ConvertToLocal(const vector<double>& xx, const vector<double>& yy,
					 VectorXd& out_xx, VectorXd& out_yy) {
  for (int i = 0; i < xx.size(); i++) {
    double dx = xx[i] - _origin_x;
    double dy = yy[i] - _origin_y;

    out_xx[i] = dx * cos(_origin_angle) + dy * sin(_origin_angle);
    out_yy[i] = -dx * sin(_origin_angle) + dy * cos(_origin_angle);
  }
}


void ReferenceTrajectory::Fit(const vector<double>& xx, const vector<double>& yy) {
  VectorXd local_xx(xx.size());
  VectorXd local_yy(yy.size());

  ConvertToLocal(xx, yy, local_xx, local_yy);
  _coeffs = polyfit(local_xx, local_yy, 3);
}

void ReferenceTrajectory::GeneratePath(int count, vector<double>& out_x, vector<double>& out_y) {
  int step = 3;
  for (double i = 0; i < 100; i += step){
    out_x.push_back(i);
    out_y.push_back(polyeval(_coeffs, i));
  }
}
