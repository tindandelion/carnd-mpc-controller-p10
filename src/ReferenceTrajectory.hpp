#ifndef __REFERENCE_TRAJECTORY_H
#define __REFERENCE_TRAJECTORY_H

#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"


using std::vector;
using Eigen::VectorXd;


class ReferenceTrajectory {
  double _origin_x = 0;
  double _origin_y = 0;
  double _origin_angle = 0;
  VectorXd _coeffs = VectorXd(0);
  

  void ConvertToLocal(const vector<double>& xx, const vector<double>& yy,
		      VectorXd& out_xx, VectorXd& out_yy);
  
public:

  void SetOrigin(double x, double y, double angle) {
    _origin_x = x;
    _origin_y = y;
    _origin_angle = angle;
  }

  void Fit(const vector<double>& xx, const vector<double>& yy);
  void GeneratePath(int count, vector<double>& out_x, vector<double>& out_y);
};

#endif
