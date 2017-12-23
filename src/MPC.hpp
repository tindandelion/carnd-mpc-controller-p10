#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class Solution {
  double _delta;
  double _a;
  vector<double> _xx;
  vector<double> _yy;
  
public:
  Solution(double delta, double a): _delta(delta), _a(a) {}
  void AddPoint(double x, double y) {
    _xx.push_back(x);
    _yy.push_back(y);
  }

  double a() const { return _a; }
  double delta() const { return _delta; }
  const vector<double>& xx() const { return _xx; }
  const vector<double>& yy() const { return _yy; }
};

class MPC {
 public:
  Solution Solve(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs);
};

#endif /* MPC_H */
