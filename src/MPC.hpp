#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;
using Eigen::VectorXd;

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
  double _latency;
  double _ref_speed;
  VectorXd ApplyLatency(const VectorXd& state, const VectorXd& actuators);
public:
  MPC(double latency, double ref_speed): _latency(latency), _ref_speed(ref_speed) {}
  Solution Solve(const VectorXd& state, const VectorXd& actuators, const VectorXd& coeffs);
};

#endif /* MPC_H */
