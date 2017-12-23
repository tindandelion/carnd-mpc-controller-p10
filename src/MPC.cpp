#include "MPC.hpp"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

size_t N = 15;
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

std::string ipopt_options() {
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";
  return options;
}

class FG_eval {
  const Eigen::VectorXd& coeffs;
  double ref_speed;
public:
  FG_eval(const Eigen::VectorXd& coeffs, double ref_speed): coeffs(coeffs), ref_speed(ref_speed) {}

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  AD<double> cost(const ADvector& vars) {
    AD<double> result = 0;

    for (int i = 0; i < N; i++) {
      result += (100*CppAD::pow(vars[cte_start + i], 2) +
		 100*CppAD::pow(vars[epsi_start + i], 2) +
		 0.1 * CppAD::pow(vars[v_start + i] - ref_speed, 2));
    }

    // Minimize the use of actuators.
    for (int i = 0; i < N - 1; i++) {
      result += 100*CppAD::pow(vars[delta_start + i], 2);
      result += CppAD::pow(vars[a_start + i], 2);
    }

    for (int i = 0; i < N - 2; i++) {
      result += 100*CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      result += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }

    // Punish for high speed with sharp steering angles
    for (int i = 0; i < N-1; i++) {
      result += 20*CppAD::pow(vars[delta_start+i] * vars[v_start+i], 2);
    }
    
    return result;
  }
  
  void operator()(ADvector& fg, const ADvector& vars) {
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    fg[0] = cost(vars);
    //
    // Setup Constraints
    //
    // NOTE: In this section you'll setup the model constraints.

    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int t = 1; t < N; t++) {
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];
      
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];
      
      AD<double> a = vars[a_start + t - 1];
      AD<double> delta = vars[delta_start + t - 1];
      
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // NOTE: The use of `AD<double>` and use of `CppAD`!
      // This is also CppAD can compute derivatives and pass
      // these to the solver.

      // TODO: Setup the rest of the model constraints
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0/Lf * delta * dt);
      fg[1 + v_start + t] = v1 - (v0 + a * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0/Lf * delta * dt);
    }
  }
};

VectorXd MPC::ApplyLatency(const VectorXd& state, const VectorXd& actuators) {
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  double delta = actuators[0];
  double a = actuators[1];

  VectorXd new_state(6);

  new_state[0] = x + v * cos(psi) * _latency;
  new_state[1] = y + v * sin(psi) * _latency;
  new_state[2] = psi - v/Lf * delta * _latency;
  new_state[3] = v + a * _latency;
  new_state[4] = cte + v * sin(epsi) * _latency;
  new_state[5] = epsi - v/Lf * delta * _latency;

  return new_state;
}

Solution MPC::Solve(const VectorXd& state, const VectorXd& actuators, const VectorXd& coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  size_t n_vars = N * 6 + (N - 1) * 2;
  size_t n_constraints = N * 6;

  VectorXd init_state = ApplyLatency(state, actuators);

  double x = init_state[0];
  double y = init_state[1];
  double psi = init_state[2];
  double v = init_state[3];
  double cte = init_state[4];
  double epsi = init_state[5];

  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = constraints_upperbound[x_start] = x;
  constraints_lowerbound[y_start] = constraints_upperbound[y_start] = y;
  constraints_lowerbound[psi_start] = constraints_upperbound[psi_start] = psi;
  constraints_lowerbound[v_start] = constraints_upperbound[v_start] = v;
  constraints_lowerbound[cte_start] = constraints_upperbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, _ref_speed);

  CppAD::ipopt::solve_result<Dvector> solution;
  CppAD::ipopt::solve<Dvector, FG_eval>(ipopt_options(), vars,
					vars_lowerbound, vars_upperbound,
					constraints_lowerbound, constraints_upperbound,
					fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  Solution result(solution.x[delta_start], solution.x[a_start]);
  for (int i = 1; i < N; i++) {
    result.AddPoint(solution.x[x_start + i], solution.x[y_start + i]);
  }

  return result;
}
