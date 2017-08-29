#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);                                    
};

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
extern double deg2rad(double x);
extern double rad2deg(double x);

#endif /* MPC_H */
