#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

struct XYPath {
    vector<double> x;
    vector<double> y;
};

struct Pair {
    double x;
    double y;
};

struct MPC_Solution {
    vector<double> variables;
    XYPath path;
    XYPath fit;
};

double polyeval(Eigen::VectorXd coeffs, double x);

class MPC {
public:
    MPC();

    virtual ~MPC();

    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuatotions.
    MPC_Solution Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
