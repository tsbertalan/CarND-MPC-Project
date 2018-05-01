#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Set the timestep length and duration
size_t N = 12;
double dt = 0.1;

// NOTE: feel free to play around with this
// or do something completely different
double ref_v = 6;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;





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

// Evaluate a polynomial.
AD<double> polyevalAD(Eigen::VectorXd coeffs, AD<double> x) {
    AD<double> result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * CppAD::pow(x, i);
    }
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

class FG_eval {
public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;

    FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector &fg, const ADvector &vars) {
        // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
        // NOTE: You'll probably go back and forth between this function and
        // the Solver function below.
        // The cost is stored is the first element of `fg`.
        // Any additions to the cost should be added to `fg[0]`.
        fg[0] = 0;

        // Reference State Cost
        // Define the cost related the reference state and
        // any anything you think may be beneficial.
        for (unsigned int t = 0; t < N; t++) {
            // Minimize the CTE.
            fg[0] += CppAD::pow(vars[cte_start + t], 2);

            // Minimize the angular error.
            fg[0] += CppAD::pow(vars[epsi_start + t], 2);

            // Minimize the difference from reference velocity.
            fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);

            if (t < N - 1) {
                // Minimize the control actions.
                fg[0] += CppAD::pow(vars[delta_start + t], 2);
                fg[0] += CppAD::pow(vars[a_start + t], 2);

                if (t < N - 2) {
                    // Minimize the abruptness of control action changes.
                    fg[0] += 100 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
                    fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
                }
            }


        }



        //
        // Setup Constraints
        //
        // NOTE: In this section you'll setup the model constraints.

        // Initial constraints
        //
        // We add 1 to each of the starting indices due to cost being located at
        // index 0 of `fg`.
        // This bumps up the position of all the other values.
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        fg[1 + v_start] = vars[v_start];
        fg[1 + cte_start] = vars[cte_start];
        fg[1 + epsi_start] = vars[epsi_start];

        // The rest of the constraints
        for (unsigned int t = 1; t < N; t++) {
            AD<double> x0 = vars[x_start + t - 1];
            AD<double> x1 = vars[x_start + t];

            AD<double> y0 = vars[y_start + t - 1];
            AD<double> y1 = vars[y_start + t];

            AD<double> psi0 = vars[psi_start + t - 1];
            AD<double> psi1 = vars[psi_start + t];

            AD<double> v0 = vars[v_start + t - 1];
            AD<double> v1 = vars[v_start + t];

            AD<double> cte0 = vars[cte_start + t - 1];
            AD<double> cte1 = vars[cte_start + t];

            AD<double> epsi0 = vars[epsi_start + t - 1];
            AD<double> epsi1 = vars[epsi_start + t];

            AD<double> d0 = vars[delta_start + t - 1];
            AD<double> a0 = vars[a_start + t - 1];
            AD<double> yDesired0 = polyevalAD(coeffs, x0);
            AD<double> psiDesired0 = CppAD::atan(coeffs[1]);

            // NOTE: The use of `AD<double>` and use of `CppAD`!
            // This is also CppAD can compute derivatives and pass
            // these to the solver.

            // Setup the rest of the model constraints
            fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[1 + v_start + t] = v1 - (v0 + a0 * dt);

            // CTE at the next step is the previous CTE plus the growth due to current psi error.
            fg[1 + cte_start + t] = cte1 - ((yDesired0 - y0) + (v0 * CppAD::sin(epsi0) * dt));

            // Psi and psi error both grow their current values due to the turning rate.
            auto dpsi = v0 * d0 / Lf * dt;
            fg[1 + psi_start + t] = psi1 - (psi0 + dpsi);
            auto epsi0_calc = psi0 - psiDesired0; // Why isn't this just epsi0?
            fg[1 + epsi_start + t] = epsi1 - (epsi0_calc + dpsi);
        }
    }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}

MPC::~MPC() {}

MPC_Solution MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    // Set the number of model variables (includes both states and inputs).
// (State is (x, y, psi, v, cte, epsi) and actuators is (d, a).
    size_t n_vars = 6 * N + 2 * (N - 1);
    // Set the number of constraints
    size_t n_constraints = N * 6;

    double x = state[0];
//    double y = state[1];
//    double psi = state[2];
    double v = state[3];
    double cte = state[4];
    double epsi = state[5];



    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }

    // Set lower and upper limits for variables.
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (i = 0; i < delta_start; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians).
    // NOTE: Feel free to change this to something else.
    for (i = delta_start; i < a_start; i++) {
        vars_lowerbound[i] = -0.436332;
        vars_upperbound[i] = 0.436332;
    }

    // Acceleration/decceleration upper and lower limits.
    // NOTE: Feel free to change this to something else.
    for (i = a_start; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[x_start] = 0;
    constraints_lowerbound[y_start] = 0;
    constraints_lowerbound[psi_start] = 0;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound[x_start] = 0;
    constraints_upperbound[y_start] = 0;
    constraints_upperbound[psi_start] = 0;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    //
    // NOTE: You don't have to worry about these options
    //
    // options for IPOPT solver
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

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
            options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
            constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
//    auto cost = solution.obj_value;
//    std::cout << "Cost " << cost << std::endl;

    // Return the first actuator values. The variables can be accessed with


    MPC_Solution result;
    result.variables = {solution.x[x_start + 1], solution.x[y_start + 1],
                        solution.x[psi_start + 1], solution.x[v_start + 1],
                        solution.x[cte_start + 1], solution.x[epsi_start + 1],
                        solution.x[delta_start], solution.x[a_start]};
    for(i=0; i<N-1; i++) {
        double x_vehicle = solution.x[x_start+1+i];
        double y_vehicle = solution.x[y_start+1+i];
        result.path.x.push_back(x_vehicle);
        result.path.y.push_back(y_vehicle);

        double x_fit = x_vehicle;
        double y_fit = polyeval(coeffs, x);
        result.fit.x.push_back(x_fit);
        result.fit.y.push_back(y_fit);
    }


    return result;
}
